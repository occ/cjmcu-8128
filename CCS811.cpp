#include "CCS811.h"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

CCS811::CCS811(std::string i2c_dev_name, uint8_t ccs811_addr)
        : i2c_dev_name(std::move(i2c_dev_name)),
          ccs811_addr(ccs811_addr) {
    open_device();
    init();
}

CCS811::~CCS811() {
    close_device();
}

void CCS811::close_device() const { if (i2c_fd >= 0) close(i2c_fd); }

uint16_t CCS811::get_co2() {
    return co2;
}

uint16_t CCS811::get_tvoc() {
    return tvoc;
}

void CCS811::init() {
    std::cout << "Checking the hardware id..." << std::endl;
    auto hw_id = read_mailbox(HW_ID);
    if (hw_id->front() != 0x81) {
        std::cerr << "Unrecognized hardware id 0x" << std::hex << (int) hw_id->front() << std::endl;
        exit(-1);
    }

    std::cout << "Resetting CCS811..." << std::endl;
    uint8_t reset_sequence[] = {0x11, 0xe5, 0x72, 0x8a};
    write_to_mailbox(SW_RESET, reset_sequence, 4);

    std::cout << "Sleeping for a second..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto hw_version = read_mailbox(HW_VERSION);
    char version_str[15];
    version_to_str(hw_version->front(), version_str);
    std::cout << "HW Version: " << version_str << std::endl;

    auto fw_boot_ver = read_mailbox(FW_BOOT_VERSION);
    version_to_str(fw_boot_ver->front(), version_str);
    std::cout << "FW Boot Version: " << version_str << "." << (int) fw_boot_ver->at(1) << std::endl;

    auto fw_app_ver = read_mailbox(FW_APP_VERSION);
    version_to_str(fw_app_ver->front(), version_str);
    std::cout << "FW Application Version: " << version_str << "." << (int) fw_app_ver->at(1) << std::endl;

    std::cout << "Starting..." << std::endl;
    uint8_t buffer[] = {APP_START};
    write_data(buffer, 1);

    std::cout << "Configuring measurement mode to Mode 1 - Constant power mode, IAQ measurement every second"
              << std::endl;
    uint8_t measurement_mode[] = {1 << 4};
    write_to_mailbox(MEAS_MODE, measurement_mode, 1);
}

void CCS811::open_device() {
    i2c_fd = open(i2c_dev_name.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "Unable to open" << i2c_dev_name << ". " << strerror(errno) << std::endl;
        throw 1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, ccs811_addr) < 0) {
        std::cerr << "Failed to communicate with the device. " << strerror(errno) << std::endl;
        throw 1;
    }
}

std::unique_ptr<std::vector<uint8_t>> CCS811::read_mailbox(CCS811::Mailbox m) {
    auto mbox_info = mailbox_info(m);

    if (!mbox_info.readable) {
        std::cerr << "Mailbox is not writeable!" << std::endl;
        // TODO
        throw 1;
    }

    // Select the mailbox.
    uint8_t mailbox_id_buf[] = {mbox_info.id};
    write_data(mailbox_id_buf, 1);

    size_t buffer_len = mbox_info.size;
    auto *read_buffer = new uint8_t[buffer_len];
    auto bytes_read = read(i2c_fd, read_buffer, buffer_len);
    if (bytes_read != buffer_len) {
        std::cerr << "Failed to read from the device. Bytes read: " << bytes_read << std::endl;
        // TODO - Have better exceptions.
        throw 1;
    }

    auto result = std::make_unique<std::vector<uint8_t>>();
#ifdef DBG
    std::cerr << "Read: ";
#endif
    for (size_t i = 0; i < mbox_info.size; i++) {
        result->push_back(read_buffer[i]);
#ifdef DBG
        std::cerr << "0x" << hex << (int)read_buffer[i] << " ";
#endif
    }
#ifdef DBG
    std::cerr << std::endl;
#endif

    delete[] read_buffer;
    return std::move(result);
}

void CCS811::read_sensors() {
    auto status = read_mailbox(STATUS);
    // Check if the sensor is ready for a read.
    if (!(status->front() & 8)) {
        std::cerr << "Device isn't ready yet." << std::endl;
        return;
    }
    auto data = read_mailbox(ALG_RESULT_DATA);
    co2 = (data->at(0) << 8 | data->at(1));
    tvoc = (data->at(2) << 8 | data->at(3));
    last_measurement = time(nullptr);
}

void CCS811::write_data(uint8_t *buffer, size_t buffer_len) {
#ifdef DBG
    std::cout << "Write: ";
     for (size_t i = 0; i < buffer_len; i++) {
      std::cout << "0x" << hex << (int)buffer[i] << " ";
     }
    std::cout << std::endl;
#endif

    auto write_c = write(i2c_fd, buffer, buffer_len);
    if (write_c < 0) {
        std::cerr << "Unable to send command." << std::endl;
        // TODO - Have better exceptions.
        throw 1;
    }
#ifdef DBG
    std::cout << "  ... wrote " << write_c << " bytes." << std::endl;
#endif
}

void CCS811::write_to_mailbox(CCS811::Mailbox m, uint8_t *buffer, size_t buffer_len) {
    auto mbox_info = mailbox_info(m);
    if (!mbox_info.writeable) {
        std::cerr << "Mailbox is not writeable!" << std::endl;
        // TODO
        throw 1;
    }

    // Make sure that we leave room for the mailbox address and that we don't
    // write more than the mailbox can take.
    auto write_buf_len = std::min(buffer_len, mbox_info.size) + 1;
    uint8_t write_buffer[write_buf_len];
    write_buffer[0] = mbox_info.id;
    memcpy(&write_buffer[1], buffer, write_buf_len);
    write_data(write_buffer, write_buf_len);
}

// This is pretty unsafe.
int CCS811::version_to_str(uint8_t version, char *buffer) {
    int major = version >> 4;
    int minor = version & 0xF;
    return sprintf(buffer, "%d.%d", major, minor);
}
