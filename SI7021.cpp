#include "SI7021.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <chrono>
#include <thread>

#undef DBG

using namespace std;

SI7021::SI7021(std::string i2c_dev_name, uint8_t ccs811_addr)
        : i2c_dev_name(std::move(i2c_dev_name)),
          ccs811_addr(ccs811_addr) {
    open_device();
    init();
}

SI7021::~SI7021() {
    close_device();
}

void SI7021::close_device() { if (i2c_fd >= 0) close(i2c_fd); }

void SI7021::init() {
    cout << "Resetting Si7021..." << endl;
    reset();
    this_thread::sleep_for(chrono::seconds(1));

    read_serial();
    read_fw_rev();
}

void SI7021::open_device() {
    i2c_fd = open(i2c_dev_name.c_str(), O_RDWR);
    if (i2c_fd < 0) {
        cerr << "Unable to open" << i2c_dev_name << ". " << strerror(errno) << endl;
        throw 1;
    }

    if (ioctl(i2c_fd, I2C_SLAVE, ccs811_addr) < 0) {
        cerr << "Failed to communicate with the device. " << strerror(errno) << endl;
        throw 1;
    }
}

uint64_t SI7021::get_serial() {
    return serial_no;
}


void SI7021::write_data(uint8_t *buffer, size_t buffer_len) {
#ifdef DBG
    cout << "Write: ";
    for (size_t i = 0; i < buffer_len; i++) {
        cout << "0x" << hex << (int) buffer[i] << " ";
    }
    cout << endl;
#endif

    auto write_c = write(i2c_fd, buffer, buffer_len);
    if (write_c < 0) {
        cerr << "Unable to send command." << endl;
        // TODO - Have better exceptions.
        throw 1;
    }
#ifdef DBG
    cout << "  ... wrote " << write_c << " bytes." << endl;
#endif
}

std::unique_ptr<std::vector<uint8_t>> SI7021::read_data(size_t buffer_size) {
    auto *read_buffer = new uint8_t[buffer_size];
    auto bytes_read = read(i2c_fd, read_buffer, buffer_size);

#ifdef DBG
    cout << "Read " << std::dec << bytes_read << " bytes" << endl;
#endif
    if (bytes_read < 0) {
        // return an empty vector if we can't read anything.
        return std::make_unique<std::vector<uint8_t>>();
    }

    auto result = std::make_unique<std::vector<uint8_t>>(read_buffer, read_buffer + bytes_read);

    delete[] read_buffer;

#ifdef DBG
    std::cout << "Read: ";
    for (auto e : *result) {
        std::cout << std::hex << (int) e << " ";
    }
    std::cout << std::endl;
#endif
    return result;
}

void SI7021::reset() {
    uint8_t cmd[] = {RESET};
    write_data(cmd, 1);
}

// TODO - Implement the CRC logic
// From the documentation: "The checksum byte is calculated using a CRC generator polynomial of
// x^8 + x^5 + x^4 + 1, with an initialization of 0x00."
uint8_t SI7021::crc(uint8_t in) {
    return 0;
}

void SI7021::read_serial() {
    uint64_t serial = 0;

    uint8_t cmd[] = {0xfa, 0x0f};
    write_data(cmd, 2);
    auto response = read_data(8);

    // Skip the crc bytes
    for (size_t i = 0; i < response->size(); i += 2) {
        serial = (serial << 8) | response->at(i);
    }

    // Read the second part
    cmd[0] = 0xfc;
    cmd[1] = 0xc9;
    write_data(cmd, 2);
    response = read_data(6);

    serial = (serial << 8) | response->at(0);
    serial = (serial << 8) | response->at(1);
    serial = (serial << 8) | response->at(3);
    serial = (serial << 8) | response->at(4);

    serial_no = serial;
}

void SI7021::read_fw_rev() {
    uint8_t cmd[] = {0x84, 0x88};
    write_data(cmd, 2);
    auto response = read_data(1);

    fw_rev = response->front();
}

uint8_t SI7021::get_fw_rev() {
    return fw_rev;
}

float SI7021::measure_humidity() {
    uint8_t cmd[] = {MEAS_REL_HUM};
    write_data(cmd, 1);
    this_thread::sleep_for(chrono::seconds(1));

    auto response = read_data(2);
    if (response->empty()) return 0;
    uint16_t rh_code = (response->at(0) << 8) | response->at(1);
    return static_cast<float>(((125.0 * rh_code) / 65536) - 6);
}

float SI7021::measure_temperature() {
    uint8_t cmd[] = {MEAS_TEMP};
    write_data(cmd, 1);
    this_thread::sleep_for(chrono::seconds(1));

    auto response = read_data(2);
    if (response->empty()) return 0;
    uint16_t temp_code = (response->at(0) << 8) | response->at(1);
    return static_cast<float>(((175.72 * temp_code) / 65536) - 46.85);
}