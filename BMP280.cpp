#include "BMP280.h"

#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <thread>
#include <unistd.h>
#include <sys/ioctl.h>

#undef DBG

using namespace std;

BMP280::BMP280(std::string i2c_dev_name, uint8_t ccs811_addr)
        : i2c_dev_name(std::move(i2c_dev_name)),
          ccs811_addr(ccs811_addr) {
    open_device();
    init();
}

BMP280::~BMP280() {
    close_device();
}

void BMP280::close_device() {
    if (i2c_fd >= 0) close(i2c_fd);
}

void BMP280::init() {
    cout << "Resetting BMP280..." << endl;
    reset();
    this_thread::sleep_for(chrono::seconds(3));

    auto id = read_id();
    if (id != 0x58) {
        throw "Invalid device id!";
    }

    cout << "Reading calibration data" << endl;
    read_calibration_data();

    cout << "Setting the measurement control register" << endl;
    uint8_t temp_oversampling = 1; // x1 oversampling
    uint8_t pres_oversampling = 1; // x1 oversampling
    uint8_t power_mode = 3; // Normal mode

    auto ctrl_meas_reg = static_cast<uint8_t>(((temp_oversampling & 7) << 5) | ((pres_oversampling & 7) << 2) |
                                              (power_mode & 3));
    uint8_t cmd[] = {0xf4, ctrl_meas_reg};
    write_data(cmd, 2);

    cout << "Setting the configuration register" << endl;
    uint8_t t_standby = 1 << 2; // 500ms
    uint8_t iir_filter = 0; // disabled
    uint8_t spi_interface = 0; // 3-wire SPI interface is disabled

    auto config_reg = static_cast<uint8_t>(((t_standby & 7) << 5) | ((iir_filter & 7) << 2) | (spi_interface & 1));
    cmd[0] = 0xf5;
    cmd[1] = config_reg;
    write_data(cmd, 2);
}

void BMP280::open_device() {
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

std::unique_ptr<std::vector<uint8_t>> BMP280::read_registers(uint8_t start, size_t count) {
    uint8_t data[] = {start};
    write_data(data, 1);

    auto *read_buffer = new uint8_t[count];
    auto bytes_read = read(i2c_fd, read_buffer, count);

    if (bytes_read < 0) {
        // return an empty vector if we can't read anything.
        return std::make_unique<std::vector<uint8_t>>();
    }

    auto result = std::make_unique<std::vector<uint8_t>>(read_buffer, read_buffer + bytes_read);

    delete[] read_buffer;

#ifdef DBG
    std::cerr << "\tRegisters: ";
    for (auto e : *result) {
        std::cerr << std::hex << (int) e << " ";
    }
    std::cerr << std::endl;
#endif

    return result;
}


void BMP280::write_data(uint8_t *buffer, size_t buffer_len) {
#ifdef DBG
    cerr << "\tWrite: ";
    for (size_t i = 0; i < buffer_len; i++) {
        cerr << "0x" << hex << (int) buffer[i] << " ";
    }
#endif

    auto write_c = write(i2c_fd, buffer, buffer_len);
    if (write_c < 0) {
        cerr << "Unable to send command." << endl;
        // TODO - Have better exceptions.
        throw 1;
    }
#ifdef DBG
    cerr << "  ... wrote " << write_c << " bytes." << endl;
#endif
}

void BMP280::read_calibration_data() {
    auto reg_data = read_registers(0x88, 24);
    dig_T1 = (reg_data->at(1) << 8) | reg_data->at(0);
    dig_T2 = (reg_data->at(3) << 8) | reg_data->at(2);
    dig_T3 = (reg_data->at(5) << 8) | reg_data->at(4);
    dig_P1 = (reg_data->at(7) << 8) | reg_data->at(6);
    dig_P2 = (reg_data->at(9) << 8) | reg_data->at(8);
    dig_P3 = (reg_data->at(11) << 8) | reg_data->at(10);
    dig_P4 = (reg_data->at(13) << 8) | reg_data->at(12);
    dig_P5 = (reg_data->at(15) << 8) | reg_data->at(14);
    dig_P6 = (reg_data->at(17) << 8) | reg_data->at(16);
    dig_P7 = (reg_data->at(19) << 8) | reg_data->at(18);
    dig_P8 = (reg_data->at(21) << 8) | reg_data->at(20);
    dig_P9 = (reg_data->at(23) << 8) | reg_data->at(22);

    std::cout << dec << dig_T1 << endl;
    std::cout << dec << dig_T2 << endl;
    std::cout << dec << dig_T3 << endl;
    std::cout << dec << dig_P1 << endl;
    std::cout << "P2: " << dec << dig_P2 << endl;
    std::cout << dec << dig_P3 << endl;
    std::cout << dec << dig_P4 << endl;
    std::cout << dec << dig_P5 << endl;
    std::cout << dec << dig_P6 << endl;
    std::cout << dec << dig_P7 << endl;
    std::cout << dec << dig_P8 << endl;
    std::cout << dec << dig_P9 << endl;
}

void BMP280::reset() {
    uint8_t cmd[] = {0xe0, 0xb6};
    write_data(cmd, 2);
}

void BMP280::set_ctrl_meas(uint8_t val) {
    uint8_t cmd[] = {0xf4, val};
    write_data(cmd, 2);
}

uint8_t BMP280::read_id() {
    auto id = read_registers(0xd0, 1);
    return id->front();
}


void BMP280::measure() {
    auto pressure_data = read_registers(0xf7, 3);
    uint8_t pressure_msb = pressure_data->at(0);
    uint8_t pressure_lsb = pressure_data->at(1);
    uint8_t pressure_xlsb = pressure_data->at(2);

    uint32_t pressure_val = (pressure_msb << 12) | (pressure_lsb << 4) | (pressure_xlsb >> 4);

    auto temp_data = read_registers(0xfa, 3);
    uint8_t temp_msb = temp_data->at(0);
    uint8_t temp_lsb = temp_data->at(1);
    uint8_t temp_xlsb = temp_data->at(2);

    uint32_t temp_val = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

    cout << "Measured: P: " << dec << pressure_val << " T: " << dec << temp_val << endl;
    pressure = compensate_pressure(pressure_val);
    temperature = compensate_temp(temp_val);
    cout << "Compensated: P: " << dec << pressure << " T: " << dec << temperature << endl;

    pressure = pressure_val;
    temperature = temp_val;
    last_measurement = time(nullptr);
}

// Compensation formulae are taken from the datasheet.
// Returns temperature in Celsuis, resolution is 0.01 DegC.
double BMP280::compensate_temp(uint32_t adc_t) {
    double var1 = (((double) adc_t) / 16384.0 - ((double) dig_T1) / 1024.0) * ((double) dig_T2);
    double var2 = ((((double) adc_t) / 131072.0 - ((double) dig_T1) / 8192.0) *
                   (((double) adc_t) / 131072.0 - ((double) dig_T1) / 8192.0)) * ((double) dig_T3);
    t_fine = static_cast<int32_t>(var1 + var2);
    return t_fine / 5120.0;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of 24674867 represents 24674867/256 = 96386.2 Pa = 963.862 hPa
double BMP280::compensate_pressure(int32_t adc_p) {
    double var1 = ((double)t_fine / 2.0) - 64000.0;
    double var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
    var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
    double p = 1048576.0 - (double)adc_p;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double) dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double) dig_P8) / 32768.0;
    return (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;
}

double BMP280::get_temperature() {
    return temperature;
}

double BMP280::get_pressure() {
    return pressure;
}
