#ifndef IAQ_SI7021_H
#define IAQ_SI7021_H

#include <memory>
#include <string>
#include <vector>

// Si7021 interface per specifications in https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf
class SI7021 {
public:
    SI7021(std::string i2c_dev_name, uint8_t ccs811_addr);

    ~SI7021();

    enum Commands : uint8_t {
        MEAS_REL_HUM_HOLD = 0xe5,
        MEAS_REL_HUM = 0xf5,
        MEAS_TEMP_HOLD = 0xe3,
        MEAS_TEMP = 0xf3,
        READ_TEMP_FROM_PREV_RH_MEAS = 0xe0,
        RESET = 0xFE,
        WRITE_RHT_REG_1 = 0xe6,
        READ_RHT_REG_1 = 0xe7,
        WRITE_HEATER_CONTROL_REG = 0x51,
        READ_HEATER_CONTROL_REG = 0x11
    };

    uint8_t get_fw_rev();
    float measure_humidity();
    float measure_temperature();
    uint64_t get_serial();
private:
    const std::string i2c_dev_name;
    const uint8_t ccs811_addr;
    int i2c_fd = -1;
    uint64_t serial_no = 0;
    uint8_t fw_rev = 0;

    uint8_t crc(uint8_t in);
    void close_device();
    void init();
    void open_device();
    std::unique_ptr<std::vector<uint8_t>> read_data(size_t buffer_size);
    void read_fw_rev();
    void read_serial();
    void reset();
    void write_data(uint8_t *buffer, size_t buffer_len);
};

#endif //IAQ_SI7021_H
