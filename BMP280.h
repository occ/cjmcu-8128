#ifndef IAQ_BMP280_H
#define IAQ_BMP280_H

#include <memory>
#include <string>
#include <vector>

// BMP280 interface per specifications in
// https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001.pdf
class BMP280 {
public:
    BMP280(std::string i2c_dev_name, uint8_t ccs811_addr);

    ~BMP280();

    double get_pressure();

    double get_temperature();

    void measure();

private:
    const std::string i2c_dev_name;
    const uint8_t ccs811_addr;
    int i2c_fd = -1;
    time_t last_measurement = 0;
    double pressure;
    double temperature;

    // Calibration values.
    uint16_t dig_T1, dig_P1;
    int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    // State for the compensation formula
    int32_t t_fine;

    double compensate_temp(uint32_t temp_val);

    double compensate_pressure(int32_t adc_P);

    void close_device();

    void init();

    void open_device();

    void read_calibration_data();

    std::unique_ptr<std::vector<uint8_t>> read_registers(uint8_t start, size_t count);

    uint8_t read_id();

    void reset();

    void set_ctrl_meas(uint8_t val);

    void write_data(uint8_t *buffer, size_t buffer_len);
};

#endif //IAQ_BMP280_H
