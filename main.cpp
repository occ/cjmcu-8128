#include "BMP280.h"
#include "CCS811.h"
#include "SI7021.h"

#include <iomanip>

int main() {
    CCS811 ccs811("/dev/i2c-1", 0x5b);
    SI7021 si7021("/dev/i2c-1", 0x40);
    BMP280 bmp280("/dev/i2c-1", 0x76);

    while (true) {
        ccs811.read_sensors();
        bmp280.measure();

        float t_si7021 = si7021.measure_temperature();
        double t_bmp20 = bmp280.get_temperature();
        float relative_humidity = si7021.measure_humidity();

        std::cout << "T(Si7021): " << std::fixed << std::setprecision(2) << t_si7021 << "°C";
        std::cout << "\tT(BMP280): " << std::fixed << std::setprecision(2) << t_bmp20 << "°C";
        std::cout << "\tRH: " << std::fixed << std::setprecision(2) << relative_humidity << "%";
        std::cout << "\tCO2: " << std::dec << ccs811.get_co2() << "ppm";
        std::cout << "\tTVOC: " << std::dec << ccs811.get_tvoc() << "ppm";
        std::cout << "\tPres: " << std::fixed << std::setprecision(2) << bmp280.get_pressure() << "hPa";
        std::cout << std::endl;

        ccs811.set_env_data(relative_humidity, (t_si7021 + t_bmp20) / 2);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
