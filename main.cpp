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

        std::cout << "T(Si7021): " << std::fixed << std::setprecision(2) << si7021.measure_temperature() << "°C";
        std::cout << "\tT(BMP280): " << std::fixed << std::setprecision(2) << bmp280.get_temperature() << "°C";
        std::cout << "\tRH: " << std::fixed << std::setprecision(2) << si7021.measure_humidity() << "%";
        std::cout << "\tCO2: " << std::dec << ccs811.get_co2() << "ppm";
        std::cout << "\tOC: " << std::dec << ccs811.get_tvoc() << "ppm";
        std::cout << "\tPres: " << std::fixed << std::setprecision(2) << bmp280.get_pressure() << "hPa";
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
