#include "BMP280.h"
#include "CCS811.h"
#include "SI7021.h"

#include <iomanip>

int main() {
    CCS811 s1("/dev/i2c-1", 0x5b);
    SI7021 s2("/dev/i2c-1", 0x40);
    BMP280 s3("/dev/i2c-1", 0x76);

    for (int i = 0; i < 3; i++) {
        s1.read_sensors();
        auto co2 = s1.get_co2();
        auto tvoc = s1.get_tvoc();

        auto hum = s2.measure_humidity();
        auto temp = s2.measure_temperature();

        std::cout << "T: " << std::fixed << std::setprecision(3) << temp;
        std::cout << "\tH: " << std::fixed << std::setprecision(3) << hum;
        std::cout << "\tC: " << dec << co2;
        std::cout << "\tO: " << dec << tvoc << endl;

        s3.measure();
        auto t2 = s3.get_temperature();
        auto pres = s3.get_temperature();

        std::cout << "T2: " << dec << t2 << "\tP: " << pres << std::endl;

        this_thread::sleep_for(chrono::seconds(2));
    }
}
