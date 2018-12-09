#include <iostream>
#include <iomanip>

#include "CCS811.h"
#include "SI7021.h"

int main() {
    CCS811 s("/dev/i2c-1", 0x5b);
    SI7021 s2("/dev/i2c-1", 0x40);

    std::cout << "Serial number: 0x" << hex << s2.get_serial() << std::endl;
    std::cout << "Firmware Revision: 0x" << hex << (int) s2.get_fw_rev() << std::endl;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (true) {

        s.read_sensors();
        auto co2 = s.get_co2();
        auto tvoc = s.get_tvoc();
        auto hum = s2.measure_humidity();
        auto temp = s2.measure_temperature();

        std::cout << "T: " << std::fixed << std::setprecision(3) << temp;
        std::cout << "\tH: " << std::fixed << std::setprecision(3) << hum;
        std::cout << "\tC: " << dec << co2;
        std::cout << "\tP: " << dec << s.get_tvoc() << endl;

        this_thread::sleep_for(chrono::seconds(1));
    }
#pragma clang diagnostic pop
}
