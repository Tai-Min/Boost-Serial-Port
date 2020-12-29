#include <cstdlib>
#include <iostream>
#include <cstdio>
#include <thread>
#include <chrono>

#include "BoostSerial.h"

int main(int argc, char** argv)
{
    using namespace std::chrono_literals;

    BoostSerial serial;
    serial.open("/dev/ttyACM0", 9600 );

    while (true)
    {
        const auto data = serial.readString();
        std::cout << data;

        std::this_thread::sleep_for(10ms);
    }

    return EXIT_SUCCESS;
}
