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
        const auto data = serial.readBuffer();

        for (const auto byte : data)
            printf("%.2x ", byte);

        if (!data.empty())
            printf("\n");


        std::this_thread::sleep_for(10ms);
    }

    return EXIT_SUCCESS;
}
