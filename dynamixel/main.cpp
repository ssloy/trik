#include <iostream>
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <cstdlib>
#include <ctime>

#include "dynamixel.h"

int main() {
    Dynamixel dxl;

    if (!dxl.open_serial("/dev/ttyACM0")) {
        std::cerr << "Can not open serial device" << std::endl;
        return -1;
    }

    for (unsigned char i=0; i<254; i++) {
        if (dxl.ping(i)) {
            std::cerr << "id " << (int)i << " is found on the bus, setting the zero position" << std::endl;
            std::cerr << "setting goal position: " << dxl.set_goal_position(i, 512) << std::endl;
            bool moving = false;
            while (dxl.is_moving(i, moving) && moving) {
                usleep(1000);
            }
        }
    }

    dxl.close_serial();
    return -1;
}

