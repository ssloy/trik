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

    srand(time(NULL));
    if (!dxl.open_serial("/dev/ttyACM0")) {
        std::cerr << "Can not open serial device" << std::endl;
        return -1;
    }

    for (unsigned char i=0; i<254; i++) {
        if (dxl.ping(i)) {
           std::cerr << "id " << (int)i << " is found on the bus, setting the zero position" << std::endl;
            std::cerr << "setting goal position: " << dxl.set_goal_position(i, 512+rand()%10) << std::endl;
            unsigned char moving = 0;
            while (Dynamixel::COMM_RXSUCCESS==dxl.is_moving(i, moving) && moving) {
                usleep(10000);
            }
            int pos = -1;
            std::cerr << "reading actual position: " << dxl.get_present_position(i, pos) << " ";
            std::cerr << pos << std::endl;
        }
    }

    dxl.close_serial();
    return -1;
}

