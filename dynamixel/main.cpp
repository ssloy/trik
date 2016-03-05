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
    srand(time(NULL));

    Dynamixel dxl(34); // 57600 baud

    srand(time(NULL));
    if (!dxl.open_serial("/dev/ttyS0")) {
        std::cerr << "Can not open serial device" << std::endl;
        return -1;
    }

    while (1) {
        int pos = rand()%200-100;
        std::cerr << dxl.set_goal_position(1,512+leg) << std::endl;
        usleep(1000*500);
    }

    dxl.close_serial();
    return -1;
}

