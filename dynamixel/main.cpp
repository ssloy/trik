#include <iostream>
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <cstdlib>
#include <ctime>

#include "dynamixel.h"


int leg_up[] = {402, 659, 291, 517, 663, 340, 621, 613, 333, 493, 590, 477};


int main() {
    srand(time(NULL));

    int home_pos[] = {412, 700, 220};
    Dynamixel dxl;

    srand(time(NULL));
    if (!dxl.open_serial("/dev/ttyACM0")) {
        std::cerr << "Can not open serial device" << std::endl;
        return -1;
    }

    int leg = rand()%4;
    for (unsigned char i=0; i<4; i++) {
        for (unsigned char j=0; j<3; j++) {
            unsigned char id = i*10 + j;
//            std::cerr << "setting goal position: " << dxl.set_goal_position(id, leg_up[((i+leg)%4)*3+j]) << std::endl;
            std::cerr << "setting goal position: " << dxl.set_goal_position(id, home_pos[j]) << std::endl;
        }
    }

    /*
    for (unsigned char i=0; i<4; i++) {
        for (unsigned char j=0; j<3; j++) {
            unsigned char id = i*10 + j;
            int pos = -1;
            std::cerr << "reading actual position: " << dxl.get_present_position(id, pos) << " ";
            std::cerr << pos << std::endl;
        }
     }
     */


    dxl.close_serial();
    return -1;
}

