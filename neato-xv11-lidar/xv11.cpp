#include <iostream>
#include <cmath>
#include <fstream>
#include <cstring>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

const char *serialport = "/dev/ttyS0";

int nframe = 0;

float rpm(unsigned char *packet) { // 22 bytes in the packet
    return float(packet[2] | ((packet[3]<<8))) / 64.;
}

bool verify_packet_checksum(unsigned char *packet) { // 22 bytes in the packet
    int checksum32 = 0;
    for (int i=0; i<10; i++) {
        checksum32 = (checksum32<<1) + packet[2*i] + (packet[2*i+1]<<8);
    }
    return packet[20]+(packet[21]<<8) == (((checksum32 & 0x7FFF) + (checksum32 >> 15)) & 0x7FFF);
}

int count_errors(unsigned char *buf) { // 1980 bytes in the buffer (90 packets)
    int nb_err = 0;
    for (int i=0; i<90; i++) {
        if (!verify_packet_checksum(buf+i*22)) {
            nb_err++;
        }
    }
    return nb_err;
}




int maxs = -1;
void drop_point(std::ofstream &ofs, unsigned char *data, int angle_degrees) { // 4 bytes in the data buffer
    int flag1 = (data[1] & 0x80) >> 7;  // No return/max range/too low of reflectivity
    //  int flag2 = (data[1] & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
    if (flag1) { return; }

    float angle = angle_degrees*M_PI/180.;

    int dist_mm  = data[0] | (( data[1] & 0x3F) << 8);    // 14 bits for the distance
    int strength = data[2] | (data[3] << 8);              // 16 bits for the signal strength
    maxs = strength;
    return;
//    std::cerr << "s: " << strength << std::endl;
    float x = dist_mm*cos(angle);
    float y = dist_mm*sin(angle);
    ofs << x << " " << y << " " << nframe << std::endl;

}

void drop_distances(std::ofstream &ofs, unsigned char *buf) {
    int angle_degrees = 0;
    int MAXS = -1;
    for (int p=0; p<90; p++) {
//        std::cerr << "#rpm: " << rpm(buf + p*22) << std::endl;
        ofs /*<< "#rpm: " << rpm(buf + p*22)*/ << std::endl;
        for (int i=0; i<4; i++) {
            drop_point(ofs, buf + p*22 + 4 + i*4, angle_degrees++);
            MAXS = std::max(MAXS, maxs);
        }
    }
    std::cerr << "max strength: " << MAXS << std::endl;
    nframe++;
}

int main(int argc, char *argv[]) {
    if (2>argc) {
        std::cerr << "Usage: " << argv[0] << " points.obj" << std::endl;
        return -1;
    }

    struct termios tty_opt;
    int tty_fd;

    std::cerr << "Opening serial port " << serialport << std::endl;
    tty_fd = open(serialport, O_RDWR);
    if (tty_fd < 0) {
        std::cerr << "Could not open port " << serialport << std::endl;
        return -1;
    }

    memset(&tty_opt, 0, sizeof(tty_opt));

    tty_opt.c_cflag = CS8 | CLOCAL | CREAD; // 8N1
    tty_opt.c_iflag = 0;
    tty_opt.c_oflag = 0;
    tty_opt.c_lflag = 0;     // noncanonical mode
    tty_opt.c_cc[VMIN] = 1;  // one char is enough
    tty_opt.c_cc[VTIME] = 0; // no timer

    cfsetospeed(&tty_opt, B115200); // 115200 baud
    cfsetispeed(&tty_opt, B115200); // 115200 baud

    tcsetattr(tty_fd, TCSANOW, &tty_opt);
    std::ofstream ofs(argv[1], /*std::fstream::app|*/std::fstream::out);

    unsigned char buf[1980];
    int idx = 0;
    while (1) {
        if (0==idx && 1==read(tty_fd, buf, 1) && 0xFA==buf[0]) {
            if (1==read(tty_fd, buf+1, 1) && 0xA0==buf[1]) {
                idx = 2;
                for (; idx<1980; idx++) {
                    if (1==read(tty_fd, buf+idx, 1)) {
                    } else {
                        break;
                    }
                }
                if (0==count_errors(buf)) {
                    drop_distances(ofs, buf);
                    std::cerr << "ok" << std::endl;
//                    break;
                }
                idx = 0;
            }
        }
//        std::cerr << "gna\n";
    }
    close(tty_fd);
    ofs.close();
    return 0;
}

