#include <iostream>
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdlib>
#include <ctime>


#define MAXNUM_TXPARAM  (150)
#define MAXNUM_RXPARAM  (60)

inline unsigned char high_byte(int v) { return (v & 0xFF00) >> 8; }
inline unsigned char  low_byte(int v) { return  v & 0xFF; }

class Dynamixel {
public:
    Dynamixel() : serial_fd_(-1) { }

    bool open_serial(const char *serial_device) {
        termios tty_opt;
        memset(&tty_opt, 0, sizeof(tty_opt));

        if ((serial_fd_ = open(serial_device, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
            std::cerr << "Serial port open error: " << serial_device << std::endl;
            return false;
        }

        tty_opt.c_cflag       = B1000000|CS8|CLOCAL|CREAD;
        tty_opt.c_iflag       = IGNPAR;
        tty_opt.c_oflag       = 0;
        tty_opt.c_lflag       = 0;
        tty_opt.c_cc[VTIME]   = 0;
        tty_opt.c_cc[VMIN]    = 0;

        tcflush(serial_fd_, TCIFLUSH);
        tcsetattr(serial_fd_, TCSANOW, &tty_opt);

        return true;
    }

    void close_serial() {
        if (serial_fd_ != -1) {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

    bool send_instruction(unsigned char id, unsigned char instruction, unsigned char *parameters, unsigned char nparams) {
        instruction_packet_[0] = instruction_packet_[1] = 0xFF;
        instruction_packet_[2] = id;
        instruction_packet_[3] = nparams + 2;
        instruction_packet_[4] = instruction;
        memcpy(instruction_packet_+5, parameters, nparams);

        unsigned char checksum = 0;
        for (unsigned char i=0; i<nparams+3; i++)
            checksum += instruction_packet_[i+2];
        instruction_packet_[nparams+5] = ~checksum;

        int bytes2write   = nparams + 6;
        int byteswritten  = write(serial_fd_, instruction_packet_, bytes2write);

        return bytes2write == byteswritten;
    }

    void goal_position(unsigned char id, int value) {
        unsigned char parameters[3] = {0x1E, low_byte(value), high_byte(value)};
        send_instruction(id, 0x03, parameters, 3);
    }

private:
    int serial_fd_;
    unsigned char instruction_packet_[MAXNUM_TXPARAM+10];
};

int main() {
    Dynamixel dxl;

    if (!dxl.open_serial("/dev/ttyACM0")) {
        std::cerr << "Can not open serial device" << std::endl;
        return -1;
    }

    srand(time(NULL));

    dxl.goal_position(3, rand()%1024);
    usleep(1000*1000);

    dxl.goal_position(1, rand()%1024);
    usleep(1000*1000);

    dxl.close_serial();
    return -1;
}

