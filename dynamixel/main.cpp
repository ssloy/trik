#include <iostream>
#include <cstring>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <errno.h>
#include <cstdlib>
#include <ctime>


inline unsigned char high_byte(int v) { return (v & 0xFF00) >> 8; }
inline unsigned char  low_byte(int v) { return  v & 0xFF; }

class Dynamixel {
public:
    static const unsigned char broadcast_id = 254;
    enum CommStatus {
        COMM_TXSUCCESS = 0,
        COMM_RXSUCCESS = 1,
        COMM_TXFAIL    = 2,
        COMM_RXFAIL    = 3,
        COMM_TXERROR   = 4,
        COMM_RXWAITING = 5,
        COMM_RXTIMEOUT = 6,
        COMM_RXCORRUPT = 7
    };

    Dynamixel() : serial_fd_(-1), baud_number_(1), baud_rate_(), byte_transfer_time_ms_() {
        baud_rate_ = 2000000.0f/(float)(baud_number_+1);
        byte_transfer_time_ms_ = (float)((1000.0f / baud_rate_) * 10.0f);
    }

    bool open_serial(const char *serial_device) {
        termios tty_opt;
        memset(&tty_opt, 0, sizeof(tty_opt));

        if ((serial_fd_ = open(serial_device, O_RDWR|O_NOCTTY|O_NONBLOCK)) < 0) {
            return false;
        }

        tty_opt.c_cflag       = B1000000|CS8|CLOCAL|CREAD; // TODO: baudrate depending on the input baud_number_ and not just default 1Mbps
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

    unsigned char status_error_byte() {
        return status_packet_[4];
    }

    bool set_goal_position(unsigned char id, int value) {
        unsigned char parameters[3] = {0x1E, low_byte(value), high_byte(value)};
        return COMM_RXSUCCESS == send_instruction_read_status(id, 0x03, parameters, 3);
    }

    bool change_id(unsigned char old_id, unsigned char new_id) {
        unsigned char parameters[2] = {0x03, new_id};
        return COMM_RXSUCCESS == send_instruction_read_status(old_id, 0x03, parameters, 2);;
    }

    bool get_present_position(unsigned char id, int &position) {
        unsigned char parameters[2] = {0x24, 2};
        CommStatus ret = send_instruction_read_status(id, 0x02, parameters, 2);
        if (COMM_RXSUCCESS != ret) return false;

        unsigned char nparams = status_packet_[3]-2;
        unsigned char rxid = status_packet_[2];
        if (rxid!=id || nparams!=2) return false;
        position = (status_packet_[6]<<8) | status_packet_[5];
        return true;
    }

    bool ping(unsigned char id) {
        CommStatus ret = send_instruction_read_status(id, 0x01, NULL, 0);
        if (ret!=COMM_RXSUCCESS) return false;
        return id == status_packet_[2];
    }



private:
    CommStatus rx(unsigned char offset, unsigned char toread, int timeout_ms) {
        long   start_ms, cur_ms;
        time_t start_s,  cur_s;
        struct timespec spec;
        clock_gettime(CLOCK_REALTIME, &spec);
        start_s  = spec.tv_sec;
        start_ms = spec.tv_nsec/1.0e6;

        int received = 0;
        while (received < toread) {
            int n = read(serial_fd_, status_packet_+offset+received, toread-received);
            if (n>=0) {
                received += n;
            } else {
                return COMM_RXFAIL;
            }
            clock_gettime(CLOCK_REALTIME, &spec);
            cur_s  = spec.tv_sec;
            cur_ms = spec.tv_nsec/1.0e6;
            if ((cur_s-start_s)*1000+(cur_ms-start_ms) > timeout_ms) return COMM_RXTIMEOUT;
        }
        return COMM_RXSUCCESS;
    }

    CommStatus read_status_packet() {
        memset(status_packet_, 0, max_packet_length_);
        CommStatus ret = rx(0, 6, 6*byte_transfer_time_ms_ + 5); // if nparams=0 (most of the status packets), then the entire status packet has 6 bytes: one call of rx()
        if (ret!=COMM_RXSUCCESS) {
            return ret;
        }

        if (0xFF!=status_packet_[0] || 0xFF!=status_packet_[1]) {
            return COMM_RXCORRUPT;
        }

        unsigned char nparams = status_packet_[3]-2;
        if (nparams>0) {
            ret = rx(5, nparams+1, (nparams+1)*byte_transfer_time_ms_);
            if (ret!=COMM_RXSUCCESS) {
                return ret;
            }
        }

        unsigned char checksum = 0;
        for (unsigned char i=0; i<nparams+3; i++) {
            checksum += status_packet_[i+2];
        }
        checksum = ~checksum;

        if (!ret || checksum!=status_packet_[nparams+5]) {
            return COMM_RXCORRUPT;
        }

        return COMM_RXSUCCESS;
    }

    CommStatus send_instruction_packet(unsigned char id, unsigned char instruction, unsigned char *parameters, unsigned char nparams) {
        instruction_packet_[0] = instruction_packet_[1] = 0xFF;
        instruction_packet_[2] = id;
        instruction_packet_[3] = nparams + 2;
        instruction_packet_[4] = instruction;
        memcpy(instruction_packet_+5, parameters, nparams);

        unsigned char checksum = 0;
        for (unsigned char i=0; i<nparams+3; i++)
            checksum += instruction_packet_[i+2];
        instruction_packet_[nparams+5] = ~checksum;

        // TODO: halfdupex TX ON
        int packet_length   = nparams + 6;
        int nbytes_sent  = write(serial_fd_, instruction_packet_, packet_length);
        tcdrain(serial_fd_);
        // TODO: halfdupex TX OFF

        return packet_length == nbytes_sent ? COMM_TXSUCCESS : COMM_TXFAIL;
    }

    CommStatus send_instruction_read_status(unsigned char id, unsigned char instruction, unsigned char *parameters, unsigned char nparams) {
        CommStatus ret = send_instruction_packet(id, instruction, parameters, nparams);
        if (COMM_TXSUCCESS != ret) return ret;
        return read_status_packet();
    }


    static const int max_packet_length_ = 255;
    int serial_fd_;
    int baud_number_;
    float baud_rate_;
    float byte_transfer_time_ms_;
    unsigned char instruction_packet_[max_packet_length_];
    unsigned char      status_packet_[max_packet_length_];
};

int main() {
    Dynamixel dxl;

    if (!dxl.open_serial("/dev/ttyACM0")) {
        std::cerr << "Can not open serial device" << std::endl;
        return -1;
    }

    /*

    srand(time(NULL));

    dxl.goal_position(3, rand()%1024);
    usleep(1000*1000);

    dxl.goal_position(1, rand()%1024);
    usleep(1000*1000);
    */

    std::cerr << dxl.ping(0x01) << std::endl;
/*
    int position = -10000;
    std::cerr << dxl.get_present_position(2, position) << " " << position << std::endl;
    std::cerr << dxl.get_present_position(2, position) << " " << position << std::endl;
*/

    dxl.set_goal_position(0, 512);
    usleep(1000*1000);
    dxl.set_goal_position(1, 512);
    usleep(1000*1000);
    dxl.set_goal_position(2, 512);
    usleep(1000*1000);
    dxl.close_serial();
    return -1;
}

