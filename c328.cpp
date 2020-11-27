#include "c328.h"
#include "c328_cmd.h"

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

C328::C328(const string& serport)
    : _serial_port(-1), _is_sync(false)
{
    FD_ZERO(&_read_fds);
    FD_ZERO(&_write_fds);
    FD_ZERO(&_except_fds);
    
    _serial_port = open(serport.c_str(), O_RDWR);
    if (_serial_port < 0)
    {
        cerr << "Error " << errno << " from serial port open: " << strerror(errno) << endl;
        return;
    }
    
    struct termios tty;
    if (tcgetattr(_serial_port, &tty) != 0)
    {
        cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        close(_serial_port);
        _serial_port = -1;
        return;
    }

    // set 8,N,1
    tty.c_cflag &= ~PARENB; // clear parity
    tty.c_cflag &= ~CSTOPB;  // clear stop field, only one stop bit
    tty.c_cflag |= CS8;      // 8 bits per byte

    // no RTS/CTS
    tty.c_cflag &= ~CRTSCTS;

    // no carrier detect and read
    tty.c_cflag |= CREAD | CLOCAL;

    // disable canonical, echo, signal char
    tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ISIG);

    // disable software flow control, special byte handling
    tty.c_iflag &= ~(IXON|IXOFF|IXANY|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

    // disable output modes
    tty.c_oflag &= ~(OPOST|ONLCR);

    // wait for up to 100msec for data during a read
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN] = 0;

    // baud rate
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // save tty settings
    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0)
    {
        cerr << "Error " << errno << " from tcsettattr: " << strerror(errno) << endl;
        close(_serial_port);
        _serial_port = -1;
        return;
    }

    // add to select set
    FD_SET(_serial_port, &_read_fds);
    
    // finished
    cerr << "Opened serial port '" << serport.c_str() << "' set to 115200 baud" << endl;
}

C328::~C328()
{
    if (_serial_port >= 0)
        close(_serial_port);
}

void C328::close_port()
{
    if (FD_ISSET(_serial_port, &_read_fds))
        FD_CLR(_serial_port, &_read_fds);
    close(_serial_port);
    _serial_port = -1;
    _is_sync = false;
}

bool C328::write_pkt(const C328CommandPacket& pkt)
{
    ssize_t n = write(_serial_port, pkt.packet(), 6);
    if (n < 0)
    {
        cerr << "Error " << errno << " during serial port write: " << strerror(errno) << endl;
        close_port();
        return false;
    }
    return true;
}

std::pair<C328CommandPacket,bool> C328::read_pkt()
{
    uint8_t buf[6];
    ssize_t count = 0;
    while (true)
    {
        ssize_t n = read(_serial_port, &buf[count], 6 - count);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            cerr << "Error " << errno << " during serial port write: " << strerror(errno) << endl;
            close_port();
            return make_pair(C328CommandPacket(), false);
        }
        else if (n == 0)
            continue;
        count += n;
        if (count == 6)
            break;
    }
    return make_pair(C328CommandPacket(buf), true);
}

bool C328::has_recvd_bytes() const
{
    // set timeout to 0 seconds
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return (select(_serial_port + 1, const_cast<fd_set*>(&_read_fds),
                   const_cast<fd_set*>(&_write_fds), const_cast<fd_set*>(&_except_fds),
                   &timeout) == 1);
}

void C328::sync()
{
    if (_serial_port < 0)
        return;
    
    uint8_t syncd[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};
    C328CommandPacket sync(syncd);
    for (int i=0; i<200; i++)
    {
        if (!write_pkt(sync))
            return;
        if (has_recvd_bytes())
            break;
        if (usleep(10) != 0) {
            cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
            close_port();
            return;
        }
    }
    // read the reply, if there is one, could timeout
    auto ackval = read_pkt();
    if (!ackval.second || !ackval.first.is_ack())
        return;
    // now read the sync
    auto syncval = read_pkt();
    if (!syncval.second || !ackval.first.is_sync())
        return;
    _is_sync = true;
}

bool C328::is_connected() const
{
    return _serial_port >= 0 && _is_sync;
}

