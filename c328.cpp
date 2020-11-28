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
    // set up raw mode / no echo / binary
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE |
                     ECHOK | ECHONL | ISIG | IEXTEN |
                     ECHOCTL | ECHOKE);
    tty.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | IUCLC | PARMRK);

    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(CSTOPB);

    tty.c_iflag &= ~(INPCK | ISTRIP);
    tty.c_cflag &= ~(PARENB | PARODD | CMSPAR);

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag &= ~(CRTSCTS);
    
    // read 1 char blocking
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

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
    close(_serial_port);
    _serial_port = -1;
    _is_sync = false;
}

bool C328::write_pkt(const C328CommandPacket& pkt, bool debug)
{
    ssize_t n = write(_serial_port, pkt.packet(), 6);
    if (n < 0)
    {
        cerr << "Error " << errno << " during serial port write: " << strerror(errno) << endl;
        close_port();
        return false;
    }
    else if (n != 6)
    {
        cerr << "Unable to write packet to port\n";
        close_port();
        return false;
    }
    if (debug)
        pkt.debug(true);
    return true;
}

bool C328::read_bytes(uint8_t* buf, ssize_t nbytes)
{
    ssize_t count = 0;
    while (count != nbytes)
    {
        ssize_t n = read(_serial_port, buf + count, nbytes - count);
        if (n < 0)
        {
            if (errno == EINTR)
                continue;
            cerr << "Error " << errno << " during serial port read: " << strerror(errno) << endl;
            close_port();
            return false;
        }
        count += n;
    }
    return true;
}

std::pair<C328CommandPacket,bool> C328::read_pkt(bool debug)
{
    uint8_t buf[6];
    bool ret = read_bytes(buf, 6);
    C328CommandPacket pkt(buf);
    if (debug)
        pkt.debug(false);
    return make_pair(pkt, ret);
}

bool C328::has_recvd_bytes() const
{
    if (_serial_port < 0)
        return false;

    fd_set readfds;
    FD_ZERO(&readfds);
    // add to select set
    FD_SET(_serial_port, &readfds);
    
    // set timeout to 0 seconds
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return (select(_serial_port + 1, &readfds, NULL, NULL, &timeout) == 1);
}

void C328::reset(bool state_only)
{
    if (_serial_port < 0)
        return;

    uint8_t resetd[6] = {0xAA, 0x08, 0x00, 0x00, 0x00, 0xFF};
    if (state_only)
        resetd[2] = 0x01;
    
    C328CommandPacket resetpkt(resetd);
    if (!write_pkt(resetpkt))
        return;

    if (usleep(2000000) != 0)
    {
        cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
        close_port();
        return;
    }
}

void C328::sync()
{
    if (_serial_port < 0)
        return;

    uint8_t syncd[6] = {0xAA, 0x0D, 0x00, 0x00, 0x00, 0x00};
    C328CommandPacket sync(syncd);
    for (int i=0; i<500; i++)
    {
        if (!write_pkt(sync))
            return;
        if (has_recvd_bytes())
            break;
        if (usleep(10000) != 0) {
            cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
            close_port();
            return;
        }
    }
    // read the reply, if there is one, could timeout
    auto ackval = read_pkt(true);
    if (!ackval.second || !ackval.first.is_ack())
    {
        cerr << "Camera did not send ack packet" << endl;
        return;
    }
    // now read the sync
    auto syncval = read_pkt(true);
    // sometimes an ack instead of a sync is sent
    if (!syncval.second || (!syncval.first.is_sync() && !syncval.first.is_ack()))
    {
        cerr << "Camera did not send sync packet" << endl;
        return;
    }
    _is_sync = true;
}

void C328::setup()
{
    if (_serial_port < 0)
        return;

    uint8_t setd[6] = {0xAA, 0x01, 0x00, 0x01, 0x03, 0x05};
    C328CommandPacket set(setd);
    if (!write_pkt(set, true))
        return;

    // read the reply
    auto ackval = read_pkt(true);
    if (!ackval.second || !ackval.first.is_ack())
    {
        cerr << "Camera did not send ack setup command" << endl;
        return;
    }
}

void C328::set_pkg_size()
{
    if (_serial_port < 0)
        return;

    // data package size = 512b
    uint8_t setpkg[6] = {0xAA, 0x06, 0x08, 0x00, 0x02, 0x00};
    C328CommandPacket setpkgsize(setpkg);
    if (!write_pkt(setpkgsize, true))
        return;

    // read the reply
    auto ackval = read_pkt(true);
    if (!ackval.second || !ackval.first.is_ack())
    {
        cerr << "Camera did not ack set package size command" << endl;
        return;
    }
}

void C328::snapshot()
{
    if (_serial_port < 0)
        return;

    uint8_t snapd[6] = {0xAA, 0x05, 0x00, 0x00, 0x00, 0x00};
    C328CommandPacket snap(snapd);
    // loop until camera succesfully takes snapshot
    while (true)
    {
        if (!write_pkt(snap, true))
            return;

        // read the reply
        auto ackval = read_pkt(true);
        auto retpkt = ackval.first;
        if (!ackval.second)
        {
            cerr << "Camera failed to respond to snapshot command" << endl;
                return;
        }
        if (retpkt.is_nak())
        {
            struct NakFields nf;
            retpkt.nak_fields(&nf);
            if (nf.err == 0x0f)
                continue;
            cerr << "Camera responded to snapshot with error: " << nf.err << endl;
        }
        if (retpkt.is_ack())
            break;
        if(usleep(10000) != 0)
        {
            cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
            close_port();
            return;
        }
    }
}

void C328::get_picture(uint8_t* pixtype, uint32_t* datasz)
{
    if (_serial_port < 0)
        return;

    *pixtype = 0;
    *datasz = 0;
    
    uint8_t gpixd[6] = {0xAA, 0x04, 0x01, 0x00, 0x00, 0x00};
    C328CommandPacket gpix(gpixd);
    while (true)
    {
        // loop until camera is ready to return picture
        while (true)
        {
            if (!write_pkt(gpix, true))
                return;

            // read the reply
            auto ackval = read_pkt(true);
            auto retpkt = ackval.first;
            if (!ackval.second)
            {
                cerr << "Camera failed to respond to get picture command" << endl;
                return;
            }
            if (retpkt.is_nak())
            {
                struct NakFields nf;
                retpkt.nak_fields(&nf);
                if (nf.err == 0x0f)
                    continue;
                cerr << "Camera responded to get picture with error: " << nf.err << endl;
            }
            if (retpkt.is_ack())
                break;
            if(usleep(10000) != 0)
            {
                cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
                close_port();
                return;
            }
        }
        // now get data
        auto datval = read_pkt(true);
        if (!datval.second)
        {
            cerr << "Camera failed to send data packet" << endl;
            return;
        }
        if (datval.first.is_data())
        {
            struct DataFields nf;
            datval.first.data_fields(&nf);
            *datasz = nf.datasz;
            *pixtype = nf.pixtype;
            break;
        }
        if (datval.first.is_nak())
        {
            struct NakFields nf;
            datval.first.nak_fields(&nf);
            cerr << "Camera sent error " << nf.err << " instead of data during snapshot" << endl;
            return;
        }
        if(usleep(10000) != 0)
        {
            cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
            close_port();
            return;
        }
    }
}

void C328::get_data_packages(uint32_t datasz, uint8_t* pixbuf)
{
    if (_serial_port < 0)
        return;

    int count = 0;
    uint32_t bytecount = 0;
    uint8_t buf[512];
    while (true)
    {
        uint8_t pkgackd[6] = {0xAA, 0x0E, 0x00, 0x00, 0x00, 0x00};
        pkgackd[4] = (count & 0xFF);
        pkgackd[5] = (count & 0xFF00) >> 8;
        C328CommandPacket pkgack(pkgackd);
        if (!write_pkt(pkgack, true))
            return;
        // read header
        if (!read_bytes(buf, 4))
        {
            cerr << "Error while reading data bytes" << endl;
            return;
        }
        uint16_t pkgid = static_cast<uint16_t>((buf[1] << 8) | buf[0]);
        uint16_t pkgsz = static_cast<uint16_t>((buf[3] << 8) | buf[2]);
        //cerr << "Package " << dec << pkgid << " with " << pkgsz << " bytes" << endl;
        if (!read_bytes(buf + 4, pkgsz + 2))
        {
            cerr << "Error while reading data bytes" << endl;
            return;
        }
        uint16_t chksum = static_cast<uint16_t>((buf[pkgsz+5] << 8) | buf[pkgsz+4]);
        //cerr << "Package checksum: " << hex << chksum;
        int sum = 0;
        for (int i=0; i<pkgsz+4; i++)
            sum += buf[i];
        uint16_t calc_chksum = sum & 0xFF;
        //cerr << " calculated: " << hex << calc_chksum << endl;
        if (chksum != calc_chksum)
        {
            cerr << "Error with package " << pkgid << " checksum" << endl;
            return;
        }
        memcpy(pixbuf + bytecount, &buf[4], pkgsz);
        bytecount += pkgsz;
        if (bytecount >= datasz)
            break;
        count++;
    }
    // send final ack
    uint8_t ackd[6] = {0xAA, 0x0E, 0x0A, 0x00, 0x00, 0x00};
    C328CommandPacket ack(ackd);
    write_pkt(ack, true);
}

bool C328::is_connected() const
{
    return _serial_port >= 0 && _is_sync;
}

