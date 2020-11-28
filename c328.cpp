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
    : _serial_port(-1), _is_sync(false), _debug(false)
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

bool C328::write_pkt(const C328CommandPacket& pkt)
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
    if (_debug)
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

std::pair<C328CommandPacket,bool> C328::read_pkt()
{
    uint8_t buf[6];
    bool ret = read_bytes(buf, 6);
    C328CommandPacket pkt(buf);
    if (_debug)
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

// implements basic pattern, where a command is sent and an ack is received
bool C328::command_response(const C328CommandPacket& cmdpkt)
{
    if (_serial_port < 0)
        return false;
    if (!write_pkt(cmdpkt))
        return false;
    // read the reply
    auto ackval = read_pkt();
    if (!ackval.second)
    {
        cerr << "Camera did not ack packet" << endl;
        return false;
    }
    else if (!ackval.first.is_ack() && ackval.first.is_nak())
    {
        struct NakFields nf;
        ackval.first.nak_fields(&nf);
        cerr << "Camera error[" << nf.errcode << "] :" << ackval.first.nak_error() << endl;
        return false;
    }
    
    return true;
}

void C328::reset(bool state_only)
{
    C328CommandPacket resetpkt(0x08, state_only ? 0x01 : 0x00, 0x00, 0x00, 0xFF);
    if (!command_response(resetpkt))
        return;
    if (usleep(2000000) != 0)
    {
        cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
        close_port();
        return;
    }
}

void C328::power_off()
{
    C328CommandPacket pwrpkt(0x09, 0x00, 0x00, 0x00, 0x00);
    if (!command_response(pwrpkt))
        return;
}

void C328::sync()
{
    if (_serial_port < 0)
        return;

    C328CommandPacket sync(0x0D, 0x00, 0x00, 0x00, 0x00);
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
    // read the reply
    auto ackval = read_pkt();
    if (!ackval.second || !ackval.first.is_ack())
    {
        cerr << "Camera did not send ack packet" << endl;
        return;
    }
    // now read the sync
    auto syncval = read_pkt();
    if (!syncval.second || !syncval.first.is_sync())
    {
        cerr << "Camera did not send sync packet" << endl;
        return;
    }
    // ack the sync
    C328CommandPacket ack(0x0E, 0x0D, ackval.first.packet()[3], 0x00, 0x00);
    if (!write_pkt(ack))
        return;
    _is_sync = true;

    // sleep for 2s to let camera stabilize
    if (usleep(2000000) != 0)
    {
        cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
        close_port();
        return;
    }
}

void C328::initial(enum ColorType ct, enum PreviewResolution pr, enum JPEGResolution jr)
{
    if (_serial_port < 0)
        return;

    C328CommandPacket ini(0x01, 0x00, static_cast<uint8_t>(ct),
                          static_cast<uint8_t>(pr),
                          static_cast<uint8_t>(jr));
    if (!write_pkt(ini))
        return;

    // read the reply
    auto ackval = read_pkt();
    if (!ackval.second || !ackval.first.is_ack())
    {
        cerr << "Camera did not ack initial command" << endl;
        return;
    }
}

// hardcoded for 512 bytes
void C328::set_pkg_size()
{
    if (_serial_port < 0)
        return;

    C328CommandPacket setpkgsize(0x06, 0x08, 0x00, 0x02, 0x00);
    if (!write_pkt(setpkgsize))
        return;

    // read the reply
    auto ackval = read_pkt();
    if (!ackval.second || !ackval.first.is_ack())
    {
        cerr << "Camera did not ack set package size command" << endl;
        return;
    }
}

void C328::snapshot(enum SnapshotType st)
{
    if (_serial_port < 0)
        return;

    C328CommandPacket snap(0x05, static_cast<uint8_t>(st), 0x00, 0x00, 0x00);
    // loop until camera succesfully takes snapshot
    while (true)
    {
        if (!write_pkt(snap))
            return;

        // read the reply
        auto ackval = read_pkt();
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
            if (nf.errcode == 0x0f)
                continue;
            cerr << "Camera snapshot error[" << nf.errcode << "] :" << retpkt.nak_error() << endl;
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

void C328::get_picture(enum PictureType pt, uint8_t* pixtype, uint32_t* datasz)
{
    if (_serial_port < 0)
        return;

    *pixtype = 0;
    *datasz = 0;
    
    C328CommandPacket gpix(0x04, static_cast<uint8_t>(pt), 0x00, 0x00, 0x00);
    while (true)
    {
        // loop until camera is ready to return picture
        while (true)
        {
            if (!write_pkt(gpix))
                return;

            // read the reply
            auto ackval = read_pkt();
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
                if (nf.errcode == 0x0f) // picture not ready error
                    continue;
                cerr << "Camera get picture error[" << nf.errcode << "] :" << retpkt.nak_error() << endl;
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
        auto datval = read_pkt();
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
            cerr << "Camera error[" << nf.errcode << "] snapshot: " << datval.first.nak_error() << endl;
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
        C328CommandPacket pkgack(0x0E, 0x00, 0x00, (count & 0xFF), (count & 0xFF00) >> 8);
        if (!write_pkt(pkgack))
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
    C328CommandPacket ack(0x0E, 0x00, 0x00, 0xF0, 0xF0);
    write_pkt(ack);
}

void C328::get_image_data(uint32_t datasz, uint8_t* pixbuf)
{
    if (_serial_port < 0)
        return;

    if (!read_bytes(pixbuf, datasz))
    {
        cerr << "Error while reading image data" << endl;
        return;
    }
    // send final ack
    C328CommandPacket ack(0x0E, 0x0A, 0x00, 0x00, 0x00);
    write_pkt(ack);
}

bool C328::is_connected() const
{
    return _serial_port >= 0 && _is_sync;
}

std::pair<uint32_t, uint8_t*> C328::jpeg_snapshot()
{
    snapshot(Compressed);

    uint8_t pixtype;
    uint32_t datasz;

    get_picture(Snapshot, &pixtype, &datasz);
    if (pixtype == 0)
    {
        std::cerr << "Error during get picture\n";
        return make_pair(0, nullptr);
    }
    //std::cerr << "Pixtype: " << static_cast<int>(pixtype) << " size: " << datasz << std::endl;

    // allocate a buffer to hold the pix
    uint8_t* pixbuf = new uint8_t[datasz];

    get_data_packages(datasz, pixbuf);
    
    return make_pair(datasz, pixbuf);
}

std::pair<uint32_t, uint8_t*> C328::uncompressed_snapshot()
{
    snapshot(Uncompressed);

    uint8_t pixtype;
    uint32_t datasz;

    get_picture(Snapshot, &pixtype, &datasz);
    if (pixtype == 0)
    {
        std::cerr << "Error during get picture\n";
        return make_pair(0, nullptr);
    }
    //std::cerr << "Pixtype: " << static_cast<int>(pixtype) << " size: " << datasz << std::endl;

    // allocate a buffer to hold the pix
    uint8_t* pixbuf = new uint8_t[datasz];

    get_image_data(datasz, pixbuf);
    
    return make_pair(datasz, pixbuf);
}

std::pair<uint32_t, uint8_t*> C328::jpeg_preview()
{
    uint8_t pixtype;
    uint32_t datasz;

    get_picture(JpegPreview, &pixtype, &datasz);
    if (pixtype == 0)
    {
        std::cerr << "Error during get picture\n";
        return make_pair(0, nullptr);
    }
    //std::cerr << "Pixtype: " << static_cast<int>(pixtype) << " size: " << datasz << std::endl;

    // allocate a buffer to hold the pix
    uint8_t* pixbuf = new uint8_t[datasz];

    get_data_packages(datasz, pixbuf);
    
    return make_pair(0, nullptr);
}

std::pair<uint32_t, uint8_t*> C328::uncompressed_preview()
{
    uint8_t pixtype;
    uint32_t datasz;

    get_picture(Preview, &pixtype, &datasz);
    if (pixtype == 0)
    {
        std::cerr << "Error during get picture\n";
        return make_pair(0, nullptr);
    }
    //std::cerr << "Pixtype: " << static_cast<int>(pixtype) << " size: " << datasz << std::endl;

    // allocate a buffer to hold the pix
    uint8_t* pixbuf = new uint8_t[datasz];

    get_image_data(datasz, pixbuf);
    
    return make_pair(datasz, pixbuf);
}
