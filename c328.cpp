#include "c328.h"
#include "c328_cmd.h"

#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

C328::C328(const string& serport, const string& baudrate)
    : _serial_port(-1), _is_sync(false), _debug(false)
{
    auto baud = B9600;
    
    // check baudrate
    if (baudrate == "19200")
        baud = B19200;
    else if (baudrate == "38400")
        baud = B38400;
    else if (baudrate == "57600")
        baud = B57600;
    else if (baudrate == "115200")
        baud = B115200;
    cerr << "Baudrate: " << baudrate << endl;
    
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
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // save tty settings
    if (tcsetattr(_serial_port, TCSANOW, &tty) != 0)
    {
        cerr << "Error " << errno << " from tcsettattr: " << strerror(errno) << endl;
        close(_serial_port);
        _serial_port = -1;
        return;
    }
    
    // finished
    cerr << "Opened serial port '" << serport.c_str() << "' at " << baudrate << " baud" << endl;
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

    int retval = select(_serial_port + 1, &readfds, NULL, NULL, &timeout);
    // cerr << "select: " << retval << endl;
    return retval == 1;
}

// implements basic pattern, where a command is sent and an ack is received
bool C328::command_response(const C328CommandPacket& cmdpkt)
{
    if (_serial_port < 0)
        return false;

    uint8_t commandid = cmdpkt.packet()[1];
    
    if (!write_pkt(cmdpkt))
        return false;
    // read the reply
    auto ackval = read_pkt();
    if (!ackval.second)
    {
        cerr << "Camera did not ack" << endl;
        return false;
    }
    else if (ackval.first.is_nak())
    {
        struct NakFields nf;
        ackval.first.nak_fields(&nf);
        cerr << "Camera error command " << static_cast<int>(commandid)
                 << " [" << static_cast<int>(nf.errcode)
             << "]: " << ackval.first.nak_error() << endl;
        return false;
    }
    else if(ackval.first.is_sync())
    {
        cerr << "Camera responded with sync for command "
             << static_cast<int>(commandid) << endl;
    }
    else if (ackval.first.packet()[2] != commandid)
    {
        cerr << "Camera responded with different commandid " << static_cast<int>(ackval.first.packet()[2])
             << " instead of " << static_cast<int>(commandid) << endl;
        //return false;
    }
    return true;
}

bool C328::reset(bool state_only)
{
    C328CommandPacket resetpkt(0x08, state_only ? 0x01 : 0x00, 0x00, 0x00, 0xFF);
    return command_response(resetpkt);
}

bool C328::power_off()
{
    C328CommandPacket pwrpkt(0x09, 0x00, 0x00, 0x00, 0x00);
    return command_response(pwrpkt);
}

void C328::sync()
{
    if (_serial_port < 0)
        return;

    C328CommandPacket sync(0x0D, 0x00, 0x00, 0x00, 0x00);
    for (int i=0; i<200; i++)
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
    if (!has_recvd_bytes())
        return;
    
    // read the reply
    auto ackval = read_pkt();
    if (!ackval.second)
    {
        cerr << "Camera did not send ack packet" << endl;
        return;
    }
    else if (!ackval.first.is_ack() && ackval.first.is_nak())
    {
        struct NakFields nf;
        ackval.first.nak_fields(&nf);
        cerr << "Camera error[" << static_cast<int>(nf.errcode)
             << "]: " << ackval.first.nak_error() << endl;
        return;
    }
    // now read the sync
    auto syncval = read_pkt();
    if (!syncval.second || !syncval.first.is_sync())
    {
        cerr << "Camera did not send sync packet" << endl;
        return;
    }
    // ack the ack
    C328CommandPacket ack(0x0E, 0x0D, 0x00, 0x00, 0x00);
    if (!write_pkt(ack))
        return;
    _is_sync = true;

    // sleep for 2s to let camera stabilize
    if (usleep(2000000) != 0)
    {
        cerr << "Error " << static_cast<int>(errno) << " during sleep: " << strerror(errno) << endl;
        close_port();
        return;
    }
}

bool C328::initial(enum ColorType ct, enum PreviewResolution pr, enum JPEGResolution jr)
{
    C328CommandPacket ini(0x01, 0x00, static_cast<uint8_t>(ct),
                          static_cast<uint8_t>(pr),
                          static_cast<uint8_t>(jr));
    return command_response(ini);
}

// hardcoded for 512 bytes
bool C328::set_pkg_size()
{
    C328CommandPacket setpkgsize(0x06, 0x08, 0x00, 0x02, 0x00);
    return command_response(setpkgsize);
}

bool C328::snapshot(enum SnapshotType st)
{
    C328CommandPacket snap(0x05, static_cast<uint8_t>(st), 0x00, 0x00, 0x00);
    return command_response(snap);
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
        bool response = command_response(gpix);
        if (response)
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
    }
    else if (datval.first.is_nak())
    {
        struct NakFields nf;
        datval.first.nak_fields(&nf);
        cerr << "Camera error[" << static_cast<int>(nf.errcode)
             << "] snapshot: " << datval.first.nak_error() << endl;
        return;
    }
    if(usleep(10000) != 0)
    {
        cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
        close_port();
        return;
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
        // put in a short sleep after each package
        if(usleep(1000) != 0)
        {
            cerr << "Error " << errno << " during sleep: " << strerror(errno) << endl;
            close_port();
            return;
        }
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
    if (!snapshot(Compressed))
        if (!snapshot(Compressed))
            return make_pair(0, nullptr);

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
    if (!snapshot(Uncompressed))
        return make_pair(0, nullptr);

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
