#ifndef C328_H_
#define C328_H_

#include <string>
#include <cstring>
#include <utility>
#include <sys/select.h>

// forward class definitions
class C328CommandPacket;

class C328
{
public:
    explicit C328(const std::string& serport);
    ~C328();

    // non-hdwr methods
    void set_debug(bool on) {_debug = on;}
    bool is_connected() const;

    // hdwr commands
    void reset(bool state_only);
    void sync();
    void initial();
    void set_pkg_size();
    void snapshot();
    void get_picture(uint8_t* pixtype, uint32_t* datasz);
    void get_data_packages(uint32_t datasz, uint8_t* pixbuf);

    // composite methods
    std::pair<uint32_t, uint8_t*> jpeg_snapshot();
    std::pair<uint32_t, uint8_t*> uncompressed_snapshot();
    std::pair<uint32_t, uint8_t*> jpeg_preview();
    std::pair<uint32_t, uint8_t*> uncompressed_preview();
    
    // serial port
    bool has_recvd_bytes() const;
    
private:

    bool read_bytes(uint8_t* buf, ssize_t nbytes);
    void close_port();
    bool write_pkt(const C328CommandPacket& pkt);
    std::pair<C328CommandPacket, bool> read_pkt();
    
    int _serial_port;
    bool _is_sync;
    bool _debug;
};

#endif // C328_H_
