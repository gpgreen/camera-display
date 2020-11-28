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

    void reset(bool state_only);
    void sync();
    void setup();
    void set_pkg_size();
    void snapshot();
    void get_picture(uint8_t* pixtype, uint32_t* datasz);
    void get_data_packages(uint32_t datasz, uint8_t* pixbuf);
    
    bool is_connected() const;
    bool has_recvd_bytes() const;
    
private:

    bool read_bytes(uint8_t* buf, ssize_t nbytes);
    void close_port();
    bool write_pkt(const C328CommandPacket& pkt, bool debug=false);
    std::pair<C328CommandPacket, bool> read_pkt(bool debug=false);
    
    int _serial_port;
    bool _is_sync;
};

#endif // C328_H_
