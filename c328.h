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

    void sync();
    bool is_connected() const;
    bool has_recvd_bytes() const;
    
private:

    void close_port();
    bool write_pkt(const C328CommandPacket& pkt);
    std::pair<C328CommandPacket, bool> read_pkt();
    
    int _serial_port;
    bool _is_sync;
    fd_set _read_fds;
    fd_set _write_fds;
    fd_set _except_fds;
    
};

#endif // C328_H_
