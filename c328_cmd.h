#ifndef C328_CMD_H_
#define C328_CMD_H_

#include <cstdint>
#include <string>

struct AckFields
{
    uint8_t cmdid;
    uint8_t counter;
    uint16_t pkgid;
};

struct NakFields
{
    uint8_t counter;
    uint8_t errcode;
};

struct DataFields
{
    uint8_t pixtype;
    uint32_t datasz;
};

class C328CommandPacket
{
public:
    C328CommandPacket() = default;
    explicit C328CommandPacket(uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6);
    explicit C328CommandPacket(const uint8_t* buf);
    ~C328CommandPacket() {;}

    const uint8_t* packet() const {return _data;}

    bool is_sync() const;
    bool is_nak() const;
    bool is_ack() const;
    bool is_data() const;
    
    void ack_fields(struct AckFields* fld) const;
    void nak_fields(struct NakFields* fld) const;
    void data_fields(struct DataFields* fld) const;

    // dump to std::err, dir is True for write
    void debug(bool dir) const;

    const std::string& nak_error() const;
    
private:
    uint8_t _data[6];
};

#endif // C328_CMD_H_
