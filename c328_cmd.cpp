#include "c328_cmd.h"
#include <cstring>
#include <cstdio>

using namespace std;

C328CommandPacket::C328CommandPacket(const uint8_t* buf)
{
    memcpy(_data, buf, 6);
}

bool C328CommandPacket::is_sync() const
{
    return _data[0] == 0xAA && _data[1] == 0x0D;
}

bool C328CommandPacket::is_ack() const
{
    return _data[0] == 0xAA && _data[1] == 0x0E;
}

bool C328CommandPacket::is_nak() const
{
    return _data[0] == 0xAA && _data[1] == 0x0F;
}

bool C328CommandPacket::is_data() const
{
    return _data[0] == 0xAA && _data[1] == 0x0A;
}

void C328CommandPacket::ack_fields(struct AckFields* fld) const
{
    fld->cmdid = _data[2];
    fld->counter = _data[3];
    fld->pkgid = (_data[5] << 8) | _data[4];
}

void C328CommandPacket::nak_fields(struct NakFields* fld) const
{
    fld->counter = _data[3];
    fld->err = static_cast<enum C328Errors>(_data[4]);
}

void C328CommandPacket::data_fields(struct DataFields* fld) const
{
    fld->pixtype = _data[2];
    fld->datasz = (_data[5] << 16) | (_data[4] << 8) | _data[3];
}

void C328CommandPacket::debug(bool dir) const
{
    fprintf(stderr, "%s %02x %02x %02x %02x %02x %02x\n", dir ? "->" : "<-",
            _data[0], _data[1], _data[2], _data[3], _data[4], _data[5]);
}
