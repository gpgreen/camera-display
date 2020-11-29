#include "c328_cmd.h"
#include <cstring>
#include <cstdio>

using namespace std;

C328CommandPacket::C328CommandPacket(uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5, uint8_t b6)
{
    _data[0] = 0xAA;
    _data[1] = b2;
    _data[2] = b3;
    _data[3] = b4;
    _data[4] = b5;
    _data[5] = b6;
}

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
    fld->errcode = _data[4];
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

struct err_to_msg 
{
    uint8_t errcode;
    std::string errmsg;
};

const struct err_to_msg nak_errs[] = {
    {0x00, "Not a known NAK errorcode, or not a NAK reply, no error message available"},
    {0x01, "Picture Type Error"},
    {0x02, "Picture Up Scale"},
    {0x03, "Picture Scale Error"},
    {0x04, "Unexpected Reply"},
    {0x05, "Send Picture Timeout"},
    {0x06, "Unexpected Command"},
    {0x07, "SRAM JPEG Type Error"},
    {0x08, "SRAM JPEG Size Error"},
    {0x09, "Picture Format Error"},
    {0x0A, "Picture Size Error"},
    {0x0B, "Parameter Error"},
    {0x0C, "Send Register Timeout"},
    {0x0D, "Command ID Error"},
    {0x0F, "Picture Not Ready"},
    {0x10, "Transfer Package Number Error"},
    {0x11, "Set Transfer Package Size Wrong"},
    {0xF0, "Command Header Error"},
    {0xF1, "Command Length Error"},
    {0xF5, "Send Picture Error"},
    {0xFF, "Send Command Error"},
};

const std::string& C328CommandPacket::nak_error() const
{
    if (is_nak()) {
        for (size_t i=0; i<sizeof(nak_errs); i++)
            if (nak_errs[i].errcode == _data[4])
                return nak_errs[i].errmsg;
    }
    return nak_errs[0].errmsg;
}
