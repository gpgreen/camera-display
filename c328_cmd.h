#ifndef C328_CMD_H_
#define C328_CMD_H_

#include <cstdint>

enum C328Errors {
    PictureTypeError = 0,
    PictureUpScale,
    PictureScaleError,
    UnexpectedReply,
    SendPictureTimeout,
    UnexpectedCommand,
    SRAMJPEGTypeError,
    SRAMJPEGSizeError,
    PictureFormatError,
    PictureSizeError,
    ParameterError,
    SendRegisterTimeout,
    CommandIDError,
    PictureNotReady,
    TransferPackageNumberError,
    SetTransferPackageSizeWrong,
    CommandHeaderError,
    CommandLengthError,
    SendPictureError,
    SendCommandError
};

struct AckFields
{
    uint8_t cmdid;
    uint8_t counter;
    uint16_t pkgid;
};

struct NakFields
{
    uint8_t counter;
    enum C328Errors err;
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
    
private:
    uint8_t _data[6];
};

#endif // C328_CMD_H_
