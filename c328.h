#ifndef C328_H_
#define C328_H_

#include <string>
#include <cstring>
#include <utility>
#include <sys/select.h>

// forward class definitions
class C328CommandPacket;

enum ColorType {
    TwoBitGrayScale = 1,
    FourBitGrayScale = 2,
    EightBitGrayScale = 3,
    TwelveBitColor = 5,
    SixteenBitColor = 6,
    JPEG = 7
};

enum PreviewResolution {
    P80x60 = 1,
    P160x120 = 3
};

enum JPEGResolution {
    J80x64 = 1,
    J160x128 = 3,
    J320x240 = 5,
    J640x480 = 7
};
    
enum SnapshotType {
    Compressed = 0,
    Uncompressed = 1
};

enum PictureType {
    Snapshot = 1,
    Preview = 2,
    JpegPreview = 5
};

class C328
{
public:
    explicit C328(const std::string& serport, const std::string& baudrate);
    ~C328();

    // non-hdwr methods
    void set_debug(bool on) {_debug = on;}
    bool is_connected() const;

    // hdwr commands
    bool initial(enum ColorType ct, enum PreviewResolution pr, enum JPEGResolution jr);
    void power_off();
    void reset(bool state_only);
    void sync();
    bool set_pkg_size();
    bool snapshot(enum SnapshotType st);
    void get_picture(enum PictureType pt, uint8_t* pixtype, uint32_t* datasz);
    void get_data_packages(uint32_t datasz, uint8_t* pixbuf);
    void get_image_data(uint32_t datasz, uint8_t* pixbuf);
    
    // composite methods
    std::pair<uint32_t, uint8_t*> jpeg_snapshot();
    std::pair<uint32_t, uint8_t*> uncompressed_snapshot();
    std::pair<uint32_t, uint8_t*> jpeg_preview();
    std::pair<uint32_t, uint8_t*> uncompressed_preview();
    
    // serial port
    bool has_recvd_bytes() const;
    
private:

    bool command_response(const C328CommandPacket& pkt);
    bool read_bytes(uint8_t* buf, ssize_t nbytes);
    void close_port();
    bool write_pkt(const C328CommandPacket& pkt);
    std::pair<C328CommandPacket, bool> read_pkt();
    
    int _serial_port;
    bool _is_sync;
    bool _debug;
};

#endif // C328_H_
