// enviroment variables recognized:
//
// BAUDRATE=xxxx set the baudrate on the serial port, note the camera only recognizes a few values
// DEBUGSERIAL if set to any value will print the command packets and replies to stderr

#include "c328.h"

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: camera_display picture_filename" << std::endl;
        return -1;
    }
    char* base = argv[1];

    std::string baudrate("9600");
    
    // check environment for baudrate
    char* envval = getenv("BAUDRATE");
    if (envval != nullptr)
        baudrate = envval;

    C328 dev("/dev/ttyUSB0", baudrate);

    // check environment for debug print
    envval = getenv("DEBUGSERIAL");
    if (envval != nullptr) {
        std::cerr << "Turning serial command packet printing on" << std::endl;
        dev.set_debug(true);
    }

    dev.sync();

    if (dev.is_connected())
    {
        std::cerr << "Connected to camera\n";
        bool retval = false;
        for (int i=0; i<5 && !retval; i++)
        {
            retval = dev.initial(TwoBitGrayScale, P160x120, J320x240);
        }
        if (!retval)
        {
            std::cerr << "not initialized" << std::endl;
            return -1;
        }
        std::cerr << "initial setup done\n";
        retval = false;
        for (int i=0; i<5 && !retval; i++)
        {
            retval = dev.set_pkg_size();
        }
        if (!retval)
        {
            std::cerr << "set pkg size failed" << std::endl;
            return -1;
        }
        std::cerr << "set pkg size done\n";
        for (int i=0; i<100; i++)
        {
            auto result = dev.jpeg_snapshot();
            if (result.first == 0)
                continue;

            // write the image file to disk
            char fname[1024];
            snprintf(fname, 1024, "%s%02d.jpg", base, i);
            
            FILE* f = fopen(fname, "w");
            if (f == nullptr)
            {
                std::cerr << "Unable to open file: '" << argv[1] << "' for write" << std::endl;
                return -1;
            }
                // write the file
            size_t n = fwrite(result.second, 1, result.first, f);
            if (n != result.first)
            {
                std::cerr << "Error " << errno << " during write of file: " << strerror(errno) << std::endl;
            }
            fclose(f);
        
            delete [] result.second;
        }
        return 0;
    }
    std::cerr << "Not connected to camera\n";
    return 1;
}
