#include "c328.h"

#include <iostream>
#include <cstdio>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "usage: camera_display picture_filename" << std::endl;
        return -1;
    }
    char* base = argv[1];

    C328 dev("/dev/ttyS4");
    dev.set_debug(true);
    dev.sync();
    if (dev.is_connected())
    {
        std::cerr << "Connected to camera\n";
        dev.initial();
        std::cerr << "setup done\n";
        dev.set_pkg_size();
        std::cerr << "set pkg size done\n";
        for (int i=0; i<100; i++)
        {
            auto result = dev.jpeg_snapshot();
            if (result.first == 0)
                continue;
            
            // open the file
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
