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
    int slen = strlen(base);
    char* ext = argv[1] + slen - 3;
    char basename[1024];
    strncpy(basename, base, slen - 4);
    
    C328 dev("/dev/ttyS4");
    dev.sync();
    if (dev.is_connected())
    {
        std::cerr << "Connected to camera\n";
        dev.setup();
        std::cerr << "setup done\n";
        dev.set_pkg_size();
        std::cerr << "set pkg size done\n";
        for (int i=0; i<100; i++)
        {
            dev.snapshot();
            uint8_t pixtype;
            uint32_t datasz;
            dev.get_picture(&pixtype, &datasz);
            if (pixtype == 0)
            {
                std::cerr << "Error during get picture\n";
                return 1;
            }
            //std::cerr << "Pixtype: " << static_cast<int>(pixtype) << " size: " << datasz << std::endl;

            uint8_t* pixbuf = new uint8_t[datasz];

            dev.get_data_packages(datasz, pixbuf);

            // open the file
            char fname[1024];
            snprintf(fname, 1024, "%s%02d.%s", base, i, ext);
            
            FILE* f = fopen(argv[1], "w");
            if (f == nullptr)
            {
                std::cerr << "Unable to open file: '" << argv[1] << "' for write" << std::endl;
                return -1;
            }
                // write the file
            size_t n = fwrite(pixbuf, 1, datasz, f);
            if (n != datasz)
            {
                std::cerr << "Error " << errno << " during write of file: " << strerror(errno) << std::endl;
            }
            fclose(f);
        
            delete [] pixbuf;
        }
        
        return 0;
    }
    std::cerr << "Not connected to camera\n";
    return 1;
}
