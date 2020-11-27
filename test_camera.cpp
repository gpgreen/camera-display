#include "c328.h"

#include <iostream>

int main(int argc, char* argv[])
{
    C328 dev("/dev/ttyS4");

    dev.sync();
    if (dev.is_connected())
    {
        std::cerr << "Connected to camera\n";
        return 0;
    } else {
        std::cerr << "Not connected to camera\n";
        return 1;
    }
}
