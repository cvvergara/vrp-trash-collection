#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>

#include "vrptools.h"

int main(int argc, char **argv)
{
    VRPTools vrp;
    std::cout << "OSRM checkOsrmClient: " << vrp.checkOsrmClient() << std::endl;
    std::cout << "OSRM available: " << vrp.osrmAvailable() << std::endl;
    return 0;
}
