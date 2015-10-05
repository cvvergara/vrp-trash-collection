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
    std::string fileDir = "../tests/InputFiles/";
    VRPTools vrp;
    std::cout << "OSRM checkOsrmClient: " << vrp.checkOsrmClient() << std::endl;
    std::cout << "OSRM available: " << vrp.osrmAvailable() << std::endl;
    vrp.readDataFromFiles(fileDir + "rivera");
    //vrp.readDataFromFiles(fileDir + "a_du_rm_cl_08");
    //std::cout << "OSRM check: " << vrp.check() << std::endl;
    vrp.solve();
    return 0;
}
