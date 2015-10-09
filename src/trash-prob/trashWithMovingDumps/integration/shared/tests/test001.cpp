#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>
// Command line options
#include <boost/program_options.hpp>
// If exist
#include <boost/filesystem.hpp>

#include "vrptools.h"

using namespace boost::program_options;

std::string base;
std::vector<std::string> dataFiles;


// tests/test001 --base ../tests/InputFiles/ rivera

/*
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
*/

void to_cout(const std::vector<std::string> &v)
{
  std::copy(v.begin(), v.end(),
    std::ostream_iterator<std::string>{std::cout, "\n"});
}


void checkData( const std::string &baseDir, const std::vector<std::string> &files )
{
    exit(0);
}

void checkOSRM()
{
    VRPTools vrp;
    bool ret = vrp.checkOsrmClient();
    if (ret) {
        std::cout << "OSRM datastore is loaded in shared memory and available. Ok!" << std::endl;
        exit(0);
    } else {
        std::cout << "OSRM datastore is not loaded!" << std::endl;
        exit(-1);
    }
}

void solve( const std::string &baseDir, const std::vector<std::string> &files )
{
    if ( boost::filesystem::exists(baseDir) && boost::filesystem::is_directory(baseDir))
    {
        for (auto & element : files) {
            std::cout << "baseDir: " << baseDir << std::endl;
            std::cout << "file: " << element << std::endl;
            VRPTools vrp;
            vrp.readDataFromFiles(baseDir + element);
            vrp.solve();
        }
        exit(0);
    } else {
        std::cerr << "Directory ("<< base << ") dont exist!" << std::endl;
    }
}

int main(int argc, char **argv)
{
    try
      {
        options_description desc{"Options"};
        desc.add_options()
          ("help,h", "Help screen")
          ("checkOSRM", "Check OSRM")
          ("checkData", "Check the data. Don't process anything.")
          ("base", value<std::string>(), "Base directory")
          ("files", value<std::vector<std::string>>()->multitoken()->zero_tokens()->composing(), "File begin without endings");

        positional_options_description pos_desc;
        pos_desc.add("files", -1);

        command_line_parser parser{argc, argv};
        parser.options(desc).positional(pos_desc).allow_unregistered();
        parsed_options parsed_options = parser.run();

        variables_map vm;
        store(parsed_options, vm);
        notify(vm);

        if (vm.count("help")) {
          std::cout << desc << std::endl;
          std::cout << "Example:"<< std::endl;
          std::cout << "\ttests/test001 --base ../tests/InputFiles/ rivera"<< std::endl;
        }

        if (vm.count("checkOSRM")) {
            checkOSRM();
        }

        if (vm.count("base")) {
            base = vm["base"].as<std::string>();
        } else {
            base = "./";
        }

        if (vm.count("files")) {
            if (vm.count("checkData")) {
                checkData( base, vm["files"].as<std::vector<std::string>>() );
            } else {
                solve( base, vm["files"].as<std::vector<std::string>>() );
            }
        }

      } catch (const error &ex) {
        std::cerr << ex.what() << std::endl;
      }
}
