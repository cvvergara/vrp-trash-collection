#include <iostream>
#include <sstream>
#include <string>
#include <exception>
#include <string.h>

#include "pg_types_vrp.h"

class LoadFromFiles {

  public:
    LoadFromFiles( std::string &filePrefix ) {
        load_containers( filePrefix + ".containers.txt" );
        load_otherlocs( filePrefix + ".otherlocs.txt" );
        load_vehicles( filePrefix + ".vehicles.txt" );
        load_ttimes( filePrefix + ".dmatrix-time.txt" );
    };

    container_t *getContainers(unsigned int &container_count);
    otherloc_t *getOtherlocs(unsigned int &otherloc_count);
    vehicle_t *getVehicles(unsigned int &vehicle_count);
    ttime_t *getTtimes(unsigned int &ttime_count);

  private:
    void load_containers( std::string infile );
    void load_otherlocs( std::string infile );
    void load_vehicles( std::string infile );
    void load_ttimes( std::string infile );
    container_t parseContainer( std::string line );
    otherloc_t parseOtherloc( std::string line );
    vehicle_t parseVehicle( std::string line );
    ttime_t parseTtime( std::string line );
    std::vector<container_t> mContainers;
    std::vector<otherloc_t> mOtherLocs;
    std::vector<vehicle_t> mVehicles;
    std::vector<ttime_t> mTimeTable;

};  // end of class LoadFromFiles
