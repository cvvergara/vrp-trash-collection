#include <assert.h>
#include <fstream>

#include "loadfromfiles.h"

LoadFromFiles::LoadFromFiles(std::string &filePrefix) {
    load_containers( filePrefix + ".containers.txt" );
    load_otherlocs( filePrefix + ".otherlocs.txt" );
    load_vehicles( filePrefix + ".vehicles.txt" );
    load_ttimes( filePrefix + ".dmatrix-time.txt" );
}


void LoadFromFiles::load_containers(std::string infile) {
    std::ifstream in( infile.c_str() );
    std::string line;
    mContainers.clear();
    while ( getline( in, line ) ) {
        if ( line[0] == '#' ) continue;
        container_t container = parseContainer( line );
        mContainers.push_back( container );
    }
    in.close();
}

void LoadFromFiles::load_otherlocs(std::string infile) {
    std::ifstream in( infile.c_str() );
    std::string line;
    mOtherLocs.clear();
    while ( getline( in, line ) ) {
        if ( line[0] == '#' ) continue;
        otherloc_t otherloc = parseOtherloc( line );
        mOtherLocs.push_back( otherloc );
    }
    in.close();
}

void LoadFromFiles::load_vehicles(std::string infile) {
    std::ifstream in( infile.c_str() );
    std::string line;
    mVehicles.clear();
    while ( getline( in, line ) ) {
        if ( line[0] == '#' ) continue;
        vehicle_t vehicle = parseVehicle( line );
        mVehicles.push_back( vehicle );
    }
    in.close();
}

void LoadFromFiles::load_ttimes(std::string infile) {
    std::ifstream in( infile.c_str() );
    std::string line;
    mTimeTable.clear();
    while ( getline( in, line ) ) {
        if ( line[0] == '#' ) continue;
        ttime_t ttime = parseTtime( line );
        mTimeTable.push_back( ttime );
    }
    in.close();
}

container_t LoadFromFiles::parseContainer(std::string line) {
    std::istringstream buffer( line );
    container_t container;
    buffer >> container.id;
    buffer >> container.x;
    buffer >> container.y;
    buffer >> container.open;
    buffer >> container.close;
    buffer >> container.demand;
    buffer >> container.service;
    buffer >> container.sid;
    return container;
}

otherloc_t LoadFromFiles::parseOtherloc(std::string line) {
    std::istringstream buffer( line );
    otherloc_t otherloc;
    buffer >> otherloc.id;
    buffer >> otherloc.x;
    buffer >> otherloc.y;
    buffer >> otherloc.open;
    buffer >> otherloc.close;
    return otherloc;
}

vehicle_t LoadFromFiles::parseVehicle(std::string line) {
    std::istringstream buffer( line );
    vehicle_t vehicle;
    buffer >> vehicle.vid;
    buffer >> vehicle.start_id;
    buffer >> vehicle.dump_id;
    buffer >> vehicle.end_id;
    buffer >> vehicle.dumpservicetime;
    buffer >> vehicle.capacity;
    buffer >> vehicle.starttime;
    buffer >> vehicle.endtime;
    return vehicle;
}

ttime_t LoadFromFiles::parseTtime(std::string line) {
    std::istringstream buffer( line );
    ttime_t ttime;
    buffer >> ttime.from_id;
    buffer >> ttime.to_id;
    buffer >> ttime.ttime;
    return ttime;
}



container_t *LoadFromFiles::getContainers(unsigned int &container_count) {
    assert( mContainers.size() );
    container_count = mContainers.size();
    container_t *pg_containers = (container_t *) malloc( container_count * sizeof(container_t) );
    assert( pg_containers );

    for ( unsigned int i=0; i<container_count; i++ ) {
        pg_containers[i].id      = mContainers[i].id;
        pg_containers[i].x       = mContainers[i].x;
        pg_containers[i].y       = mContainers[i].y;
        pg_containers[i].open    = mContainers[i].open;
        pg_containers[i].close   = mContainers[i].close;
        pg_containers[i].service = mContainers[i].service;
        pg_containers[i].demand  = mContainers[i].demand;
        pg_containers[i].sid     = mContainers[i].sid;
    }
    return pg_containers;
}

otherloc_t *LoadFromFiles::getOtherlocs(unsigned int &otherloc_count) {
    assert( mOtherLocs.size() );
    otherloc_count = mOtherLocs.size();
    otherloc_t *pg_otherlocs = (otherloc_t *) malloc( otherloc_count * sizeof(otherloc_t) );
    assert( pg_otherlocs );

    for ( unsigned int i=0; i<otherloc_count; i++ ) {
        pg_otherlocs[i].id    = mOtherLocs[i].id;
        pg_otherlocs[i].x     = mOtherLocs[i].x;
        pg_otherlocs[i].y     = mOtherLocs[i].y;
        pg_otherlocs[i].open  = mOtherLocs[i].open;
        pg_otherlocs[i].close = mOtherLocs[i].close;
    }
    return pg_otherlocs;
}

vehicle_t *LoadFromFiles::getVehicles(unsigned int &vehicle_count) {
    assert( mVehicles.size() );
    vehicle_count = mVehicles.size();
    vehicle_t *pg_vehicles = (vehicle_t *) malloc( vehicle_count * sizeof(vehicle_t) );
    assert( pg_vehicles );

    for ( unsigned int i=0; i<vehicle_count; i++ ) {
        pg_vehicles[i].vid              = mVehicles[i].vid;
        pg_vehicles[i].start_id         = mVehicles[i].start_id;
        pg_vehicles[i].dump_id          = mVehicles[i].dump_id;
        pg_vehicles[i].end_id           = mVehicles[i].end_id;
        pg_vehicles[i].capacity         = mVehicles[i].capacity;
        pg_vehicles[i].dumpservicetime  = mVehicles[i].dumpservicetime;
        pg_vehicles[i].starttime        = mVehicles[i].starttime;
        pg_vehicles[i].endtime          = mVehicles[i].endtime;
    }
    return pg_vehicles;
}

ttime_t *LoadFromFiles::getTtimes(unsigned int &ttime_count) {
    ttime_count = mTimeTable.size();

    // if not ttime data then return NULL
    if (ttime_count == 0) return (ttime_t *) 0;

    ttime_t *pg_ttimes = (ttime_t *) malloc( ttime_count * sizeof(ttime_t) );
    assert( pg_ttimes );

    for ( unsigned int i=0; i<ttime_count; i++ ) {
        pg_ttimes[i].from_id = mTimeTable[i].from_id;
        pg_ttimes[i].to_id = mTimeTable[i].to_id;
        pg_ttimes[i].ttime = mTimeTable[i].ttime;
    }
    return pg_ttimes;
}
