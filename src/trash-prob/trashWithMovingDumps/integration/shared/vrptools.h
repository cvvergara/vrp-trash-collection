#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <string>

// Boost filesystem
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif

/*
#ifdef OSRMCLIENT
#include "osrmclient.h"
#endif
*/

#include "pg_types_vrp.h"

class VRPTools {

public:
    VRPTools();
    // Containers
    void setContainers( container_t *containers, unsigned int count ) {
        mContainers = containers;
        mContainersCount = count;
    }
    container_t* getContainers() { return mContainers; }
    // Other locs
    void setLocs( otherloc_t *otherloc, unsigned int count ) {
        mOtherLocs = otherloc;
        mOtherlocsCount = count;
    }
    otherloc_t* getOtherLoc() { return mOtherLocs; }
    // Vehicles
    void setVehicles( vehicle_t *vehicles, unsigned int count ) {
        mVehicles = vehicles;
        mVehiclesCount = count;
    }
    vehicle_t* getVehicles() { return mVehicles; }
    // Initial table time
    void setTimeTable( ttime_t *ttable, unsigned int count ) {
        mTimeTable = ttable;
        mTimeTableCount = count;
    }
    ttime_t* getTimeTable() { return mTimeTable; }

    // Logeo
    void setLogFilePath(std::string logDir, std::string logFileName);
    fs::path getLogFilePath() { return mLogDir / mLogFile; }

    // Levante por la derecha
    void setRightSide(bool opt) { mRightSide = opt; }
    bool getRightSide() { return mRightSide; }
    // Levante por la derecha
    bool osrmAvailable();
    void setUseOsrm(bool opt);
    bool getUseOsrm() { return mUseOsrm; }
    // Levante por la derecha
    void setNIters(bool opt) { mNIters = opt; }
    bool getNIters() { return mNIters; }
    // Test if OSRM datastore is alive
    bool checkOsrmClient();
    // Read all data from file
    bool readDataFromFile(std::string fileBasePath);
    // Solve the problem
    bool check();
    // Solve the problem
    void solve();

private:
    // Apuntan al primer elemento del array. Cada elemento tiene la estructura.
    container_t *mContainers;
    unsigned int mContainersCount;
    otherloc_t *mOtherLocs;
    unsigned int mOtherlocsCount;
    vehicle_t *mVehicles;
    unsigned int mVehiclesCount;
    ttime_t *mTimeTable;
    unsigned int mTimeTableCount;

    unsigned int mNIters;
    bool mCheck;
    bool mRightSide;
    bool mReady;
    bool mUseOsrm;

    fs::path mLogDir;
    fs::path mLogFile;
};
