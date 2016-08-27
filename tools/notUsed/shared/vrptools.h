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

/**
 * @class VRPTools
 * @brief Main class for vrptools shared library.
 *
 *  A more elaborate class description.
 */
class VRPTools {

public:
    VRPTools();
    /**
     * @brief A member to set containers
     * @param containers pointer (array) of containers.
     * @param count number of elements in array.
     */
    void setContainers( container_t *containers, unsigned int count ) {
        mContainers = containers;
        mContainersCount = count;
    }
    /**
     * @brief A member to get containers
     * @return pointer (array) of containers
     */
    container_t* getContainers() { return mContainers; }
    // Other locs
    void setLocs( otherloc_t *otherloc, unsigned int count ) {
        mOtherLocs = otherloc;
        mOtherLocsCount = count;
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
    bool readDataFromFiles(std::string fileBasePath);
    // Solve the problem
    bool check();
    // Solve the problem
    void solve();

private:
    // Apuntan al primer elemento del array. Cada elemento tiene la estructura.
    container_t *mContainers; ///< Containers
    unsigned int mContainersCount; ///< Number of containers
    otherloc_t *mOtherLocs;
    unsigned int mOtherLocsCount;
    vehicle_t *mVehicles;
    unsigned int mVehiclesCount;
    ttime_t *mTimeTable;
    unsigned int mTimeTableCount;

    unsigned int mNIters; ///< Number of iterations before stop in the solver
    bool mRightSide; ///< Get containers from right side
    bool mReady; ///< Detailed description after the member
    bool mUseOsrm; ///< Use OSRM

    bool mLogging; ///< Log vrptools actions
    fs::path mLogDir; ///< Logging directory path
    fs::path mLogFile; ///< Logging file name (not path)
};
