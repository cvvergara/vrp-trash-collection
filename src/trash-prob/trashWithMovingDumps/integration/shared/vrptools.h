#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>

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
    void setContainers( const container_t *containers, unsigned int count ) {
        mContainers = containers;
        mContainersCount = count;
    }
    const container_t* getContainers() { return mContainers; }
    // Other locs
    void setLocs( const otherloc_t *otherloc, unsigned int count ) {
        mOtherLocs = otherloc;
        mOtherlocsCount = count;
    }
    const otherloc_t* getOtherLoc() { return mOtherLocs; }
    // Vehicles
    void setVehicles( const vehicle_t *vehicles, unsigned int count ) {
        mVehicles = vehicles;
        mVehiclesCount = count;
    }
    const vehicle_t* getVehicles() { return mVehicles; }
    // Initial table time
    void setTimeTable( const ttime_t *ttable, unsigned int count ) {
        mTimeTable = ttable;
        mTimeTableCount = count;
    }
    const ttime_t* getTimeTable() { return mTimeTable; }
    //
    void solve();

private:
    // Apuntan al primer elemento del array. Cada elemento tiene la estructura.
    const container_t *mContainers;
    unsigned int mContainersCount;
    const otherloc_t *mOtherLocs;
    unsigned int mOtherlocsCount;
    const vehicle_t *mVehicles;
    unsigned int mVehiclesCount;
    const ttime_t *mTimeTable;
    unsigned int mTimeTableCount;

    unsigned int mTteration;
    unsigned int mCheck;
    bool mRightSide;
    bool mReady;
};
