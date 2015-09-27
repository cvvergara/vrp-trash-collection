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

#ifdef OSRMCLIENT
#include "osrmclient.h"
#endif

#include "logger.h"
#include "timer.h"
#include "stats.h"
#include "osrmclient.h"

#include "trashconfig.h"
#include "truckManyVisitsDump.h"
#include "fleetOpt.h"

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
    //
    void solve();

private:
    container_t *mContainers;
    unsigned int mContainersCount;
    otherloc_t *mOtherLocs;
    unsigned int mOtherlocsCount;
    vehicle_t *mVehicles;
    unsigned int mVehiclesCount;
    ttime_t *mTimeTable;
    unsigned int mTimeTableCount;

    unsigned int mTteration;
    unsigned int mCheck;
    bool mRightSide;
    bool mReady;
};
