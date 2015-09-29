#include <stdlib.h> /* malloc, calloc, realloc, free */

#include "vrptools.h"
#include "loadfromfiles.h"

#include "trashconfig.h"
#include "trashprob.h"
#include "truckManyVisitsDump.h"
#include "fleetOpt.h"

VRPTools::VRPTools():
    mRightSide(false),
    mReady(false),
    mNIters(100),
    mContainersCount(0),
    mVehiclesCount(0),
    mOtherLocsCount(0),
    mTimeTableCount(0)
{
    // Set logs
    mLogDir = fs::temp_directory_path();
    mLogFile = fs::path("libvrptools");
    mUseOsrm = osrmAvailable();

#ifdef DOVRPLOG
  if ( not google::IsGoogleLoggingInitialized() ) {
    FLAGS_log_dir = mLogDir.c_str();
    google::InitGoogleLogging( mLogFile.c_str() );
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::FATAL;
    FLAGS_minloglevel = google::INFO;
    FLAGS_logbufsecs = 0;
    // Shutdown
    //google::ShutdownGoogleLogging();
  }
#endif

}

void VRPTools::setLogFilePath(std::string logDir, std::string logFileName) {
    fs::path p(logDir);
    if (fs::exists(p) && fs::is_directory(p)) {
        if ( fs::portable_name(logFileName) ) {
            mLogFile = fs::path(logFileName);
            mLogDir = p;
        }
    }
}

bool VRPTools::osrmAvailable() {
    return osrmi->getConnection();
}

void VRPTools::setUseOsrm(bool opt) {
    if (opt) {
        if ( osrmi->getConnection() ) {
            mUseOsrm = opt;
        } else {
            mUseOsrm = false;
        }
    } else {
        mUseOsrm = opt;
    }
}

bool VRPTools::checkOsrmClient()
{
    bool testResult = false;
#ifdef OSRMCLIENT
    osrmi->useOsrm(true);
    testResult = osrmi->testOsrmClient(
        -34.905113, -56.157043,
        -34.906807, -56.158463,
        -34.9076,   -56.157028
    );
#ifdef VRPMINTRACE
    if (testResult)
        DLOG(INFO) << "osrm test passed";
    else
        DLOG(INFO) << "osrm test FAIL";
#endif  // VRPMINTRACE
#endif  // OSRMCLIENT
    return testResult;
}

bool VRPTools::readDataFromFiles(std::string fileBasePath)
{
    // .containers.txt .otherlocs.txt .vehicles.txt .dmatrix-time.txt
    LoadFromFiles loader(fileBasePath);
    mContainers = loader.getContainers(mContainersCount);
    mOtherLocs = loader.getOtherlocs(mOtherLocsCount);
    mVehicles = loader.getVehicles(mVehiclesCount);
    mTimeTable = loader.getTtimes(mTimeTableCount);
#ifdef VRPMINTRACE
    DLOG(INFO) << "mContainersCount: " << mContainersCount;
    DLOG(INFO) << "mOtherLocsCount: " << mOtherLocsCount;
    DLOG(INFO) << "mVehiclesCount: " << mVehiclesCount;
    DLOG(INFO) << "mTimeTableCount: " << mTimeTableCount;
#endif
}

bool VRPTools::check()
{
    osrmi->useOsrm( mUseOsrm );
    if (mContainers && mOtherLocs && mTimeTable && mVehicles) {
        TrashProb prob(
            mContainers,
            mContainersCount,
            mOtherLocs,
            mOtherLocsCount,
            mTimeTable,
            mTimeTableCount,
            mVehicles,
            mVehiclesCount,
            1 // Only check, not run!
        );
        bool ret = false;
        if ( prob.isValid() or prob.getErrorsString().size() == 0 ) {
            ret = true;
        } else {
            ret = false;
        #ifdef VRPMINTRACE
            DLOG(INFO) << "Errors: " << prob.getErrorsString();
        #endif
            //*data_err_msg = strdup( prob.getErrorsString().c_str() );
        }
        twc->cleanUp();
        return ret;
    } else {
        return false;
    }
}

void VRPTools::solve()
{
    osrmi->useOsrm( mUseOsrm );
    if (mContainers && mOtherLocs && mTimeTable && mVehicles) {
        TrashProb prob(
            mContainers,
            mContainersCount,
            mOtherLocs,
            mOtherLocsCount,
            mTimeTable,
            mTimeTableCount,
            mVehicles,
            mVehiclesCount,
            0 // Not check, run!
        );

    #ifdef DOVRPLOG
        DLOG(INFO) << "Datos del problema";
        prob.dumpdataNodes();
        prob.dumpDepots();
        prob.dumpDumps();
        prob.dumpPickups();
    #endif

        // If not valid -> exit
        if ( !prob.isValid() ) {
        #ifdef DOVRPLOG
            DLOG(INFO) << "Problema no es válido";
            DLOG(INFO) << prob.getErrorsString();
        #endif
            twc->cleanUp();
            return;
        }

        TruckManyVisitsDump tp( prob );
        tp.process(0);
    #ifdef DOVRPLOG
        DLOG(INFO) << "Initial solution: 0 is best";
    #endif
        double best_cost = 9999999;
        Solution best_sol( tp );
        best_cost = best_sol.getCostOsrm();

        for (int icase = 1; icase < 7; ++icase) {
        #ifdef DOVRPLOG
            DLOG(INFO) << "initial solution: " << icase;
        #endif
            tp.process(icase);
            if (best_cost > tp.getCostOsrm()) {
        #ifdef DOVRPLOG
            DLOG(INFO) << "initial solution: " << icase << " is best";
        #endif
            best_cost = tp.getCostOsrm();
            best_sol = tp;
          }
        }

        Optimizer optSol(best_sol, mNIters);
        if (best_cost > optSol.getCostOsrm()) {
            best_cost = optSol.getCostOsrm();
            best_sol = optSol;
        }

    #ifdef DOVRPLOG
        DLOG(INFO) << "=-=-=-=-=-=- OPTIMIZED SOLUTION -=-=-=-=-=-=-=";
        DLOG(INFO) << "Number of containers: " << best_sol.countPickups();
        best_sol.dumpCostValues();
        DLOG(INFO) << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=";
        best_sol.tau();
        DLOG(INFO) << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=";
    #endif
        // Limpio
        twc->cleanUp();
    } else {

    }
}
