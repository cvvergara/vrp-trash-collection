#include "vrptools.h"

#include "trashconfig.h"
#include "truckManyVisitsDump.h"
#include "fleetOpt.h"

/*
void VRPTools::setx(int newx) { 
    myx = newx; 
}

int  VRPTools::getx() { 
    return myx; 
}
*/

VRPTools::VRPTools()
{
#ifdef DOVRPLOG
  if ( not google::IsGoogleLoggingInitialized() ) {
    FLAGS_log_dir = "/tmp/";
    google::InitGoogleLogging( "libvrptools" );
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::FATAL;
    FLAGS_minloglevel = google::INFO;
    FLAGS_logbufsecs = 0;
  }
#endif

#ifdef OSRMCLIENT
    osrmi->useOsrm(true);
    bool testResult = osrmi->testOsrmClient(
        -34.905113, -56.157043,
        -34.906807, -56.158463,
        -34.9076,   -56.157028
    );
   // remove the following comment when testing OSRM only
   // assert(true==false);
#ifdef VRPMINTRACE
    if (testResult)
        DLOG(INFO) << "osrm test passed";
    else
        DLOG(INFO) << "osrm test FAIL";
#endif  // VRPMINTRACE
#endif  // OSRMCLIENT

}

void VRPTools::solve()
{

}
