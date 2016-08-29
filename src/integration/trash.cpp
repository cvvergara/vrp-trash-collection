/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/

#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>

#include "baseClasses/logger.h"
#include "baseClasses/timer.h"
#include "baseClasses/stats.h"

#include "baseTrash/trashconfig.h"
#include "initTrash/truckManyVisitsDump.h"
#include "solverTrash/fleetOpt.h"


void Usage()
{
  std::cout << "Usage: trash file (no extension)\n";
  std::cout << "trash file data_path\n";
}


/* Logging Severity Levels
    0   INFO
    1   WARNING
    2   ERROR
    3   FATAL  NOTE FATAL is converted to ERROR when NDEBUG is defined

    DLOG(<level>) << "message";
    DLOG_IF(<level>, <cond>) << "message";
    DLOG_EVERY_N(<level>, <n>) << "message";
    CHECK(<cond>) << "message";  // print "message" and FATAL if false

    http://google-glog.googlecode.com/svn/trunk/doc/glog.html

    GLOG_logtostderr=1 ./bin/trash ...  // to get output to terminal
*/

int main(int argc, char **argv) {

  if ( not google::IsGoogleLoggingInitialized() ) {
    FLAGS_log_dir = "./logs/";
    google::InitGoogleLogging( "vrp_trash_collection" );
    FLAGS_logtostderr = 0;
    FLAGS_stderrthreshold = google::FATAL;
    FLAGS_minloglevel = google::INFO;
    FLAGS_logbufsecs = 0;
  }

  if (argc < 2) {
    Usage();
    return 1;
  }

  std::string infile = argv[1];

  try {
#ifdef DOSTATS
    Timer starttime;
#endif

#ifdef VRPMINTRACE
    //CONFIG->dump("CONFIG");
#endif

#ifdef VRPMINTRACE
   DLOG(INFO) << "log file started for: " << infile;
#endif


   /* testing the server before using */
   { 
       osrmi->useOsrm(true);
       bool testResult = osrmi->testOsrmClient(
               -34.87198931958, -56.190261840820305,
               -34.88156563691107, -56.17189407348633,
               -34.90634621832085, -56.16365432739258
               );
       assert(testResult == true);
   }
#if 0
   /* set to 1 when testing OSRM only */
   assert(true==false);        
#endif




   int iteration = 50;
   double best_cost = 9999999;

   /* 
    * Initializing the problem
    */
   TruckManyVisitsDump tp(infile);


   tp.process(2);

   STATS_PRINT("stats");
   assert(true==false);        

   DLOG(INFO) << "Initial solution: 0 is best";
   Solution best_sol(tp);
   best_cost = best_sol.getCostOsrm();

   for (int icase = 1; icase < 7; ++icase) {
       DLOG(INFO) << "initial solution: " << icase;
       tp.process(icase);
       if (best_cost > tp.getCostOsrm()) {
           DLOG(INFO) << "initial solution: " << icase << " is best";
           best_cost = tp.getCostOsrm();
           best_sol = tp;
       }
   }


   Optimizer optSol(best_sol, iteration);
   if (best_cost > optSol.getCostOsrm()) {
       best_cost = optSol.getCostOsrm();
       best_sol = optSol;
   }



#ifdef VRPMINTRACE
   best_sol.dumpCostValues();
   best_sol.tau();
#endif

   best_sol.dumpSolutionForPg();
   twc.cleanUp();

  } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      return 1;
  }

  return 0;
}




