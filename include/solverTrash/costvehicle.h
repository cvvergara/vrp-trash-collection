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
#ifndef COSTVEHICLE_H
#define COSTVEHICLE_H

#include <limits>
#include <vector>
#include <sstream>

#include "baseClasses/logger.h"
#include "baseClasses/basictypes.h"
#include "baseClasses/twpath.h"
#include "baseClasses/twc.h"
#include "baseClasses/twpath.h"
#include "baseClasses/move.h"
#include "nodes/trashnode.h"
#include "baseTrash/basevehicle.h"


class CostVehicle: public BaseVehicle {
public:


///////
const Trashnode& last() const;
double shiftLength() const;
double estimatedZ() const;
int estimatedN() const; //!< estimated number of trips
double arrivalEclosesLast(const Trashnode &last) const;
double serviceE() const;






  CostVehicle(const Trashnode &S, const Trashnode &D, const Trashnode &E, double maxcap)
    : BaseVehicle(S, D, E, maxcap),
      ttSC(0.0), ttDC(0.0), ttCD(0.0), ttDE(0.0), ttCC(0.0),
      realttSC(0.0), realttDC(0.0), realttCD(0.0), realttDE(0.0), realttCC(0.0),
      N(0), Nreal(0), minDumpVisits(0), maxDumpVisits(0), realDumpVisits(0),
      Z(0), z1(0), z2(0), realz1(0), realz2(0), n(0), z(0), Zmissing(0), lastn(0),
      totalTime(0.0), realTotalTime(0.0), lastRealTotalTime(0.0),
      forcedWaitTime(0.0), totalWaitTime(0.0), idleTime(0.0),
      realForcedWaitTime(0.0), realtotalWaitTime(0.0), realIdleTime(0.0),
      idleTimeSCDE(0.0), idleTimeSDCDE(0.0),
      realIdleTimeSCDE(0.0), realIdleTimeSDCDE(0.0),
      sumIdle(0.0), penalty(0.0),
      v_cost(0.0), workNotDonePerc(0.0) {
   DLOG(INFO)<< "CostVehicle constructor";}



  CostVehicle()
    : ttSC(0.0), ttDC(0.0), ttCD(0.0), ttDE(0.0), ttCC(0.0),
      realttSC(0.0), realttDC(0.0), realttCD(0.0), realttDE(0.0), realttCC(0.0),
      N(0), Nreal(0), minDumpVisits(0), maxDumpVisits(0), realDumpVisits(0),
      Z(0), z1(0), z2(0), realz1(0), realz2(0), n(0), z(0), Zmissing(0), lastn(0),
      totalTime(0.0), realTotalTime(0.0), lastRealTotalTime(0.0),
      forcedWaitTime(0.0), totalWaitTime(0.0), idleTime(0.0),
      realForcedWaitTime(0.0), realtotalWaitTime(0.0), realIdleTime(0.0),
      idleTimeSCDE(0.0), idleTimeSDCDE(0.0),
      realIdleTimeSCDE(0.0), realIdleTimeSDCDE(0.0),
      sumIdle(0.0), penalty(0.0),
      v_cost(0.0), workNotDonePerc(0.0) {
  };


  CostVehicle( std::string line, const TwBucket &otherlocs )
    : BaseVehicle( line, otherlocs ),
      ttSC(0.0), ttDC(0.0), ttCD(0.0), ttDE(0.0), ttCC(0.0),
      realttSC(0.0), realttDC(0.0), realttCD(0.0), realttDE(0.0), realttCC(0.0),
      N(0), Nreal(0), minDumpVisits(0), maxDumpVisits(0), realDumpVisits(0),
      Z(0), z1(0), z2(0), realz1(0), realz2(0), n(0), z(0), Zmissing(0), lastn(0),
      totalTime(0.0), realTotalTime(0.0), lastRealTotalTime(0.0),
      forcedWaitTime(0.0), totalWaitTime(0.0), idleTime(0.0),
      realForcedWaitTime(0.0), realtotalWaitTime(0.0), realIdleTime(0.0),
      idleTimeSCDE(0.0), idleTimeSDCDE(0.0),
      realIdleTimeSCDE(0.0), realIdleTimeSDCDE(0.0),
      sumIdle(0.0), penalty(0.0),
      v_cost(0.0), workNotDonePerc(0.0) {

  };

  CostVehicle( int _vid, int _start_id, int _dump_id, int _end_id,
               double _capacity, double _dumpservicetime, double _starttime,
               double _endtime, const TwBucket &otherlocs )
    : BaseVehicle( _vid, _start_id, _dump_id, _end_id,
                   _capacity, _dumpservicetime, _starttime,
                   _endtime, otherlocs ) {
    };


  inline int  realN() const { return ( path.dumpVisits() + 1 ) ;}
  inline double  totalServiceTime() {
      return ( path.totServiceTime() +  dumpSite.serviceTime() +
              endingSite.serviceTime() ) ;
  }

  Trashnode& avgC() {return C;}
  double getCost() const { return v_cost;};
  double getCost() {
      if (size() > 1) setCost(path.last());
      else setCost(C);

      return v_cost;
  };
  double getCostOsrm() {
      evaluateOsrm();
      if (size() > 1) setCost(path.last());
      else setCost(C);

      return v_cost;
  };

  int getz1() const  {return realz1;};
  int getz2() const {return realz2;};
  int getn() const {return n;};

  void setInitialValues( const Trashnode &node, const TwBucket &picks );
  void setCost(const Trashnode &last);
  double getDeltaCost( double deltaTravelTime, int deltan ) ;

#ifdef DOVRPLOG
  void dumpCostValues() const;
#endif
  //for cost function
private:
  Trashnode C;
  double ttSC = 0;
  double ttDC = 0;
  double ttCD = 0;
  double ttDE = 0;
  double ttCC = 0;
  double realttSC = 0;
  double realttDC = 0;
  double realttCD = 0;
  double realttDE = 0;
  double realttCC = 0;
  int N = 0;
  int Nreal = 0;
  int minDumpVisits = 0;
  int maxDumpVisits = 0;
  int realDumpVisits;
  int Z = 0;
  int z1 = 0;
  int z2 = 0;
  int realz1 = 0;
  int realz2 = 0;
  int n = 0;
  int z = 0;
  int Zmissing = 0;
  int lastn;
  double totalTime = 0;
  double realTotalTime = 0;
  double lastRealTotalTime;
  double forcedWaitTime = 0;
  double totalWaitTime = 0;
  double idleTime = 0;
  double realForcedWaitTime = 0;
  double realtotalWaitTime = 0;
  double realIdleTime = 0;
  double idleTimeSCDE = 0;
  double idleTimeSDCDE = 0;
  double realIdleTimeSCDE = 0;
  double realIdleTimeSDCDE = 0;
  double sumIdle = 0;
  double penalty = 0;
  double v_cost = 0;
  double workNotDonePerc = 0;
};


#endif

