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


#include <iostream>
#include <sstream>
#include <deque>

#include "baseClasses/logger.h"
#include "baseClasses/stats.h"
#include "baseClasses/timer.h"

#include "baseClasses/twpath.h"
#include "nodes/trashnode.h"

#include "baseClasses/move.h"
#include "solverTrash/vehicle.h"
#include "baseTrash/basevehicle.h"



double CostVehicle::serviceE() const {
  return endingSite.serviceTime();
}
const Trashnode& CostVehicle::last() const {
  return path.size() > 1? path.last(): C;
}
double CostVehicle::shiftLength() const {return endTime - startTime; }

/*! \brief Estimated number of containers in each trip */
double CostVehicle::estimatedZ() const {return  floor(maxcapacity / C.demand()); }
int CostVehicle::estimatedN() const {return  N;}

/*! \brief Arrival time at endsite when departing from the average closing 
  time of the nodes*/
double CostVehicle::arrivalEclosesLast(const Trashnode &last) const {
   double value = last.closes() + last.serviceTime()
                  + twc.TravelTime(last, dumpSite) + dumpSite.serviceTime()
                  + twc.TravelTime(dumpSite, endingSite); 
   return value > endTime? endTime : value;
}

// for multiple dumps on path
void CostVehicle::setInitialValues( const Trashnode &node,
                                    const TwBucket &picks ) {

  C = node;
  ttSC = twc.getAverageTime(depot, picks);
  // assert(ttSC > 0);
  ttDC = twc.getAverageTime(dumpSite, picks);
  ttCD = twc.getAverageTime(picks, dumpSite);
  ttDE = twc.TravelTime(dumpSite.nid(), endingSite.nid());
  ttCC = twc.TravelTime(C.nid(), C.nid());

  //double serviceE = endingSite.serviceTime();
  //shiftLength = endTime - startTime;
  //e_makeFeasable( 0 );
  totalTime = 0;
  double fixedTime = depot.serviceTime() // begining of trip
                   + ttDE + serviceE();  // ending of trip 
  double variableTime = 0.0;
  totalTime = fixedTime + variableTime;
  N = 0;
  do {
    if (N == 0) { //first trip
      variableTime =
          C.opens() + ttSC
          + estimatedZ() * C.serviceTime() 
          + estimatedZ() * ttCC
          + ttCD + dumpSite.serviceTime();
    } else { // not first trip
      variableTime =
          ttDC   
          + estimatedZ() * C.serviceTime() 
          + estimatedZ() * ttCC
          + ttCD + dumpSite.serviceTime();
    }
    totalTime += variableTime;
#ifdef VRPMINTRACE
DLOG(INFO) << "estimated number of trips N " << N;
DLOG(INFO) << "totalTime " << totalTime;
DLOG(INFO) << "arrivalEcloseslst(C) " << arrivalEclosesLast(C);
#endif
    N++;
  } while (totalTime + C.departureTime() < arrivalEclosesLast(C) + serviceE());
  // N has now the estimated number of trips

  //estimated:
  // forcedWaitTime > 0: truck finishes duties before the shift ends
  // forcedWaitTime < 0: truck finishes duties after the shift ends
  // For this problem: serviceE == 0
  // From the point of view of the truck the endingSite closes at end of shift
  // therefore the expected value is positive otherwise a time violation exists
  // at the ending site
  forcedWaitTime = endTime - (arrivalEclosesLast(C)  +  serviceE());

  // the actual time at the ensSite that the truck remains idle
  totalWaitTime = endTime - ( endingSite.arrivalTime() + serviceE() );
  //because of moving dumps we calculate if that idle time can be:
  // used in the first trip
  idleTimeSCDE = C.closes() - ( depot.serviceTime() + ttSC );
  // truck has time to pickup z1 containers on the first (current) trip)
  z1 = idleTimeSCDE / ( C.serviceTime() + ttCC );
  // used in the second trip
  idleTimeSDCDE = C.closes() - ( dumpSite.departureTime() + ttDC );
  // truck has time to pickup z2 containers on the second (next) trip
  z2 = idleTimeSDCDE / ( C.serviceTime() + ttCC );
  // the accumulated waiting times + the time the truck is at endsite doing nothing
  //   minus the time its forced to wait because its imposible to do more work
  // because of time windows
  idleTime = endingSite.totWaitTime() + totalWaitTime - forcedWaitTime;
  // nomatter the containers, the travel time from dump to endsite is constant
  realttDE = ttDE;
}

void CostVehicle::setCost(const Trashnode &last) {
  realttSC = path.size() > 1 ? path[1].totTravelTime()  : ttSC;
  ttSC = std::min( realttSC, ttSC );

  realttCC = size() > 1 ? (path.getTotTravelTime() - realttSC) / (size() - 1) : ttCC;
  ttCC = std::min( realttCC, ttCC );

  realttCD = 0;
  realttDC = 0;

  if ( path.dumpVisits() != 0 ) {
    for (UINT i = 1; i < path.size() - 1; i++) {

      if ( path[i - 1].isDump() )
        realttCD += twc.TravelTime(path[i - 1], path[i]);

      if ( path[i].isDump() )
        realttDC += twc.TravelTime(path[i], path[i + 1]);
    }
  } else realttDC = ttDC; //without moving dumps this is 0

  // without moving dumps this is  twc.TravelTime(last, dumpSite)
  realttCD = (realttCD + twc.TravelTime(last, dumpSite)) / (path.dumpVisits() + 1.0);

  ttCD = std::min( realttCD, ttCD );
  ttDC = std::min( realttDC, ttDC );


  double realArrivalEclosesLast = arrivalEclosesLast(this->last());

  //>0 the latest the truck can arrive
  //arrivalEclosesLast = std::max( realArrivalEclosesLast, arrivalEclosesLast );

  realForcedWaitTime = endTime - (realArrivalEclosesLast  +  serviceE());
  forcedWaitTime = std::min (realForcedWaitTime , forcedWaitTime);

  n  = size() - 1 - (realN() - 1);
  //>0 allways good, we have one more container (truck point fo view)
  double deltan = n - lastn;
  //setting this n as the last
  lastn = n;

  if (Z == 0) Z = estimatedZ();
  z = (realN() == 1)?  n: n % Z;
  //>0 good, we can work more containers/trip
  //double deltaZ = Z - z;

  // ==0 we are in the limit of container pickup
  // >0 we need to pickup more containers
  Zmissing = Z - z > 0? Z - z: 0;

  //its never negative
  assert(Zmissing >= 0);

  realTotalTime = endingSite.arrivalTime() - path[0].departureTime();
  lastRealTotalTime = realTotalTime;

#ifdef DOVRPLOG

  if ( realArrivalEclosesLast < realTotalTime ) {
    last.dumpeval(maxcapacity);
    dumpCostValues();
  };

#endif

  //otherwise we are in a TWV and something is wrong on the calculation
  //assert ( realArrivalEclosesLast > realTotalTime );

  realIdleTime =  shiftLength() -  realTotalTime ;

  realIdleTimeSCDE =  ( Zmissing > 0 ) ?
                      ( C.serviceTime() + realttCC ) * Zmissing :
                      C.closes() - ( depot.departureTime() +  realttSC ) ;

  realz1 = std::min( ( int ) ( floor( realIdleTime /
                                      ( C.serviceTime() + realttCC ) ) ) , Zmissing ) ;

  //cant have negative idleTime
  realIdleTimeSDCDE = std::max( ( C.closes() -
                                  ( dumpSite.departureTime() + realttDC ) ) , 0.0 );

  realz2 = floor( realIdleTimeSDCDE / (C.serviceTime() +  realttCC) );

  sumIdle = realIdleTimeSCDE + realIdleTimeSDCDE + realIdleTime;

  //tengo z contenedores en el utimo viaje
  //me faltan Zmissing contenedores para un viaje lleno al dump
  //pero solo puedo hacer z1 contenedores mas en ese viaje al dump;


  // aumente el numero de contenedores  deltaz1>0 es bueno,
  // aumente contenedor y pudo puedo aumentar mas contenedores todavia
  // (no tiene sentido)
  if ( deltan >= 0 )
    z1 = std::max(z1 - 1, realz1);

  //el numero de contenedores no cambio  deltaz1>0 es bueno
  if ( deltan == 0 )
    z1 = std::max(z1, realz1) ;

  // quite un contenedor, deltaz>0 me hace falta un contenedor z1 debe
  // de haber aumentado minimo en 1
  if ( deltan < 0 ) z1 = std::max(z1 + 1, realz1);



  // aumente el numero de contenedores  deltaz2>0 es bueno,
  // aumente contenedor y pudo puedo aumentar mas contenedores todavia
  // (no tiene sentido)
  if ( deltan >= 0 )
    z2 = std::max ( z2 - 1, realz2 );

  // el numero de contenedores no cambio  deltaz2>0 es bueno
  if ( deltan == 0 )
    z2 = std::max ( z2, realz2 ) ;

  // quite un contenedor, deltaz>0 me hace falta un contenedor z1 debe
  // de haber aumentado minimo en 1
  if ( deltan < 0 )
    z2 = std::max ( z2 + 1, realz2 );

#ifdef VRPMAXTRACE
  // >0 el promedio de viaje entre contenedores es mayor
  double deltattCC = realttCC - ttCC;
  // >0 viaja mas lejos para llegar al primer contenedor
  double deltattSC = realttSC - ttSC;
  //>0 bad thing the forcedWaitTime has increased
  double deltaForcedWaitTime = realForcedWaitTime - forcedWaitTime;
  //>0 the latest the truck can arrive is better
  double deltaArrivalEclosesLast = realArrivalEclosesLast -
                                   arrivalEclosesLast(this->last());
  // >0 el viaje del dump al contenedor es mas largo que
  // lo esperado (worse)
  double deltattDC = realttDC - ttDC;
  // >0 el viaje del contenedor al dump es mar largo que lo esperado
  double deltattCD = realttCD - ttCD;
  double deltaz1 = realz1 - z1;
  // >0 the total time has increased  good or bad depends on deltan
  double deltaRealTotalTime = realTotalTime - lastRealTotalTime;
  double deltaz2 = realz2 - z2;
  DLOG( INFO ) << "TODOS LOS DELTAS2\n"
               << "deltattSC    " << deltattSC    << "\n"
               << "deltattCC    " << deltattCC    << "\n"
               << "deltattDC    " << deltattDC    << "\n"
               << "deltattCD    " << deltattCD    << "\n"
               << "deltaArrivalEclosesLast    " << deltaArrivalEclosesLast    << "\n"
               << "deltaForcedWaitTime    " << deltaForcedWaitTime    << "\n"
               << "deltan    " << deltan    << "\n"
               << "deltaRealTotalTime    " << deltaRealTotalTime    << "\n"
               << "deltaz1    " << deltaz1    << "\n"
               << "deltaz2    " << deltaz2    << "\n";
#endif


  v_cost = getDuration();
}


double CostVehicle::getDeltaCost( double deltaTravelTime, int deltan ) {
  double newrealTotalTime = realTotalTime + deltaTravelTime;
  double newrealIdleTime = arrivalEclosesLast(C) - arrivalEclosesLast(path.last());
  int newn = n + deltan;
  int newz = ( realN() == 1 ) ?  newn  : newn % Z ;
  int newZmissing = ( Z > newz ) ? Z - newz : 0;
  double newrealIdleTimeSCDE =  ( newz ) ? newrealIdleTime -
                                ( C.serviceTime() + realttCC ) * newZmissing :
                                C.closes() - ( depot.departureTime() +  realttSC );
  double newrealz1 = std::min ( ( int ) ( floor( newrealIdleTime /
                                          ( C.serviceTime() + realttCC ) ) ) , newZmissing ) ;
  double newrealIdleTimeSDCDE =  C.closes() - ( dumpSite.departureTime() +
                                 deltaTravelTime + realttDC );
  double  newrealz2 = newrealIdleTimeSDCDE / ( C.serviceTime() +  realttCC );

  double newv_cost = newrealTotalTime + ( newrealz1 + newrealz2 ) * newn +
                     newrealIdleTimeSCDE + newrealIdleTimeSDCDE;
  double deltacost = newv_cost - v_cost;
  return deltacost;
}




#ifdef DOVRPLOG
void CostVehicle::dumpCostValues() const
{
  DLOG( INFO ) << " +++++++++++++++++++++  	 TRUCK #<<" << vid
               << "      +++++++++++++++++++++";
  DLOG( INFO ) << " Average Container:";
  C.dump();
  DLOG( INFO ) << " ------  current path -------";
  tau();

  DLOG( INFO ) << " ------  truck time limits -------";
  DLOG( INFO ) << "Shift Starts\t" << startTime;
  DLOG( INFO ) << "Shift ends\t" << endTime;
  DLOG( INFO ) << "Shift length\t" << shiftLength();


  DLOG( INFO ) << "------Real  Values of current truck in the solution -------\n"
               << "                   realttSC=\t" << realttSC << "\n"
               << "                   realttCC=\t" << realttCC << "\n"
               << "                   realttCD=\t" << realttCD << "\n"
               << "                   realttDC=\t" << realttDC << "\n"
               << "                   realttDE=\t" << realttDE << "\n"
               << "                 service(E)=\t" << serviceE() << "\n"
               << "                maxcapacity=\t" << maxcapacity << "\n"
               << "                 C.demand()=\t" << C.demand() << "\n"
               << "            C.servicetime()=\t" << C.serviceTime()  << "\n"
               << "                 C.closes()=\t" << C.closes() << "\n"
               << "    path[size()-1].closes()=\t" << path[size() - 1].closes() << "\n"
               << "  dumpSite.getservicetime()=\t" << dumpSite.serviceTime()  << "\n"
               << "dumpSite.getDepartureTime()=\t" << dumpSite.departureTime() << "\n"
               << " (number of trips)    realN=\t" << realN()  << "\n"
               << "endingSite.getArrivalTime()=\t" << endingSite.arrivalTime()  << "\n"
               << "      depot.departureTime()=\t" << path[0].departureTime() << "\n"
               << "                       size=\t" << size() << "\n"

               << "Average # of containers per trip\n"
               << "                         Z =\t" << estimatedZ()
               << "\t= floor( maxcapacity/C.demand())"
               << "\t= floor( " << maxcapacity << "/" << C.demand() << ")\t"
               << "\n"

               << "                        z =\t" << z 
               << "\t=(realN()==1)? n  : n % Z\t"  
               << "\t=(" << realN() << "== 1) " << n << ": " << n % Z  
               << "\n"

               << "Number of containers that can still fit on last trip\n"
               << "                 Zmissing =\t" << Zmissing
               << "\t= Z - z"
               << "\t= " << Z << " - " << z
               << "\n"


               << "                     n =\t" << n  << "\t=size() - 1 - (realN() - 1)  \t" << n << "\n"


               << "                realz1 =\t" << realz1 <<
               "\t== min ( floor ( realIdleTimeSCDE / (C.getservicetime() + realttCC) ) , Zmissing )\t"
               << realz1 << "\n"
               << "                realz2 =\t" << realz2 <<
               "\t=idleTimeSDCDE / (C.getservicetime() + realttCC)\t"    << realz2 << "\n"

               << "realArrivalEclosesLast =\t" << arrivalEclosesLast(this->last()) <<
               "\t=path[size()-1].closes() + realttCD + dumpSite.getservicetime() + realttDE \t"
               << "\n"

               << "    realForcedWaitTime =\t" << realForcedWaitTime  <<
               "\t=shiftEnds -( realArrivalEclosesLast  +  serviceE() )\t" << realForcedWaitTime
               << "\n"

               << "         realTotalTime =\t" << realTotalTime  
               << "\t =endingSite.arrivalTime()  - path[0].departureTime()\t"
               << endingSite.arrivalTime() <<" - " << path[0].departureTime() 
               << "\n"

               << "          realIdleTime =\t" << realIdleTime 
               << "\t = shiftLength  -  realTotalTime\t"
               << shiftLength() << " - " << realTotalTime
               << "\n"

               << "      realIdleTimeSCDE =\t" << realIdleTimeSCDE  <<
               "\t=( Zmissing>0 )?  (C.getservicetime() + realttCC ) * Zmissing :\n"
               "\t\t(Zmissing==0? C.closes() - ( depot.getDepartureTime() +  realttSC):0) ;\t"
               << realIdleTimeSCDE << "\n"
               << "     realIdleTimeSDCDE =\t" << realIdleTimeSDCDE  <<
               "\t=C.closes() - ( dumpSite.getDepartureTime() + realttDC)\t" <<
               realIdleTimeSDCDE << "\n"

               << "                sumIdle=\t" << sumIdle <<
               "\t=sumIdle=realIdleTimeSCDE+realIdleTimeSDCDE+realIdleTime\t" << sumIdle <<
               "\n"

               << "        workNotDonePerc=\t" << workNotDonePerc <<
               "\t=(double (realz1 + realz2))  /(double (n + realz1+realz2))\n"
               << "     1+ workNotDonePerc=\t" << ( 1 + workNotDonePerc ) <<
               "\t=(double (realz1 + realz2))  /(double (n + realz1+realz2))\n"
               << "realTotalTime + sumIdle)=\t" << ( realTotalTime + sumIdle ) << "\n"
               //                    << "\n\n             v_cost=\t" << v_cost <<
               //                     "\t= (realTotalTime + sumIdle) *( 1 + workNotDonePerc)\n"
               ;


#if 0
      << "\n\n\n DELTA TIME SIMULATION\n"
      << "if a container is added into a very full truck:\n";

  for (double delta = -20; delta < 20; delta++) { //changes in time
    if (n) {
      DLOG(INFO) << "same amount of containers delta=" << delta <<
                 "\t    delta+delta/n=" << (penalty = delta / n) << "\t";
      DLOG(INFO) << "penalty*sumIdle= " << (penalty * sumIdle) << "\n";
    }

    if (n + 1) {
      DLOG(INFO) << "1 container more          delta=" << delta <<
                 "\tdelta+delta/(n+1)=" << (delta / (n + 1)) << "\t";
      DLOG(INFO) << "penalty*sumIdle= " << (penalty * sumIdle) << "\n";
    }

    if (n - 1) {
      DLOG(INFO) << "1 container less          delta =" << delta <<
                 "\tdelta+delta/(n-1)=" << (delta / (n - 1)) << "\t";
      DLOG(INFO) << "penalty*sumIdle= " << (penalty * sumIdle) << "\n";
    }
  }

#endif

#if 0
  DLOG(INFO) <<
             "\n\n\n ------estimated  Values for emtpy truck that is in the solution -------\n"
             << "ttSC=\t" << ttSC << "\n"
             << "ttCC=\t" << ttCC << "\n"
             << "ttCD=\t" << ttCD << "\n"
             << "ttDC=\t" << ttDC << "\n"
             << "ttDE=\t" << ttDE << "\n"
             << "service(E)\t" << serviceE() << "\n"
             << " Z = floor( maxcapacity/C.getdemand() )    <<--- this is still the estimation\n"
             << Z << " = floor( " << maxcapacity << "/" << C.getdemand() << " )\n"

             << " \narrivalEcloseslast = C.closes() + ttCD + dumpSite.getservicetime() + ttDE \n"
             << arrivalEcloseslast << " = " << C.closes() << " + " << ttCD << " + " <<
             dumpSite.getservicetime() << " + " << ttDE << "\n"

             << "\n forcedWaitTime = shiftEnds -( arrivalEcloseslast  +  serviceE() )\n"
             << forcedWaitTime << " = " << endTime << " - (" << arrivalEcloseslast << " + "
             << serviceE() << " )\n"

             << " \nwith N=1\n"
             << " upperLimit(totalTime) = depot.getservicetime()  + ttSC + ttDE + endingSite.getservicetime()\n"
             << "+ N * Z * C.getservicetime() + N * (Z - 1) * ttCC + (N -1) * ttDC\n"
             << "+ N * ( dumpSite.getservicetime() + ttCD )\n"
             << totalTime << " = " << depot.getservicetime() << " + " << ttSC << " + " <<
             ttDE << " + " << endingSite.getservicetime() << "\n"
             << " + " << N << " *" << Z << " * " << C.getservicetime() << " +" << N << " * ("
             << Z << " - 1) * " << ttCC << " + (" << N << " -1) *" << ttDC << "\n"
             << " + " << N << " * (" << dumpSite.getservicetime() << " +" << ttCD << " )" <<
             "\n"

             << " \n last (and only trip) can/cant serve Z containers? =  upperLimit(totalTime) <= arrivalEcloseslast (CAN) \n"
             << " last (and only trip) " << ( totalTime <= arrivalEcloseslast ? "CAN" :
                 "CAN NOT" ) << " serve <<" << Z << " containers  "
             << (totalTime <= arrivalEcloseslast) << "=" << totalTime << " <= " <<
             arrivalEcloseslast << "\n"

             << " \n idleTimeSCDE = C.closes() - ( depot.getDepartureTime() + ttSC )\n"
             << idleTimeSCDE << " = " << C.closes() << " - ( " << depot.getDepartureTime() <<
             " + " << ttSC << " )\n"

             << "\n z1 = idleTimeSCDE / (C.getservicetime() + ttCC)\n"
             << z1 << " = " << idleTimeSCDE << " / ( " << C.getservicetime() << " + " << ttCC
             << " )\n"
             << z1 << " containers can be served in a trip: SCDE\n"

             << " \n idleTimeSDCDE = C.closes() - ( dumpSite.getDepartureTime() + ttDC)\n"
             << idleTimeSDCDE << " = " << C.closes() << " - ( " <<
             dumpSite.getDepartureTime() << " + " << ttDC << " )\n"

             << "\n z2 = idleTimeSDCDE / (C.getservicetime() + ttCC)\n"
             << z2 << " = " << idleTimeSDCDE << " / ( " << C.getservicetime() << " + " <<
             ttCC << " )\n"
             << z2 << " containers can be served in a trip: SDCDE\n"




             ;
  DLOG(INFO) << "\n\n\n ------  DOCUMENT COST  VARIABLES -------" <<
             " ------  REAL COST  VARIABLES -------\t " << " ------  PERCENTAGES  -------\n"
             << "ttSC=\t" << ttSC << "\t" << "realttSC=\t"    << realttSC << "\t" <<
             "realttSC/ttSC=\t\t" << realttSC / ttSC * 100 << "%\n"
             << "ttCC=\t" << ttCC << "\t" << "realttCC=\t"    << realttCC << "\t" <<
             "realttCC/ttCC=\t\t" << realttCC / ttCC * 100 << "%\n"
             << "ttCD=\t" << ttCD << "\t" << "realttCD=\t"    << realttCD << "\t" <<
             "realttCD/ttCD=\t\t" << realttCD / ttCD * 100 << "%\n"
             << "ttDC=\t" << ttDC << "\t" << "realttDC=\t"    << realttDC << "\t" <<
             "realttDC/ttDC=\t\t" << realttDC / ttDC * 100 << "%\n"
             << "ttDE=\t" << ttDE << "\t" << "realttDE=\t"    << realttDE << "\t" <<
             "realttDE/ttDE=\t\t" << realttDE / ttDE * 100 << "%\n"
             << "service(E)\t" << serviceE() << "\t" << "service(E)\t" << serviceE << "\n"
             << "Z\t\t"   << Z << "\n"
             << "N\t\t"   << N << "\t" << "real N\t" << realN() << "\t" << "realN/N \t\t" <<
             realN() / N * 100 << "%\n"
             << "totalTime\t" << totalTime << "\t" << "realTotalTime\t" << realTotalTime <<
             "\t" << "realTotalTime/totalTime\t" << realTotalTime / totalTime * 100 << "%\n"
             << "\t\t\t\t\t\t\t" << "realTotalTime/shiftLength\t" << realTotalTime /
             shiftLength * 100 << "%\n"
             << "totalWaitTime\t" << totalWaitTime << "\t\t\t\ttotalWaitTime/totalTime\t" <<
             totalWaitTime / totalTime * 100 << "%\n"
             << "\t\t\t\t\t\t\t" << "totalWaitTime/shiftLength\t" << totalWaitTime /
             shiftLength * 100 << "%\n"

             << "forcedWaitTime\t" << forcedWaitTime << "\t\t\t\t" <<
             "forcedWaitTime/shiftLength\t" << forcedWaitTime / shiftLength * 100 << "%\n"
             << "idleTime\t" << idleTime << "\t" << "idleTime/shiftLength\t" << idleTime /
             shiftLength * 100 << "%\n"
             << "arrivalEcloseslast\t" << arrivalEcloseslast << "\n";
#endif
}
#endif
