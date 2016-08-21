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

#include "truckManyVisitsDump.h"
#include "fleetOpt.h"


void TruckManyVisitsDump::initializeTrip(Trip &trip, bool fromStart) {
  invariant();
  trip.evaluate();
#ifdef VRPMINTRACE
  if (!trip.feasable()) {
    trip.dumpeval("NOT FEASABLE in initializeTrip");
    trip.tau("SHOULD BE FEASABLE trip");
    assert(trip.feasable());
  }
#endif
  assert(trip.feasable());
  Trashnode bestNode;
  UINT bestPos;
  float8 bestTime;
  if (unassigned.size() == 0) return;
  switch (icase) {
    case 6:
      trip.findFastestNodeTo(false, unassigned, bestPos, bestNode, bestTime);
      safePushBackNode(trip, bestNode);
      insertBigSubPathAtEnd(trip);
      insertNodesOnPath(trip);
      break;
    case 5:
      insertBestPairInCleanTrip(trip);
      insertNodesOnPath(trip);
      fillTrip(trip);
      break;
    case 4:
      insertBestPairInCleanTrip(trip);
      insertNodesOnPath(trip);
      break;
    case 3:
      trip.findFastestNodeTo(true, unassigned, bestPos, bestNode, bestTime);
      safePushBackNode(trip, bestNode);
      insertBigSubPathAtEnd(trip);
      break;
    case 2:
      trip.findFastestNodeTo(true, unassigned, bestPos, bestNode, bestTime);
      safePushBackNode(trip, bestNode);
      insertBigSubPathAtBegin(trip);
      break;
    case 1:
      insertBestPairInCleanTrip(trip);
      insertBigSubPathAtBegin(trip);
      break;
    case 0:
      insertBestPairInCleanTrip(trip);
      break;
    default:
    ;
  }
  trip.getCostOsrm();
  remove_CV(trip);
  invariant();
}



bool TruckManyVisitsDump::insertBestPairInCleanTrip(Trip &trip) {
#ifdef VRPMINTRACE1
  DLOG(INFO) << "TruckManyVisitsDump::insertBestPairInCleanTrip";
#endif
  invariant();
  assert(trip.feasable());
  assert(trip.size() == 1);
  assert(unassigned.size() != 0);

  if (unassigned.size() == 1) {
    Trashnode onlyNode = unassigned[0];
    return safeInsertNode(trip, onlyNode, 1);
  };

  UINT bestFrom;
  UINT bestTo;
  Bucket subPath;
  // subpath includes bestFrom and bestTo
  trip.findPairNodesHasMoreNodesOnPath(assigned, unassigned, bestFrom, bestTo, subPath);
  assert(subPath.size() != 0);
#ifdef VRPMAXTRACE
  subPath.dumpid("BEST SUBPATH FOR INITIALIZING TRIP");
#endif
  safePushFrontSubpath(trip, subPath);

  trip.getCostOsrm();
#ifdef VRPMINTRACE1
  trip.tau("after inserting");
#endif
  invariant();
  return true;
}


bool TruckManyVisitsDump::insertBigSubPathAtBegin(Trip &trip) {
#ifdef VRPMINTRACE
  DLOG(INFO) << "TruckManyVisitsDump::insertBigSubPathAtBegin";
#endif
  invariant();
  if (unassigned.size() == 0) return false;
  UINT bestNode;
  Bucket subPath;
  twc->findBestFromNodeHasMoreNodesOnPath(assigned, unassigned, bestNode, trip[1].nid(), subPath);
  assert(subPath.size() != 0);
#ifdef VRPMAXTRACE
  trip.tau();
  subPath.dumpid("subpath at before");
#endif

  invariant();
 if(subPath.size() == 1) return false;
  trip.insert(subPath, 1);
  assigned = assigned + subPath;
  unassigned = unassigned - subPath;

  trip.getCostOsrm();
#ifdef VRPMINTRACE
  trip.tau("after inserting");
#endif
  invariant();
  return true;
}

bool TruckManyVisitsDump::insertBigSubPathAtEnd(Trip &trip) {
#ifdef VRPMINTRACE1
  DLOG(INFO) << "TruckManyVisitsDump::insertBigSubPathAtEnd";
#endif
  invariant();
  if (unassigned.size() == 0) return false;
  UINT bestNode;
  Bucket subPath;
  twc->findBestToNodeHasMoreNodesOnPath(assigned, unassigned, trip[trip.size()-1].nid(), bestNode, subPath);
  assert(subPath.size() != 0);
#ifdef VRPMAXTRACE
  trip.tau();
  subPath.dumpid("subpath at after");
#endif
  invariant();
  if(subPath.size() == 1) return false;
  trip.insert(subPath, trip.size());
  assigned = assigned + subPath;
  unassigned = unassigned - subPath;

  trip.getCostOsrm();
#ifdef VRPMINTRACE1
  trip.tau("after inserting" );
#endif
  invariant();
  return true;
}


bool TruckManyVisitsDump::insertBestPairSubPath(std::deque<Trip> &trips) {
#ifdef VRPMINTRACE1
  DLOG(INFO) << "TruckManyVisitsDump::insertBestPairSubPath";
#endif
  invariant();
  float8 bestAvgNodeTime = 999999;
  float8 currAvgNodeTime = 999999;
  int lessFilledTrip = 0;
  float8 worseProportion = 1;

  for( UINT i = 0; i< trips.size(); ++i) {
    if (i == 0 || worseProportion > trips[i].size()/trips[i].estimatedZ()) {
      worseProportion = trips[i].size()/trips[i].estimatedZ();
      lessFilledTrip = i;
    }
  };

  // DLOG(INFO) << "lessFilledTrip" << lessFilledTrip;
  UINT from, to;
  Bucket subPath;
  assert(unassigned.size());
  twc->findPairNodesHasMoreNodesOnPath(assigned, unassigned, from, to, subPath);

  if (subPath.size() < 3) return false;
  float8 bestTime; 
  Trashnode bestNode;
  UINT bestPos;
  
  Bucket aux;  
  trips[lessFilledTrip].findFastestNodeTo(true, subPath, bestPos, bestNode, bestTime);
  for (UINT i = 0; i < subPath.size(); ++i) {
    invariant();
    aux.clear();
    aux.push_back(subPath[i]);
    trips[lessFilledTrip].findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);
    safeInsertNode(trips[lessFilledTrip], bestNode, bestPos);
    // DLOG(INFO) << "bestNode" << bestNode.id();
    // DLOG(INFO) << "at pos " << bestPos << " after " << trips[lessFilledTrip][bestPos-1].id();
    insertNodesOnPath(trips[lessFilledTrip]);
  }

  invariant();
  return true;
}

void TruckManyVisitsDump::fillTrip(Trip &trip) {
#ifdef VRPMINTRACE1
  DLOG(INFO) << "TruckManyVisitsDump::fillTrip";
#endif
  invariant();
  UINT oldSize = trip.size();;
  trip.getCostOsrm();
  remove_CV(trip);
#ifdef VRPMINTRACE
  if (!trip.feasable()) {
     trip.tau("NOT FEASABLE in fillTrip");
     return;
     assert(trip.feasable());
  }
#endif
  assert(trip.feasable());
  if (unassigned.size() == 0) return;
  assert(unassigned.size() != 0);
  Trashnode bestNode;
  UINT bestPos;
  Bucket subPath;
  trip.getCostOsrm();
#ifdef VRPMINTRACE
  DLOG(INFO) << "trip needs " << trip.getz1() << " nodes";
  trip.tau("before inserting nodes in path");
  // if (trip.getz1()>10) assert(true==false);
#endif
  float8 bestTime, currTime;

  
  Bucket aux;
  // bestNode = unassigned[0];
  // instead of this:
  //trip.findFastestNodeTo(true, unassigned, bestPos, bestNode, bestTime);
  // use twc->getTimeOnTrip(from, middle, to)
  while (!trip.has_cv()) {
    bestTime = 99999;
    oldSize = trip.size();
    if (unassigned.size() == 0) break;
    for (UINT i = 0; i < trip.size(); i++) {
      for (UINT j = 0; j < unassigned.size(); j++) {
        if (i < trip.size() - 1) {
          currTime = twc->getTimeOnTrip(trip[i], unassigned[j], trip[i+1]);
        } else {
          currTime = twc->getTimeOnTrip(trip[i], unassigned[j], trip.getDumpSite());
        }
        if (currTime < bestTime) {
          bestTime = currTime;
          bestPos = i + 1;
          bestNode = unassigned[j];
        }
      }
    }
//    aux.clear();
//    aux.push_back(bestNode);
//    trip.findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);
//    DLOG(INFO) << "inside while bestNode" << bestNode.id();
//    DLOG(INFO) << "at pos " << bestPos << " after " << trip[bestPos-1].id();
    safeInsertNode(trip, bestNode, bestPos);
    insertNodesOnPath(trip);
    trip.evaluate();
    invariant();
 
#ifdef VRPMAXTRACE
    trip.tau("after inserting nodes in path");
    DLOG(INFO)  << "oldSize" << oldSize << "  newsize " << trip.size();
    DLOG(INFO) << "assigned " << assigned.size();
    DLOG(INFO) << "unassigned " << unassigned.size();
 #endif
  } // while

  assert(oldSize <= trip.size());
  remove_CV(trip);
  invariant();
  return;

  trip.evaluate();
  if (trip.feasable()) {
    fillTrip(trip); // recursive until its unfeasable
    trip.evaluate();
  }
  trip.evaluate();
  if (trip.feasable()) {
    return; // we ran out of nodes
  }

  trip.evaluate();
#ifdef VRPMINTRACE
  if (trip.feasable()) {
     trip.dumpeval("SHOULD BE INFEASABLE in fillTrip");
     assert(!trip.feasable());
  }
#endif
  assert(!trip.feasable());
  remove_CV(trip);

  if (trip.has_twv()) {
    // need to delete nodes until it doesnt have cv
    trip.dumpeval();
    assert(true==false);
  }
  assert(!trip.has_cv());
  assert(!trip.has_twv());
  assert(trip.feasable());
}

void TruckManyVisitsDump::remove_CV(Trip &trip) {
#ifdef VRPMINTRACE
  DLOG(INFO) << "TruckManyVisitsDump::remove_CV";
#endif
  invariant();
  trip.getCostOsrm();
  if (trip.has_cv()) {
#ifdef VRPMINTRACE1
    trip.tau("trip with cv");
#endif
    // need to delete nodes until it doesnt have cv
    trip.evaluate();
    while(trip.has_cv()) {
      // TODO use the safe
      safePopBackNode(trip);
      trip.getCostOsrm();
    }
#ifdef VRPMINTRACE1
  trip.tau("trip with no cv");
#endif
  }
  assert(!trip.has_cv());
  invariant();
}


void TruckManyVisitsDump::remove_TWV(Trip &trip) {
  invariant();
  trip.getCostOsrm();
  if (trip.has_twv()) {
#ifdef VRPMINTRACE1
    DLOG(INFO) << "has twv";
    trip.tau("trip with twv");
#endif
    // need to delete nodes until it doesnt have twv
    trip.evaluate();
    while(trip.has_twv()) {
      safePopBackNode(trip);
      trip.getCostOsrm();
    }
  }
#ifdef VRPMINTRACE1
  trip.tau("trip with no twv");
  if (trip.has_twv()) trip.dumpeval();
#endif
  assert(!trip.has_twv());
  invariant();
}

/*
ManyVisitsDump::remove_TWV_fromTruck(Vehicle &trip) {
  invariant();
  trip.getCostOsrm();
  if (trip.has_twv()) {
#ifdef VRPMAXTRACE
    DLOG(INFO) << "has twv";
    trip.tau("trip with twv");
#endif
    // need to delete nodes until it doesnt have twv
    trip.evaluate();
    while(trip.has_twv()) {
      unassigned.push_back(trip[trip.size()-1]);
      assigned.erase(trip[trip.size()-1]);
      trip.e_remove(trip.size()-1);
      trip.evaluate();
    }
  }
  trip.getCostOsrm();
  assert(!trip.has_twv());
#ifdef VRPMMAXTRACE
  trip.tau("trip with no twv");
#endif
  invariant();
}
*/

void TruckManyVisitsDump::initializeTruck(Vehicle &truck, std::deque<Trip> &trips) {
  invariant();
  assert(unassigned.size() != 0);
  truck.getCostOsrm();
#ifdef VRPMINTRACE1
  DLOG(INFO) << "Estimated number of trips" << truck.estimatedN();
#endif
  Trip trip;
  trips.clear();
  for (int i = 0; i < truck.estimatedN(); i++) {
    trip = truck.get_new_trip();
    trip.setInitialValues(truck.avgC(), pickups);
    if (i == 0) {
      initializeTrip(trip, true);
    } else {
      trip.clear();
      trips[i-1].getDumpSite().dump();
      trip.set_startingSite(trips[i-1].getDumpSite()); 
      initializeTrip(trip, false);
    }
    
    if ( i < truck.estimatedN() - 1 ) {
      trip.set_endingSite(trip.getDumpSite());
      trip.getCostOsrm();
      trips.push_back(trip);
    } else {
        trips.push_back(trip);
    }
  }

  invariant();
}

void TruckManyVisitsDump::deleteNodesOfTrip(Trip &trip, int cant) {
  // remove last trip to give time to trips to work
  invariant();
  for (int i = 0; i < cant; i++) {
    safePopBackNode(trip);
  }
  invariant();
  return;
}


void TruckManyVisitsDump::fillTruck(Vehicle &truck, std::deque<Trip> &trips) {
  invariant();
  assert(unassigned.size() != 0);
  initializeTruck(truck, trips);
#ifdef VRPMINTRACE1
  for (const auto &trip: trips) {
    trip.tau("Initialization");
  }
#endif

  for (auto &trip: trips) {
    insertNodesOnPath(trip);
    remove_CV(trip);
  }

#ifdef VRPMINTRACE1
  for (const auto &trip: trips) {
    trip.tau("After inserting nodes on trip");
  }
#endif


  if (unassigned.size() > 0) {
    for (UINT i = 0; i < trips.size(); ++i) {
#ifdef VRPMINTRACE1
      DLOG(INFO) << "/n/n/n $$$$$$$$$$$$$$$$$$$$$  Filling trip: " << i;
      trips[i].tau("before");
#endif
      if (i == trips.size() - 1 && trips[i].size() == 1) {
        trips.pop_back();
        trips[i-1].set_endingSite(truck.getEndingSite());
        continue;
      }

      if (!trips[i].feasable()){
         trips[i].intraTripOptimizationNoOsrm();
      }
      if (trips[i].getz1() > 0 && trips[i].feasable() && unassigned.size() > 0) {
        int oldz1;
        do {
          oldz1 = trips[i].getz1();
          fillTrip(trips[i]);
          trips[i].getCostOsrm();
          if (!trips[i].feasable()){
            trips[i].intraTripOptimizationNoOsrm();
          }
          // I am here without a CV but may have a TWV
          assert(!trips[i].has_cv());
        } while (trips[i].feasable() && unassigned.size() > 0 && trips[i].getz1() < oldz1); 
        trips[i].tau(" $$$$$$$$$$$$$   after");
        if (trips[i].getz1() > 0 && unassigned.size() == 0 && i < trips.size() - 1) {
          DLOG(INFO) << "need a node";
          trips[i].dumpCostValues();
          // its not full needs z1, no more nodes and its not the last trip
          trips[i].tau("the trip that needs more nodes");
          trips[trips.size()-1].tau("the trip where we are taking nodes");
          deleteNodesOfTrip(trips[trips.size()-1], trips[i].getz1());
          if ( trips[trips.size()-1].size() == 1) {
            DLOG(INFO) << "removing last trip";
            trips.pop_back();
          } 
          --i;
        }
      }
    }
  }

 #if 0 
  for (UINT i = 0; i < trips.size(); ++i) {
    trips[i].tau("Trip to be optimized");
    trips[i].intraTripOptimizationNoOsrm();
    trips[i].tau("Optimized trip");
    // trips[i].dumpCostValues();
  }
  #endif

  buildTruck(truck, trips);

  // optimize the truck goes here
  truck.basicOptimization();
  truck.dumpCostValues();
  if (truck.getz2() > 0 && unassigned.size() > 0) {
    DLOG(INFO) << "truck needs extra trip";
    add_extra_trip(truck);
  }
  truck.dumpCostValues();
  invariant();
}

void TruckManyVisitsDump::add_extra_trip(Vehicle &truck) {
  Trip trip(truck.get_new_trip());
  // DLOG(INFO) << "NEW trip";
  trip.evaluate();
  // trip.print_short_eval();
  // DLOG(INFO) << "CHECK WITH";
  // truck.dumpeval();
  trip.setInitialValues(truck.avgC(), pickups);
  initializeTrip(trip, false);
  // trip.dumpeval();
  // trip.dumpCostValues();
  
  int oldz1;
  do {
    oldz1 = trip.getz1();
    fillTrip(trip);
    trip.getCostOsrm();
    if (!trip.feasable()){
      trip.intraTripOptimizationNoOsrm();
    }
    // I am here without a CV but may have a TWV
    assert(!trip.has_cv());
  } while (trip.feasable() && unassigned.size() > 0 && trip.getz1() < oldz1); 
  trip.intraTripOptimizationNoOsrm();
  truck.add_trip(trip);

}



// based on the trips we build the truck
// if 
void TruckManyVisitsDump::buildTruck(Vehicle &truck, std::deque<Trip> &trips) {
  truck.e_clean();
  for (UINT i = 0; i < trips.size(); ++i) {
    trips[i].getCostOsrm();
  }
  // std::sort(trips.begin(), trips.end(), BaseVehicle::Comptrips());
  for (UINT i = 0; i < trips.size(); ++i) {
    truck.add_trip(trips[i]);
  }
  return;
#if 0
    for (UINT j = 1; j < trips[i].size(); ++j) {
      truck.push_back(trips[i][j]);
    }
    if (i != trips.size() - 1) truck.push_back(truck.getDumpSite());   
  }
  if (truck[truck.size()-1].isDump()) truck.Path().pop_back();
#endif
  truck.getCostOsrm();
  // truck.tau("before adjust dumps");
  truck.e_adjustDumpsToNoCV(1);
  truck.getCostOsrm();
  // truck.tau("before adjust adjust time window");
  truck.getCostOsrm();
  truck.dumpCostValues();
  // truck.tau();
  // assert(true==false);
  //remove_TWV(truck);
  truck.getCostOsrm();
  truck.tau("after adjust adjust time window");
  // once the truck is built need to reconstruct the trips
  // and adjust them to limit the time windows
  // so that on trip is after the other
  if (!truck.feasable()) { truck.dumpeval(); truck.tau();};
  assert(truck.feasable());
  invariant();
}

void TruckManyVisitsDump::fillFleet() {
  invariant();
  int i = 0;
  while (unassigned.size() > 0) {
    DLOG(INFO) << "-+-+-+-+-+-+-+-+-+-+-+-+-+-+- TRUCK " << i << " +-+-+-+-+-+-+-+-+-+-+-+-+-+-";
    Vehicle truck;
    std::deque<Trip> trips;
    truck = getTruck();
    trips.clear();
    fillTruck(truck, trips);
// assert(true==false);
#if 1
    fleet.push_back(truck);
#else
    for (UINT i = 0; i < trips.size(); ++i) {
      fleet.push_back(trips[i]);
    }
#endif
   ++i; 
  }
  invariant();
  assert(unassigned.size() == 0);
}

void TruckManyVisitsDump::insertNodesOnPath(Trip &trip) {
#ifdef VRPMINTRACE1
  DLOG(INFO) << "--> TruckManyVisitsDump::insertNodesOnPath";
#endif
  Bucket streetNodes, tmp;
  invariant();

  streetNodes.clear();
  twc->getNodesOnPath(trip.Path(), trip.getDumpSite(), unassigned, streetNodes);
  tmp = streetNodes;
  assert((tmp * assigned).size() == 0);
  Bucket aux;
  Trashnode bestNode;
  float8 bestTime;
  UINT bestPos;
    // insert the containers that are in the path
  int lastBestPos = 0;
  while (streetNodes.size() > 0) {
        invariant();
        aux.clear();
        aux.push_back(streetNodes[0]);
        streetNodes.erase(streetNodes[0]);
        trip.findFastestNodeTo(false, aux, bestPos, bestNode, bestTime);
#ifdef VRPMINTRACE1
        DLOG(INFO) << "trying inserting node in path: " << bestNode.id() << " at position " << bestPos;
#endif
        if (bestPos <= lastBestPos) bestPos++;
        trip.e_insert(bestNode, bestPos);
        lastBestPos = bestPos;
        // trip.tau("inserted on path");
        assigned.push_back(bestNode);
        unassigned.erase(bestNode);
        invariant();
  }
  invariant();
#ifdef VRPMAXTRACE
  DLOG(INFO) << "<-- TruckManyVisitsDump::insertNodesOnPath";
#endif
}



//    PROCESS
//
//    This implements a feasable solution

void TruckManyVisitsDump::process(int pcase)
{
  unusedTrucks = trucks;
  usedTrucks.clear();
  assigned.clear();
  unassigned = pickups;
  twc->initializeTravelTime();
  invariant();

  fleet.clear();
  icase = pcase;

  DLOG(INFO) << "Starting initial Solution Proccess\n";
  bool oldState = osrmi->getUse();
  osrmi->useOsrm(true);
  fillFleet();
  DLOG(INFO) << "OPTIMIZING\n";
  tau();
  Fleetopt opt_fleet;

  opt_fleet.insert(fleet);
  opt_fleet.optimize(10);
  fleet.clear();
  fleet = opt_fleet.get_opt_fleet();
  DLOG(INFO) << "OPTIMIZED\n";
  tau();
  //assert(true==false);
  invariant();
}
