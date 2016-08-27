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

#include "solverTrash/fleetOpt.h"

void Optimizer::optimizefleet(int iter) {
    Fleetopt opt_fleet;
    opt_fleet.insert(fleet);
    opt_fleet.optimize(iter);
    DLOG(INFO) << "FINAL OPTIMIZATION\n";
    tau();
}


  
void Fleetopt::insert(const std::deque <Vehicle> &p_fleet) {
  fleet = p_fleet;
}

void Fleetopt::extract_trips() {
  for (UINT i = 0; i < fleet.size(); ++i) {
    for (const auto  &trip: fleet[i].trips) {
      ids.push_back(i);
      trips.push_back(trip);
    }
  }
}

void Fleetopt::optimize(int iter) {
  extract_trips();
  tauTrips("fleet::AFTER extract");

  intraTripOptimizationNoOsrm();
  tauTrips("fleet::AFTER intraOpt");

  auto count = 0;
  auto tot_count = 0;

  for (int i = 0; i < iter; ++i) {
    count = exchangesWorse(10);
    for (auto &trip : trips) {
      count = 0;
      for (auto &o_trip : trips) {
        count +=exchangesWithOnPath(o_trip, trip);
        count +=exchangesWithOnPath(trip, o_trip);
        count +=exchangesWithNotOnPath(o_trip, trip);
        count +=exchangesWithNotOnPath(trip, o_trip);
        tot_count += count;
      }
    }
    DLOG(INFO) << "fleet::exchanges Performed: " << count;
    if (count == 0) break;
    intraTripOptimizationNoOsrm();
    intraTripOptimizationNoOsrm();
    intraTripOptimizationNoOsrm();
  }
  intraTripOptimizationNoOsrm();
  intraTripOptimizationNoOsrm();
  intraTripOptimizationNoOsrm();
  DLOG(INFO) << "fleet::total exchanges Performed: " << count;

  tauTrips("fleet::AFTER intraTripOptimizationNoOsrm");
  reconstruct_fleet();
}


void Fleetopt::reconstruct_fleet() {

  for (Vehicle &truck: fleet) {
    Trashnode start = truck.getStartingSite();
    truck.clear();
    truck.trips.clear();
    truck.push_back(start);
  };

  DLOG(INFO) << fleet.size();
  for (UINT i = 0; i < trips.size(); ++i) {
    fleet[ids[i]].add_trip(trips[i]);;
    DLOG(INFO) << ids[i];
  }
}

std::deque<Vehicle> Fleetopt::get_opt_fleet() {
  return fleet;
}

