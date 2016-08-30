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

#include "initTrash/basicOperations.h"

void Basicoperations::invariant() {
#if 0
    if ((unassigned.size() + assigned.size()) != 109) {
    DLOG(INFO) << "assigned:" << assigned.size();
    assigned.dumpid();
    DLOG(INFO) << "unassigned" << unassigned.size();
    unassigned.dumpid();
    assert ((unassigned.size() + assigned.size()) == 109);
    }
#endif
    assert(pickups.size() == (unassigned.size() + problematic.size() + assigned.size()));
    assert(pickups == (unassigned + problematic + assigned));
    assert(!(unassigned * problematic).size());
    assert(!(unassigned * assigned).size());
    assert(!(problematic * assigned).size());
}

// inserts node if only if node is not being in assigned set
bool Basicoperations::safeInsertNode(Trip &trip, const Trashnode &node, UINT pos) {
  invariant();
  if (!unassigned.hasNode(node)) return false;
  assert(!assigned.hasNode(node));
  assert(!trip.Path().hasNode(node));

    trip.e_insert(node,pos);
    assigned.push_back(node);
    unassigned.erase(node);

  assert(!unassigned.hasNode(node));
  assert(assigned.hasNode(node));
  invariant();
  return true;
}

bool Basicoperations::safeDeleteNode(Trip &trip, UINT pos) {
  invariant();
  assert(pos < trip.size());
  Trashnode node = trip[pos];
  if (!assigned.hasNode(node)) return false;
  assert(!unassigned.hasNode(node));
  assert(trip.Path().hasNode(node));

    trip.e_remove(pos);
    unassigned.push_back(node);
    assigned.erase(node);

  assert(!assigned.hasNode(node));
  assert(unassigned.hasNode(node));
  assert(!trip.Path().hasNode(node));
  invariant();
  return true;
}

bool Basicoperations::safeInsertSubpath(Trip &trip, TwBucket &subPath, UINT pos){
  invariant();
  assert(pos != 0);
  assert(pos <= trip.size());
  assert(subPath.size() != 0);
  Trashnode node;
  trip.tau("trip");
  subPath.dumpid("subpath");
  assigned.dumpid("assigned");
  unassigned.dumpid("unassigned");
  
  for (UINT i = 0; i < subPath.size(); ++i) { 
    node = subPath[i];
    assert(!assigned.hasNode(node));
    assert(unassigned.hasNode(node));
  }

  trip.insert(subPath, pos);
  assigned = assigned + subPath;
  unassigned = unassigned - subPath;

  for (UINT i = 0; i < subPath.size(); ++i) { 
    node = subPath[i];
    assert(assigned.hasNode(node));
    assert(!unassigned.hasNode(node));
  }
  invariant();
  return true;
}

bool Basicoperations::safePushFrontSubpath(Trip &trip, TwBucket &subPath){
  invariant();
  return  safeInsertSubpath(trip, subPath, 1);
}

bool Basicoperations::safePushBackSubpath(Trip &trip, TwBucket &subPath){
  invariant();
  return  safeInsertSubpath(trip, subPath, trip.size());
}

bool Basicoperations::safePushBackNode(Trip &trip, Trashnode &node) {
  invariant();
    return safeInsertNode(trip, node, trip.size());
}

bool Basicoperations::safePushFrontNode(Trip &trip, Trashnode &node) {
  invariant();
    return safeInsertNode(trip, node, 1);
}


bool Basicoperations::safePopBackNode(Trip &trip) {
  invariant();
    if (trip.size() == 1) return false;
    return safeDeleteNode(trip, trip.size()-1);
}

bool Basicoperations::safePopFrontNode(Trip &trip) {
  invariant();
    if (trip.size() == 1) return false;
    return safeDeleteNode(trip, 1);
}


Vehicle  Basicoperations::getTruck() {
  assert(unusedTrucks.size() > 0);
  Vehicle  truck = unusedTrucks[0];
  truck.getCostOsrm();
  usedTrucks.push_back(truck);
  if (unusedTrucks.size() > 1) {
    unusedTrucks.erase(unusedTrucks.begin());
  }
  assert(unusedTrucks.size() > 0);
  return truck;
}


