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
#ifndef FEASABLESOL_H
#define FEASABLESOL_H

#include <string>
#include <iostream>
#include <vector>

#include "trashnode.h"
#include "prob_trash.h"
#include "solution.h"
#include "vehicle.h"



class FeasableSol : public Solution
{
private:
  typedef  TwBucket<Trashnode> Bucket;
  typedef  unsigned long int UID;


  std::deque<Vehicle> unusedTrucks;
  std::deque<Vehicle> usedTrucks;
  Bucket unassigned;
  Bucket problematic;
  Bucket assigned;


  int tmp;
public:

  FeasableSol( const std::string &infile ): Solution( infile ) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    tmp = 0;
    process();
  };

  FeasableSol( const Prob_trash &P ): Solution( P ) {
    unusedTrucks = trucks;
    unassigned = pickups;
    fleet.clear();
    tmp = 0;
    process();
  };


private:
  void stepOne( Vehicle &truck );
  Vehicle getTruck();
  void process();

};

#endif
