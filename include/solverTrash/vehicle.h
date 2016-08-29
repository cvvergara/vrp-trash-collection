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
#ifndef VEHICLE_H
#define VEHICLE_H

#include <limits>
#include <vector>
#include <sstream>

#include "baseClasses/logger.h"
#include "baseClasses/basictypes.h"
#include "baseClasses/twpath.h"
#include "nodes/trashnode.h"
#include "baseClasses/twc.h"
#include "baseClasses/move.h"
#include "solverTrash/costvehicle.h"

using namespace vrptc;
using namespace vrptc::nodes;


class Vehicle1: public CostVehicle {
protected:
  typedef std::set<Move, Move::compMove> Moves;

public:
  /*! @name constructors */
  ///@{
  Vehicle1():CostVehicle(){}
  Vehicle1(const Trashnode &S, const Trashnode &D, const Trashnode &E, double maxcap)
    : CostVehicle(S, D, E, maxcap) { 
   DLOG(INFO)<< "Vehicle1 constructor";}

  Vehicle1(const std::string &line, const TwBucket &otherlocs )
    : CostVehicle(line, otherlocs)   { }
  Vehicle1( int _vid, int _start_id, int _dump_id, int _end_id,
           int _capacity, int _dumpservicetime, int _starttime,
           int _endtime, const TwBucket &otherlocs )
    : CostVehicle( _vid, _start_id, _dump_id, _end_id,
                   _capacity, _dumpservicetime, _starttime,
                   _endtime, otherlocs ) {}
  ///@}

  /*! @name timePCN 
    Specialized timePCN because of the moving dumps, they use bucket::timePCN
  */
  ///@{
  double timePCN( POS from, POS middle, POS to ) const;
  double timePCN( POS from, Trashnode &middle ) const;
  ///@}

  ///@{
  // void intraTripOptimizationNoOsrm();

  ///}


  /*! @name evaluation of moves for optimization */
  ///@{
  long int eval_intraSwapMoveDumps( Moves &moves, POS  truckPos) const;
  long int eval_interSwapMoveDumps( Moves &moves, const Vehicle1 &otherTruck,
                                    POS  truckPos, POS  otherTruckPos,
                                    double factor   ) const;
  long int eval_interSwapMoveDumps( Moves &moves, const Vehicle1 &otherTruck,
                                    POS  truckPos, POS  otherTruckPos,
                                    POS fromPos, POS toPos   ) const;
  long int eval_insertMoveDumps( const Trashnode &node, Moves &moves,
                                 POS fromTruck, POS formPos, POS toTruck,
                                 double savings ) const;
  bool eval_erase( POS at, double &savings ) const;
  ///@}

  /*! @name  applying a move */
  ///@{
  bool applyMoveINSerasePart( UID nodeNid, POS pos );
  bool applyMoveINSinsertPart( const Trashnode &node, POS pos );
  bool applyMoveInterSw( Vehicle1 &otherTruck, POS truckPos, POS otherTruckPos );
  bool applyMoveIntraSw( POS fromPos, POS withPos );
  ///@}

  /*! @name  functions requiered because of moving dumps */
  ///@{
  bool e_makeFeasable( POS currentPos );
  bool e_insertIntoFeasableTruck( const Trashnode &node, POS pos );
  ///@}




};


#endif

