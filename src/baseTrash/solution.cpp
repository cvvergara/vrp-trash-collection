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


#include "baseClasses/logger.h"
#include "baseClasses/pg_types_vrp.h"
#include "baseTrash/solution.h"

bool Solution::feasable() const
{
  assert( fleet.size() );

  for ( UINT i = 0; i < fleet.size(); i++ )
    if ( not fleet[i].feasable() ) return false;

  return true;
}

void Solution::evaluate()
{
  assert( fleet.size() );

  for ( UINT i = 0; i < fleet.size(); i++ )
    fleet[i].evaluate();

}


int Solution::countPickups() {
    assert( fleet.size() );

    int count = 0;
    for ( UINT i = 0; i < fleet.size(); i++ )
      count += fleet[i].countPickups();

    return count;
}


int Solution::v_computeCosts()
{
  totalCost = 0.0;
  totalDistance = 0.0;
  int removedPos = -1;

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    if ( fleet[i].size() == 1 ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "FOUND A TRUCK WITHOUT CONTAINERS";
#endif
      removedPos = i;
      trucks.push_back( fleet[i] );
      fleet.erase( fleet.begin() + i );
      break;
    };
  }

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    totalCost += fleet[i].getCost();
  }

  return removedPos;
}


double Solution::getCostOsrm() {
  totalCost = 0.0;

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    totalCost += fleet[i].getCostOsrm();
  }

  return totalCost;
}

int Solution::computeCosts()
{
  return v_computeCosts();
}

double Solution::getCost() const
{
  return totalCost;
}

double Solution::getDistance() const
{
  return totalDistance;
}
/*
void Solution::dumpSolutionForPg () const
{
  vehicle_path_t *results;
  UINT count;
  results = getSolutionForPg( count ) ;

  for (UINT i = 0; i < count; i++)
    std::cout << "i" << i <<
              "\tseq:" << results[i].seq <<
              "\tVID:" << results[i].vid <<
              "\tnid" << results[i].nid <<
              "\tntype" << results[i].ntype <<
              "\tdeltaTime" << results[i].deltatime <<
              "\tcargo" << results[i].cargo << "\n";

  for ( UINT i = 0; i < fleet.size(); ++i ) {
    if ( fleet[i].size() <= 1 ) continue;

    for ( UINT j = 0; j < fleet[i].size(); ++j ) {
      std::cout << "VID: " << fleet[i].getVid() <<
                "\tid: " << fleet[i][j].id() <<
                "\tntype: " << fleet[i][j].type() <<
                "\tDeparture: " << fleet[i][j].departureTime() <<
                "\tdeltaTime:" << fleet[i][j].deltaTime() <<
                "\tdeltaCargo" << fleet[i][j].demand() << "\n";
    }

    std::cout << "VID: " << fleet[i].getVid() << "\tid: " <<
              fleet[i].getDumpSite().id() <<
              "\tntype: " << fleet[i].getDumpSite().type() <<
              "\tDeparture: " << fleet[i].getDumpSite().departureTime() <<
              "\tdeltaTime:" <<  fleet[i].getDumpSite().deltaTime() <<
              "\tdeltaCargo" << fleet[i].getDumpSite().demand() << "\n";
    std::cout << "VID: " << fleet[i].getVid() << "\tid: " <<
              fleet[i].getEndingSite().id() <<
              "\ttype: " << fleet[i].getEndingSite().type() <<
              "\tDeparture: " << fleet[i].getEndingSite().departureTime() <<
              "\tdeltaTime:" <<  fleet[i].getEndingSite().deltaTime() <<
              "\tdeltaCargo" << fleet[i].getEndingSite().demand() << "\n";
  }

  free(results);
}*/

//Duplicated dumpSolutionForPg in order to change the output
void Solution::dumpSolutionForPg () const
{
  vehicle_path_t *results;
  UINT count;
  results = getSolutionForPg( count ) ;
/*
  std::cout <<"\tseq:" <<
            "\tVID:" <<
            "\tnid" <<
            "\tntype" <<
            "\tdeltaTime" <<
            "\tcargo" << std::endl;
  for (UINT i = 0; i < count; i++)
    std::cout << i <<
              "\t" << results[i].seq <<
              "\t" << results[i].vid <<
              "\t" << results[i].nid <<
              "\t" << results[i].ntype <<
              "\t" << results[i].deltatime <<
              "\t" << results[i].cargo << std::endl;
              */
  auto mapPhantomNodes = twc.getPhantomNodes();

  for ( UINT i = 0; i < fleet.size(); ++i ) {
    if ( fleet[i].size() <= 1 ) continue;
    int seq = 0;
    std::cout <<
              "seq" <<
              "\tVID" <<
              "\tx" <<
              "\ty" <<
              "\tid" <<
              "\tntype" <<
              "\tDepart" <<
              "\tdTime" <<
              "\tdCargo" << std::endl;
    for ( UINT j = 0; j < fleet[i].size(); ++j ) {
        seq++;
        auto it = mapPhantomNodes.find(fleet[i][j].id());
        if ( it!=mapPhantomNodes.end() ) {
#ifdef REWRITE
            /*
             * This section use phantomnodes
             *
             * TODO n is not defined
             *
             */

          Twnode before = Twnode(twc->IPNId(),it->second.reveNodeId(),it->second.beforePNode().x(),it->second.beforePNode().y());
          n.set_type( Twnode::kPhantomNode );
          Twnode after = Twnode(twc->IPNId(),it->second.forwNodeId(),it->second.afterPNode().x(),it->second.afterPNode().y());
          n.set_type( Twnode::kPhantomNode );

          //Print beforeNode
          std::cout <<
                     seq <<
                    "\t" << fleet[i].getVid() <<
                    "\t" << it->second.beforePNode().x() <<
                    "\t" << it->second.beforePNode().y() <<
                    "\t" << it->second.reveNodeId() <<
                    "\t" << TWC::mIPNId <<
                    "\t" << "" <<
                    "\t" << "" <<
                    "\t" << "" << std::endl;
          //Print Node
            std::cout <<
                       seq <<
                      "\t" << fleet[i].getVid() <<
                      "\t" << fleet[i][j].x() <<
                      "\t" << fleet[i][j].y() <<
                      "\t" << fleet[i][j].id() <<
                      "\t" << fleet[i][j].type() <<
                      "\t" << fleet[i][j].departureTime() <<
                      "\t" << fleet[i][j].deltaTime() <<
                      "\t" << fleet[i][j].demand() << std::endl;
            //Print afterNode
            std::cout <<
                       seq <<
                      "\t" << fleet[i].getVid() <<
                      "\t" << it->second.afterPNode().x() <<
                      "\t" << it->second.afterPNode().y() <<
                      "\t" << it->second.forwNodeId() <<
                      "\t" << TWC::mIPNId <<
                      "\t" << "" <<
                      "\t" << "" <<
                      "\t" << "" << std::endl;
#endif
        }
        else
        {
            //Print Node
              std::cout <<
                         seq <<
                        "\t" << fleet[i].getVid() <<
                        "\t" << fleet[i][j].x() <<
                        "\t" << fleet[i][j].y() <<
                        "\t" << fleet[i][j].id() <<
                        "\t" << fleet[i][j].type() <<
                        "\t" << fleet[i][j].departureTime() <<
                        "\t" << fleet[i][j].deltaTime() <<
                        "\t" << fleet[i][j].demand() << std::endl;
        }
    }

    seq++;
    std::cout <<
              seq  <<
              "\t" << fleet[i].getVid() <<
              "\t" << fleet[i].getDumpSite().id() <<
              "\t" << fleet[i].getDumpSite().type() <<
              "\t" << fleet[i].getDumpSite().departureTime() <<
              "\t" <<  fleet[i].getDumpSite().deltaTime() <<
              "\t" << fleet[i].getDumpSite().demand() << std::endl;
    seq++;
    std::cout <<
              seq  <<
              "\t" << fleet[i].getVid() <<
              "\t" << fleet[i].getEndingSite().id() <<
              "\t" << fleet[i].getEndingSite().type() <<
              "\t" << fleet[i].getEndingSite().departureTime() <<
              "\t" <<  fleet[i].getEndingSite().deltaTime() <<
              "\t" << fleet[i].getEndingSite().demand() << std::endl;


  }
  free(results);
}


vehicle_path_t *Solution::getSolutionForPg( UINT &count ) const
{
  count = 0;
  UINT fleetSize = fleet.size();
  //fleetSize=1;

  // count the number of records we need for the output
  for ( UINT i = 0; i < fleetSize; ++i )
    if ( fleet[i].size() > 1 )          // don't count empty routes
      count += fleet[i].size() + 2;   // add final dump and ending nodes

  // malloc memory to hold the output results
  vehicle_path_t *results;
  results = ( vehicle_path_t * ) malloc( count * sizeof( vehicle_path_t ) );

  if ( not results ) {
    count = -1;
    return NULL;
  }

  // remap internal node types to pg node types
  int map[] = {0, 1, 2, 3};  // uselsss now, because internally we are using the same numbers now

  UINT seq = 0;

  for ( UINT i = 0; i < fleetSize; ++i ) {
    if ( fleet[i].size() <= 1 ) continue;

    for ( UINT j = 0; j < fleet[i].size(); ++j ) {
      results[seq].seq       = seq + 1;
      results[seq].vid       = fleet[i].getVid();
      results[seq].nid       = fleet[i][j].id();
      results[seq].ntype     = map[fleet[i][j].type()];
      results[seq].deltatime     =  fleet[i][j].deltaTime();
      results[seq].cargo     =  fleet[i][j].demand();

      ++seq;
    }

    // add the final dump
    results[seq].seq       = seq + 1;
    results[seq].vid       = fleet[i].getVid();
    results[seq].nid       = fleet[i].getDumpSite().id();
    results[seq].ntype     = map[fleet[i].getDumpSite().type()];
    results[seq].deltatime = fleet[i].getDumpSite().departureTime() -
                             fleet[i][fleet[i].size() - 1].departureTime();
    results[seq].cargo     = fleet[i].getDumpSite().demand();
    ++seq;

    // add the ending location
    results[seq].seq       = seq + 1;
    results[seq].vid       = fleet[i].getVid();
    results[seq].nid       = fleet[i].getEndingSite().id();
    results[seq].ntype     = map[fleet[i].getEndingSite().type()];
    results[seq].deltatime = fleet[i].getEndingSite().departureTime() -
                             fleet[i].getDumpSite().departureTime();
    results[seq].cargo     = fleet[i].getEndingSite().demand();
    ++seq;
  }

#ifdef DOVRPLOG
  DLOG( INFO ) << "Solution::getSolutionForPg: seq: " << seq << ", count: " <<
               count;
  for (UINT i = 0; i < count; i++)
    DLOG( INFO ) << "i" << i <<
              "\tseq:" << results[i].seq <<
              "\tVID:" << results[i].vid <<
              "\tnid" << results[i].nid <<
              "\tntype" << results[i].ntype <<
              "\tdeltaTime" << results[i].deltatime <<
              "\tcargo" << results[i].cargo;
#endif
  return results;
}


// this is a list of the node ids representing a vehicle route and
// each vehicle is separated with a -1

std::string Solution::solutionAsText() const
{
  //DLOG(INFO) << "fleets size" << fleet.size();
  std::stringstream ss;;
  const std::vector<int> sol = solutionAsVector();

  for ( UINT i = 0; i < sol.size(); i++ ) {
    if ( i ) ss << ",";

    ss << sol[i];
  }

  //DLOG(INFO) << ss.str() <<"  Solution::solutionAsText();
  return ss.str();
}

std::string Solution::solutionAsTextID() const
{
  std::stringstream ss;;
  const std::vector<int> sol = solutionAsVectorID();

  for ( UINT i = 0; i < sol.size(); i++ ) {
    if ( i ) ss << ",";

    ss << sol[i];
  }

  return ss.str();
}


// create a vector of node ids representing a solution
// this can be used to save a compute solution while other changes
// are being tried on that solution and can be used with
// buildFleetFromSolution() to reconstruct the solution

std::vector<int>  Solution::solutionAsVectorID() const
{
  std::vector<int> sol;
  sol.push_back( -1 );

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    if ( fleet[i].size() == 0 ) continue;

    sol.push_back( fleet[i].getVid() );
    sol.push_back( -1 );

    for ( UINT j = 0; j < fleet[i].size(); j++ ) {
      sol.push_back( fleet[i][j].id() );
    }

    sol.push_back( fleet[i].getDumpSite().id() );
    sol.push_back( fleet[i].getDepot().id() );
    sol.push_back( -1 );
  }

  return sol;
}


std::vector<int>  Solution::solutionAsVector() const
{
  std::vector<int> sol;
  sol.push_back( -2 );

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    if ( fleet[i].size() == 0 ) continue;

    sol.push_back( fleet[i].getVid() );
    sol.push_back( -2 );

    for ( UINT j = 0; j < fleet[i].size(); j++ ) {
      sol.push_back( fleet[i][j].nid() );
    }

    sol.push_back( fleet[i].getDumpSite().nid() );
    sol.push_back( fleet[i].getDepot().nid() );
    sol.push_back( -2 );
  }

  return sol;
}

#ifdef DOPLOT
void Solution::plot( std::string file, std::string title )
{

  Plot<Trashnode> graph( datanodes );
  graph.setFile( file + ".png" );
  graph.setTitle( datafile + ": " + title );
  graph.drawInit();
  Prob_trash::plot( graph );

  for ( int i = 0; i < fleet.size(); i++ ) {
    fleet[i].plot( graph, i );
  }

  graph.save();

  // a grpah for individual truck but with all nodes

  for ( UINT j = 0; j < fleet.size(); j++ ) {
    Plot<Trashnode> graph1( datanodes );
    std::stringstream convert;
    convert << j;
    std::string carnum = convert.str();

    graph1.setFile( file + "car" + carnum + ".png" );
    graph1.setTitle( datafile + ": " + title + " car #" + carnum );
    graph1.drawInit();
    Prob_trash::plot( graph1 );
    fleet[j].plot( graph1, j );
    //graph1.drawPath(fleet[j].getpath(), graph1.makeColor(j*10), 1, false);
    graph1.save();
  }

  //     now a graph for each individual truck
  for ( UINT i = 0; i < fleet.size(); i++ ) {
    fleet[i].plot( file, datafile + ": " + title, i );
  }

}
#endif

#ifdef DOVRPLOG
void Solution::tau()
{
  DLOG( INFO ) << "Tau:";

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    fleet[i].tau();
  };

}

void Solution::dumproutes()
{
  DLOG( INFO ) << "Vehicle:";

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    DLOG( INFO ) << " -----> Vehicle#" << i;
    fleet[i].dump();
  }

  tau();
}
#endif

double Solution::getAverageRouteDurationLength()
{
  double len = 0.0;
  int n = 0;

  for ( UINT i = 0; i < fleet.size(); i++ ) {
    if ( fleet[i].size() > 0 ) {
      len += fleet[i].getDuration();
      n++;
    }
  }

  if ( n == 0 ) return 0;

  return len / n;
}


#if 0
Solution::Solution( const std::string &infile,
                    const std::string &solFile )
 : Prob_trash(infile) {
  std::vector< Twnode::NodeType > soltype;
  std::vector<int64_t> solid;
  std::ifstream in(solFile.c_str());
  std::string line;
  int type, id;

  if (!in)  {
    DLOG(WARNING) << "\nFailed to open file " << solFile << "\n";
    return;
  }

  while ( in >> type >> id )  {
         solid.push_back(id);
         soltype.push_back((Twnode::NodeType) type);
  }
  size_t nid, vid;
  Vehicle truck;
  Bucket unassigned = pickups;
  Bucket assigned;
#ifdef VRPMAXTRACE
    unassigned.dumpid("Unassigned");
    assigned.dumpid("Assigned");
#endif


  fleet.clear();
  Bucket solPath;
  Bucket stops;

  size_t i = 0;
  while (i < solid.size()) {
    // with both negatives we are done
    if ((solid[i] < 0) && soltype[i] < 0 ) break; 

    // with type negative its the truck id
    if ( soltype[i] < 0 ) { 
        //get the truck from the truks:
        for ( size_t tr = 0; tr < trucks.size(); tr++ ) {
            if ( trucks[tr].getVid() == vid ) {
                truck = trucks[tr];
                break;
            }
        }
        solPath.clear();
        ++i;
        continue;
    }

    // both numbers are positive
    while ( i < solid.size() && soltype[i] >= 0) {
      switch (soltype[i]) {

      case  Twnode::kStart:
      	    solPath.push_back( truck.getStartingSite() );
            break;

      case  Twnode::kPickup:
            nid = pickups.getNidFromId( solid[i] );
      	    solPath.push_back( datanodes[nid] );
      	    stops.push_back( datanodes[nid] );
            break;

      case Twnode::kDump:
      	    solPath.push_back( truck.getDumpSite() );
            break;

      case  Twnode::kEnd:
      	    solPath.push_back( truck.getEndingSite() );
#ifdef VRPMINTRACE
            solPath.dumpid( "solPath" );
#endif
            if ( truck.e_setPath( solPath ) ) {
                fleet.push_back( truck );
                assigned = assigned + stops;
                unassigned = unassigned - stops;
            };
            break;
      case  Twnode::kInvalid:
      case  Twnode::kDelivery:
      case  Twnode::kUnknown:
      case  Twnode::kLoad:
      case  Twnode::kPhantomNode:
            break;

      }  // switch
      ++i;
    }  // while     


  }  //while

  computeCosts();

  if (unassigned.size() ||  !(assigned == pickups)) {
#ifdef VRPMINTRACE
    DLOG( INFO ) << "Something went wrong creating the solution \n";
    unassigned.dumpid("Unassigned");
    assigned.dumpid("Assigned");
    pickups.dumpid("Pickups");
#endif
  }
}
#endif



Solution::Solution( const std::string &infile,
                    const std::vector<int> &sol ): Prob_trash( infile ) {

  int nid, vid;
  Vehicle truck;
  TwBucket unassigned = pickups;
  TwBucket assigned;
  bool idSol = true;

  if ( sol[0] == -2 ) idSol = false;

  fleet.clear();
  TwBucket solPath;

  UINT i = 1;

  while ( i < sol.size() ) {
    if ( sol[i] < 0 and sol[i + 1] > 0 ) break; //expected: vid -1

    vid = sol[i];

    //get the truck from the truks:
    for ( UINT tr = 0; tr < trucks.size(); tr++ )
      if ( trucks[tr].getVid() == vid ) {
        truck = trucks[tr];
        break;
      }

    i = i + 2;
    solPath.clear();

    while ( i<sol.size() and sol[i] >= 0 ) {

      if ( idSol ) nid = pickups.getNidFromId( sol[i] );
      else nid = sol[i];

      solPath.push_back( datanodes[nid] );
      i++;
    }

    solPath.dumpid( "solPath" );

    if ( truck.e_setPath( solPath ) ) {
      fleet.push_back( truck );
      assigned = assigned + solPath;
      unassigned = unassigned - solPath;
    }

    i++;
  };

  //computeCosts();
  evaluate();

#ifdef DOVRPLOG
  if ( unassigned.size() or ( assigned == pickups ) )
    DLOG( INFO ) << "Something went wrong creating the solution";

#endif
}





// code moved from OLD CODE TO BE INTEGRATED
double Solution::getduration() const
{
  double d = 0;

  for ( UINT i = 0; i < fleet.size(); i++ )
    d += fleet[i].getDuration();

  return d;
}


// get the total cost of the solution
// cost = w1 * getduration() + w2 * getTWV() + w3 * getCV()

double Solution::getcost() const
{
  double d = 0;

  for ( UINT i = 0; i < fleet.size(); i++ )
    d += fleet[i].getcost();

  return d;
}


// get the total number of TWV in the dolution

int Solution::twvTot() const
{
  int count = 0;

  for ( UINT i = 0; i < fleet.size(); i++ )
    count += fleet[i].twvTot();

  return count;
}


// get the total number of CV in the solution

int Solution::cvTot() const
{
  int count = 0;

  for ( UINT i = 0; i < fleet.size(); i++ )
    count += fleet[i].cvTot();

  return count;
}


// dump the problem and the solution
#ifdef DOVRPLOG
void Solution::dumpFleet() const
{
  DLOG( INFO ) << "--------- Fleet ------------";

  for ( UINT i = 0; i < fleet.size(); i++ )
    fleet[i].dump();
}


void Solution::dump() const {
  depots.dump("----------- depots -----------");
  dumps.dump("----------- dumps -----------");
  pickups.dump("----------- pickups -----------");
  dumpFleet();
  dumpSummary();
}
#endif

bool Solution::applyInsMove( const Move &move )
{
  assert( move.getmtype() == Move::Ins );

  fleet[ move.getInsFromTruck() ].applyMoveINSerasePart(
    move.getnid1(), move.getpos1() );

  fleet[ move.getInsToTruck() ].applyMoveINSinsertPart(
    datanodes[ move.getnid1() ], move.getpos2() );

  assert( fleet[ move.getInsFromTruck() ].feasable() );
  assert( fleet[ move.getInsToTruck() ].feasable() );
  return ( fleet[ move.getInsFromTruck() ].feasable()
           and  fleet[ move.getInsToTruck() ].feasable() );
}


// 2 vehicles involved
bool Solution::applyInterSwMove( const Move &move )
{
  assert( move.getmtype() == Move::InterSw );
  assert( not ( move.getInterSwTruck1() == move.getInterSwTruck2() ) );

  if ( not ( fleet[move.getInterSwTruck1()][ move.getpos1()].nid() ==
             move.getnid1() ) ) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "ERROR APPLYING INTERSW ";
    move.Dump();
#endif
    fleet[move.getInterSwTruck1()][ move.getpos1()].dump();
    fleet[move.getInterSwTruck2()][ move.getpos2()].dump();
  }

  assert( fleet[move.getInterSwTruck1()][ move.getpos1()].nid() ==
          move.getnid1() );
  assert( fleet[move.getInterSwTruck2()][ move.getpos2()].nid() ==
          move.getnid2() );

  fleet[move.getInterSwTruck1()].applyMoveInterSw( fleet[move.getInterSwTruck2()],
      move.getpos1(), move.getpos2() ) ;

  assert( fleet[ move.getInterSwTruck1() ].feasable() );
  assert( fleet[ move.getInterSwTruck2() ].feasable() );
  return ( fleet[ move.getInterSwTruck1() ].feasable()
           and fleet[ move.getInterSwTruck2() ].feasable() );
}

//1 vehichle involved
bool Solution::applyIntraSwMove( const Move &move )
{

  assert( move.getmtype() == Move::IntraSw );
  assert( fleet[move.getIntraSwTruck()][ move.getpos1()].nid()  ==
          move.getnid1() );
  assert( fleet[move.getIntraSwTruck()][ move.getpos2()].nid()  ==
          move.getnid2() );

  fleet[move.getIntraSwTruck()].applyMoveIntraSw(  move.getpos1(),
      move.getpos2() )  ;

  assert( fleet[ move.getIntraSwTruck() ].feasable() );
  return ( fleet[ move.getIntraSwTruck() ].feasable() ) ;
}

void Solution::dumpCostValues()
{
  for ( UINT i = 0; i < fleet.size(); i++ )
    fleet[i].getCost();

#ifdef DOVRPLOG
  for ( UINT i = 0; i < fleet.size(); i++ )
    fleet[i].dumpCostValues();
#endif
}

void Solution::setInitialValues()
{
#ifdef DOVRPLOG
  DLOG( INFO ) << "Average Container: \n" ;
  C.dump();
#endif
  for ( UINT i = 0; i < fleet.size(); i++ )
    fleet[i].setInitialValues( C, pickups );
}


// dump summary of the solution

#ifdef DOVRPLOG
void Solution::dumpSummary() const
{
  DLOG( INFO ) << "--------- Summary ------------";
  DLOG( INFO ) << "Total path length: " << getduration();
  DLOG( INFO ) << "Total path cost: " << getcost();
  DLOG( INFO ) << "Total count of TWV: " << twvTot();
  DLOG( INFO ) << "Total count of CV: " << cvTot();
  DLOG( INFO ) << "Solution: " << solutionAsText();
}
#endif


