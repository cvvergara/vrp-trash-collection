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
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <math.h>

#include "baseClasses/logger.h"
#include "baseClasses/stats.h"
#include "baseTrash/prob_trash.h"




Prob_trash::Prob_trash(const char *infile) {
    DLOG(INFO) << "---- char * Constructor --------------";
    std::string file = infile;
    loadProblem(file);
}

Prob_trash::Prob_trash(const std::string &infile) {
    DLOG(INFO) << "Prob_trash---- string Constructor --------------";
    loadProblem(infile);
}


void Prob_trash::loadProblem(const std::string &infile)
{
    datafile = infile;
    Bucket nodes;
    Bucket intersection;
    DLOG(INFO) << "Prob_trash LoadProblem -------------- "
        << datafile;


    twc->emptiedTruck = false;

    load_pickups(datafile + ".containers.txt");
    load_otherlocs(datafile + ".otherlocs.txt");

  // Mmmmmmhhhhhh
  intersection = otherlocs * pickups;
  invalid = invalid + intersection;
  pickups = pickups - intersection;
  nodes = nodes - intersection;

    if (!invalid.empty()) {
        invalid.dump("invalid");
        DLOG(WARNING) << "************** Redefining problem ********************";
        DLOG(INFO) << "************** Redefining problem ********************";
        pickups = pickups - intersection;
        nodes = nodes - intersection;
    }

    nodes = pickups + otherlocs;
    nodes.push_back(C);

    UINT i(0);
    for (auto &n : nodes.path) {
        n.set_nid(i);
        auto id = n.id();

        if (pickups.hasId(id)) {
            pickups[pickups.posFromId(id)].set_nid(i);
        } else if (otherlocs.hasId(id)) {
            otherlocs[otherlocs.posFromId(id)].set_nid(i);
        }
        ++i;
    };

    C = nodes.back();

    assert(pickups.size());
    assert(otherlocs.size());

    datanodes = nodes;


    twc->loadAndProcess_travelTimes(
            datafile + ".dmatrix-time.txt",
            datanodes,
            invalid);

    load_trucks(datafile + ".vehicles.txt");
    

    twc->setHints(dumps);
    twc->setHints(nodes);
    twc->setHints(depots);
    twc->setHints(pickups);
    twc->setHints(endings);

    // twc->fill_travel_time_onTrip();
    twc->settCC(C, pickups);


    assert(!trucks.empty() && !depots.empty() && !dumps.empty() && !endings.empty());


    for (UINT i = 0; i < trucks.size(); i++) {
        trucks[i].setInitialValues(C, pickups);
    }

#ifdef VRPMAXTRACE
    C.dump();
    dumps.dump(" ---------------------- dumps ---------------------- ");
    depots.dump(" ---------------------- depots ---------------------- ");
    pickups.dump(" ---------------------- pickups ---------------------- ");
    endings.dump("------------- endings --------------------- ");
    datanodes.dump("------------- ALL data nodes --------------");
    DLOG(INFO) << "TRUCKS";
    for (int i = 0; i < trucks.size(); i++) trucks[i].tau();

#ifdef VRPMINTRACE
    DLOG(INFO) << "INVALID TRUCKS";
#endif
    if (invalidTrucks.size()==0) DLOG(INFO) << " NONE\n";
    for (int i = 0; i < invalidTrucks.size(); i++) invalidTrucks[i].tau();


    // twc->dump();
#endif

    DLOG(INFO) << "-------- Leaving Prob_trash::LoadProblem --------------";
}



void Prob_trash::load_trucks(std::string infile) {
    assert (otherlocs.size());
    std::ifstream in(infile.c_str());
    std::string line;
    DLOG(INFO) << "Prob_trash:LoadTrucks" << infile;

    trucks.clear();

    while (getline( in, line)) {

        if (line[0] == '#') continue;

        Vehicle truck(line, otherlocs);

        if (truck.isvalid()) {
            trucks.push_back(truck);
            depots.push_back(truck.getStartingSite());
            dumps.push_back(truck.getDumpSite());
            endings.push_back(truck.getEndingSite());
        } else {
            invalidTrucks.push_back(truck);
        }
    }

    in.close();

}

void Prob_trash::setPhantomNodesForPickups()
{

}

#if 0
void Prob_trash::load_depots( std::string infile )
{
#ifdef VRPMINTRACE
  DLOG( INFO ) << "Prob_trash:Load_depots" << infile;
#endif
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;

  depots.clear();

  while ( getline( in, line ) ) {
    cnt++;

    if ( line[0] == '#' ) continue;

    Trashnode node( line );

    if ( not node.isValid() or not node.isDepot() ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "ERROR: line: " << cnt << ": " << line;
#endif
      invalid.push_back( node );
    } else {
      depots.push_back( node );
    }
  }
#endif
void Prob_trash::load_otherlocs(std::string infile)
{
    DLOG(INFO) << "Prob_trash:Load_otherlocs" << infile;
    std::ifstream in(infile.c_str());
    std::string line;
    int cnt = 0;

    otherlocs.clear();

    while (getline( in, line)) {
        cnt++;

        if (line[0] == '#') continue;

        Trashnode node(line);

        if (not node.isValid()) {
            DLOG(WARNING) << "ERROR: line: " << cnt << ": " << line;
            invalid.push_back(node);
        } else {
            otherlocs.push_back(node);
        }
    }

    in.close();
}

#if 0
void Prob_trash::load_dumps( std::string infile )   //1 dump problem
{
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;

  // Delete previous
  dumps.clear();
>>>>>>> origin/right-side-montevideo


void Prob_trash::load_pickups(std::string infile) {
    std::ifstream in(infile.c_str());
    std::string line;
    int cnt = 0;
    pickups.clear();
    double st, op, cl, dm, x, y;
    st = op = cl = dm = x = y = 0;

    while (getline(in, line)) {
        cnt++;

<<<<<<< HEAD
        if (line[0] == '#') continue;
=======
    if ( not node.isValid() or not node.isDump() ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "ERROR: line: " << cnt << ": " << line;
#endif
      invalid.push_back( node );
    } else {
      dumps.push_back( node );
    }
  }

  in.close();
}
#endif


void Prob_trash::load_pickups( std::string infile )
{
  std::ifstream in( infile.c_str() );
  std::string line;
  int cnt = 0;

  // Delete previous
  pickups.clear();

  double st, op, cl, dm, x, y;
  st = op = cl = dm = x = y = 0;

  while ( getline( in, line ) ) {
    cnt++;

    if ( line[0] == '#' ) continue;

        Trashnode node(line);
        node.set_type(Twnode::kPickup);

        if (not node.isValid()) {
            DLOG(WARNING) << "ERROR: line: " << cnt << ": " << line;
            invalid.push_back(node);
        } else {
            pickups.push_back(node);
            st += node.serviceTime();
            op += node.opens();
            cl += node.closes();
            dm += node.demand();
            x += node.x();
            y += node.y();
        }
    }

    in.close();
    st = st / pickups.size();
    op = op / pickups.size();
    cl = cl / pickups.size();
    dm = dm / pickups.size();
    x = x / pickups.size();
    y = y / pickups.size();
    C.set(-1, -1, x, y, dm, op, cl, st);
}

