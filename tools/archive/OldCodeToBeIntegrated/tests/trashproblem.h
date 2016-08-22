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
#ifndef TRASHPROBLEM_H
#define TRASHPROBLEM_H

#include <string>
#include <iostream>
#include <vector>

#include "twc.h"
#include "trashnode.h"
#include "prob_trash.h"
#include "vehicle.h"


enum Selector {
    ANY         =0,     // any
    UNASSIGNED  =1,     // must be unassigned
    CLUSTER1    =2,     // in nid's cluster1
    CLUSTER2    =4,     // in nid's cluster2
    LIMITDEMAND =8,     // with demand < demandLimit
    PICKUP      =16,    // must be pickup nodes
    DEPOT       =32,    // must be depot nodes
    DUMP        =64,    // must be dump nodes
    RATIO       =128    // only select nodes with RATIO<ratio
};

class TrashProblem : Prob_trash {
  private:
    std::vector<Vehicle> fleet;  //in solution
//    Twpath<Trashnode> datanodes;
//    TWC<Trashnode> twc;
    std::deque<int> depots;
    std::deque<int> dumps;
    std::deque<int> pickups;

    std::vector<int> unassigned;

    std::vector< std::vector<double> > dMatrix;

    double ratio; // ratio for select boarder nodes in assignmentSweep3


  public:
    // structors

    //TrashProblem(): Prob_trash() { ratio = 0.85; };
    TrashProblem(const std::string &infile): Prob_trash(infile) { ratio = 0.85; };

    // accessors
    double distance(int nq, int n2) const;

    bool filterNode(const Trashnode &tn, int i, int selector, int demandLimit);

    //// these should be const
    int findNearestNodeTo(int nid, int selector, int demandLimit);
    int findNearestNodeTo(Vehicle &v, int selector, int demandLimit, int &pos);

    // get solution
    std::string solutionAsText() const;
    std::vector<int> solutionAsVector() const;

    double getduration() const;
    double getcost() const;
    int getTWV() const;
    int getCV() const;
    double getRatio() const { return ratio; };

    int getVehicleCount() const {return fleet.size(); };
    Vehicle getVehicle(int i) const { return fleet[i]; };

    void dumpDmatrix() const;
    void dumpFleet() const;
    void dumpdataNodes() const;
    void dumpDepots() const;
    void dumpDumps() const;
    void dumpPickups() const;
    void dump() const;
    void dumpSummary() const;

    void plot( std::string file, std::string title, std::string font );
    void plot( std::string file, std::string title, std::string font, std::deque<int> highlight );

    // mutators
    void clearFleet() { fleet.clear(); };
    void loadproblem(std::string& file);
    void setNodeDistances(Trashnode& n);
    void setRatio(double r) { ratio = r; };

    void buildDistanceMatrix();

    // methods to build initial solution
    bool buildFleetFromSolution(std::vector<int> solution);
    bool findVehicleBestFit(int nid, int& vid, int& pos);
    void dumbConstruction();
    void nearestNeighbor();
    // void nearestInsertion();
    // void farthestInsertion();
    void assignmentSweep();
    void assignmentSweep2();
    void assignmentSweep3();

    // intra-route optimization routines
    void opt_2opt();
    void opt_3opt();
    void opt_or_opt();
    void optimize();

};

#endif
