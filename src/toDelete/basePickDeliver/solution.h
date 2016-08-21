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
#ifndef SOLUTION_H
#define SOLUTION_H

#include <deque>
#include <cmath>

#include "prob_pd.h"
#include "twpath.h"
#include "vehicle.h"
#include "plot.h"
#include "orders.h"

const double EPSILON = 0.001;

class Solution: public Prob_pd {
  protected:
typedef  Twpath<Dpnode> Bucket;

    std::deque<Vehicle> fleet;


    double totalDistance;
    double totalCost;
    double w1,w2,w3;

  public:

    Solution(const Prob_pd& P):Prob_pd(P){}; 


    void setweights(double _w1,double _w2,double _w3) {w1=_w1;w2=_w2;w3=_w3;};
    void dump();
    void dumproutes();
    void tau() ;
    void plot(std::string file,std::string title);

    void computeCosts();
    double getCost();
    double getDistance();
    double getAverageRouteDurationLength();

    Solution& operator=( const Solution& rhs ) {
        if ( this != &rhs ) {
            totalDistance = rhs.totalDistance;
            totalCost = rhs.totalCost;
            fleet = rhs.fleet;
        }
        return *this;
    };

    bool operator == (Solution &another) const {
        return fleet.size() == another.fleet.size() &&
               std::abs(totalCost - another.totalCost) < EPSILON;
    };

   bool solutionEquivalent (Solution &another)  {
        computeCosts();
        another.computeCosts();
        return fleet.size() == another.fleet.size() &&
               std::abs(totalCost - another.totalCost) < EPSILON;

    };

    bool operator <  (Solution &another) const {
        return fleet.size() < another.fleet.size() || totalCost < another.totalCost;

    

    };
};

#endif

