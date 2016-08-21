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
#ifndef PROBLEM_H
#define PROBLEM_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>

#include "order.h"
#include "orders.h"
#include "bucketn.h"
#include "twpath.h"
#include "twc.h"

class Prob_pd {
  protected:
typedef  TwBucket<Dpnode> Bucket;

    Dpnode depot;

    TWC<Dpnode> twc;
    //Bucket datanodes;
    Twpath<Dpnode> datanodes;

    Orders ordersList;
    std::string datafile;


  public:
    int K;      // number of vehicles
    int Q;      // capacity

    Prob_pd(char *infile);
    Dpnode getdepot() const { return depot;};
    void loadProblem(char *infile);

    unsigned int getNodeCount() const;
    bool checkIntegrity() const;

    unsigned int getOrderCount() const;

    double distance(int n1, int n2) const;
    double DepotToPickup(int n1) const ;
    double DepotToDelivery(int n1) const ;
    int getOrderOid(int i) const;
    int getOrderPid(int i) const;
    int getOrderDid(int i) const;
    double nodeDemand(int i) const;
    double nodeServiceTime(int i) const;
    bool earlyArrival(int nid,double D) const; 
    bool lateArrival(int nid,double D) const; 
    bool isAsignedOrder(int oid) const;
    Dpnode& getDeliveryNodeFromOrder(int i);
    Dpnode& getPickupNodeFromOrder(int i);
    void sortNodeById();
    void sortNodeByDistReverse();
    void sortNodeByTWC();
    void sortOrdersbyDist();
    void sortOrdersbyId();
    void sortOrdersbyIdReverse();
    void sortOrdersbyDistReverse();


    void twcijDump() const;

    Order& getOrder(int i) ;

    void makeOrders();

    void nodesdump();
    void plot(Plot<Dpnode> &graph);
    void ordersdump( const std::deque<Order> orders ) const;
   void dump();


   inline double _MAX() { (std::numeric_limits<double>::max()); };
   inline double _MIN() { ( - std::numeric_limits<double>::max() ); };

    //void calcAvgTWLen();
};

#endif
