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
#ifndef SRC_BASECLASSES_TWC_H_
#define SRC_BASECLASSES_TWC_H_
#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <algorithm>
#include <utility>
// #include <limits>
#include <iomanip>      // std::setprecision

#include "logger.h"
#include "stats.h"

#include "osrm_connection/osrmclient.h"


#include "baseClasses/basictypes.h"
#include "nodes/node.h"
#include "nodes/trashnode.h"
#include "nodes/phantomnode.h"
// #include "baseClasses/twbucket.h"
#include "baseClasses/twpath.h"
#include "baseClasses/singleton.h"
#include "baseClasses/pg_types_vrp.h"
#include "baseClasses/signalhandler.h"



/*! \class TWC
 * \brief Class TWC (Time Window Compatibility) provides tools for working with Twpath objects.
 *
 * There are two separate concepts that this class deals with. Compatibility
 * refers to Time Window Compatibility (TWC) and reachability that refers to
 * logical connectedness between nodes.
 *
 * \b Compatibility
 *
 * Time windows on node can be narrow or wide. There is a lot of analysis and
 * complexity and evaulating and manipulating node in a path when you need to
 * consider time windows. When all time windows are infinitely wide then there
 * are no time wondow constraints bacause any node can be place anywhere in
 * the path without creating a time window violation. Conversely, the narrower
 * the time windows the mode constrained the problem becomes.
 *
 * This class provides tools for manipulating and analysis of paths with
 * respect to issues of time window compatibility.
 *
 * Many of the ideas in this class are taken from the paper "A sequential
 * insertion heuristic for the initial solution to a constrained vehicle
 * routing problem" by JW Joubert and SJ Claasen, 2004.
 *
 * \b Reachability
 *
 * Reachability is a logical concept, for example think of a pickup and delivery
 * problem, it is logically in consistent to arrive at a delivery node before
 * you have made the related pickup for that delivery and visa versa. These
 * are determined by looking in a travel time matrix for plus infinity which
 * indicates there is no logical connection from source to destination.
 *
 * This also presumes that you have updated the travel time matrix and these
 * values appropriately if you plan to use the reachability functions.
 *
 */

// class TwBucket;

using namespace vrptc;
using namespace vrptc::nodes;

typedef std::map<UID,PhantomNode> PhantomNodes;   ///< Type definiton for PahntomNodes.


class TWC {
 private:

     /*! @brief  compares type id_time
      * 
      * From largest double to smallest double
      * tie brakes:
      *   from largest \b to to smallest \to
      * tie brakes:
      *   from largest \b from to smallest \from
      */
     template<typename Pair>
         class CompareSecond {
             public:
                 bool operator()(const Pair& firstPair, const Pair& secondPair) {
                     if (firstPair.second > secondPair.second) return true;
                     if (firstPair.second < secondPair.second) return false;
                     if (firstPair.first.second > secondPair.first.second) return true;
                     if (firstPair.first.second < secondPair.first.second) return false;
                     return firstPair.first.first > secondPair.first.first;
                     return false;
                 }
         };
     typedef std::pair< std::pair <UINT, UINT>,  double> id_time;
     typedef std::set<id_time,CompareSecond< id_time>> Ordering;
     // End CompareSecond class

     PhantomNodes mPhantomNodes;                       ///< PahntomNodes information for pickups. UID is the id of pickups nodes.
     UID mIPNId = 900000;                              ///< Id for PahntomNodes as Node
     UID mPNId = 900000;                               ///< Id for PahntomNodes as Node

     /*! \todo */
     class TTindex {  // all are ids when prev==from its a 3 node call
         UID prev_;
         UID from_;
         UID middle_;
         UID to_;
         public:
         UID prev() const { return prev_;}
         UID from() const { return from_;}
         UID middle() const { return middle_;}
         UID to() const { return to_;}
         TTindex()
             :prev_(0), from_(0), middle_(0), to_(0) {
             }
         TTindex(UID prev, UID from, UID middle, UID to)
             :prev_(prev), from_(from), middle_(middle), to_(to) {
             }
     };



 public:
     /**
      * @brief get phantom nodes for pickups sites
      *
      * This information is relevant in right side pickup trucks/containers
      *
      */
     PhantomNodes getPhantomNodes() const {
         return mPhantomNodes;
     }

     UID IPNId() const {return mIPNId;}
     UID PNId() const {return mPNId;}

 private:
     /**
      * @brief find and set phantom nodes for pickups sites
      *
      * This information is relevant in right side pickup trucks/containers
      *
      */
     void setPhantomNodes();


     //TODO change to deque?
     TwBucket original;                                 ///< The nodes. Set in loadAndProcess_distance and setNodes
     std::map< std::string, int> streetNames;                  ///< Map with street name and id
     mutable std::vector< std::vector<double> > twcij;
     mutable std::vector< std::vector<double> > travel_Time;   ///< Travel time matrix
     mutable std::vector< std::vector<double> > travel_time_onTrip;
     mutable std::vector< std::vector< std::deque< size_t > > > nodes_onTrip;  ///< The nodes on trip

     Ordering process_order;
     Ordering process_order_far;

 public:
     /*! \brief cleans all the tables, leaving them blank for a next execution */
     void cleanUp();


     bool emptiedTruck = false;
     int z1Tot = 0;
     int z2Tot = 0;

     /*
      * truckManyVisitsDump.cpp:674
      */
     void initializeTravelTime();

     void getProcessOrder();



     /*! @name findNearestNodeTo
       \brief Searches a bucket of unassigned nodes for one that is
       nearest to the existing path (truck) and is compatible with
       the time windows.

       \warning  size(unsassigned)>0
       \warning  compatability checks:
       path[pos] -> bestNode && bestNode -> path[pos]

       given a \b trucks path and a set of unassigned nodes
       finds the closest node to the path that is locally compatible
       at position pos.

       path's segment: (pos-1, \b pos)
       \b bestNode distance to path segment: \b besDist

       \param[in] truck The truck that we want to find the best node for.
       \param[in] unassigned The bucket of unassined nodes to look through.
       \param[out] pos The right position of the segment in the truck:
       \param[out] bestNode The best node from the unassigned bucket
       \param[out] bestDist has the best distance

       \return True if valid results where found
       */
     ///@{
#if 0 // def USE
 public:
     bool findNearestNodeTo(const TwBucket &truck,
             const  TwBucket &unassigned,
             POS &pos,
             Trashnode &bestNode,
             double &bestDist) const {
         STATS_INC("findNearestNodeTo");
         assert(unassigned.size());
         int flag = false;
         bestDist = VRP_MAX();   // dist to minimize
         pos = 0;        // position in path to insert
         double d;


         for ( size_t i = 0; i < unassigned.size(); i++ ) {
             for ( size_t j = 0; j < truck.size() - 1; j++ ) {
                 if ( isCompatibleIAJ(truck[j], unassigned[i], truck[j + 1]) ) {
                     d = truck.segmentDistanceToPoint(j , unassigned[i]);

                     if (d < bestDist) {
                         bestDist = d;
                         pos = j + 1;
                         bestNode = unassigned[i];
                         flag = true;
                     }
                 }
             }
         }
         return flag;
     }

 public:
     bool  findNearestNodeUseExistingData(const TwBucket &truck,
             const  TwBucket &unassigned,
             POS &pos,
             Trashnode &bestNode,
             double &bestDist) const {
         STATS_INC("findNearestNodeUseExistingData");
         assert(unassigned.size());
         int flag = false;
         bestDist = VRP_MAX();   // dist to minimize
         pos = 0;        // position in path to insert
         double d;


         for ( size_t i = 0; i < unassigned.size(); i++ ) {
             for ( size_t j = 0; j < truck.size() - 1; j++ ) {
                 if (!(j == 0)
                         && ((travel_Time[truck[j].nid()][unassigned[i].nid()] == -1)
                             || (travel_Time[unassigned[i].nid()][truck[j + 1].nid()] == -1)))
                     // all nodes from the depot that are missing should be calculated
                     continue;

                 if (isCompatibleIAJ(truck[j], unassigned[i], truck[j + 1])) {
                     d = truck.segmentDistanceToPoint(j , unassigned[i]);

                     if (d < bestDist) {
                         bestDist = d;
                         pos = j + 1;
                         bestNode = unassigned[i];
                         flag = true;
                     }
                 }
             }
         }

         if (!flag) return findNearestNodeTo(truck, unassigned, pos, bestNode
                 , bestDist);

         return flag;
     }
#endif  // USE


     // TODO
 public:
     void  findBestToNodeHasMoreNodesOnPath(
             const TwBucket &assigned,
             const TwBucket &unassigned,
             UINT From, UINT &bestTo, TwBucket &subPath) const;

 public:
     float8 getTimeOverNodesCount(
             const Trashnode &fromNode, const Trashnode &toNode,
             const TwBucket &assigned,
             const TwBucket &subPath) const;

 public:
     float8 getTimeOnTrip(const Trashnode from, const Trashnode middle, const Trashnode to);

 public:
     void  findBestFromNodeHasMoreNodesOnPath(
             const TwBucket &assigned,
             const TwBucket &unassigned,
             UINT &bestFrom, UINT To, TwBucket &subPath) const;


 public:
     bool  findPairNodesHasMoreNodesOnPath(
             const TwBucket &assigned, const TwBucket &unassigned,
             UINT &bestFrom, UINT &bestTo, TwBucket &subPath) const;






 public:
     bool  findNodeHasMoreNodesOnPath(const TwBucket &trip,
             const TwBucket &assigned, const TwBucket &unassigned,
             const Trashnode &dumpSite, UINT &bestNode, UINT &bestPos, TwBucket &subPath) const;

     size_t actualCantNodesOnTrip(UINT from, UINT to, const TwBucket &assigned) const;

     TwBucket actualNodesOnTrip(UINT from, UINT to, const TwBucket &assigned) const;

 public:
     void fill_travel_time_onTrip();

     void fill_travel_time_onTrip_work(Ordering &process_order);


 private:
     double
         fn_travel_time_onTrip(Trashnode from, Trashnode to) const;


 private:
     /*!
      * precondition: travel_time_onTrip.size() == original.size()
      */
     void process_pair_onPath(Trashnode from, Trashnode to) const;


 private:
     // the values for non containers to/from containers should be filled
     void compulsory_fill();




 private:
     void fill_times(const TwBucket nodesOnPath) const;


 public:

     /*!
       From the unassigned bucket all nodes that are in the truck's path
       will be placed on streetNodes Bucket

       The caller is resposible of the contents of orderedStreetNodes
       */
     TwBucket getNodesOnPath(
             const TwBucket &truck,
             const Trashnode &dumpSite,
             const TwBucket &unassigned) const;



     TwBucket getNodesOnPathInclusive(
             const Trashnode &from,
             const Trashnode &to,
             const TwBucket &unassigned
             ) const;




     /*!
      * truck: 0 1 2 3 4 5 6 D
      * call: 0 1 2 3 4 5 6 D       D 6 5 4 3 2 1 0
      * |_____________|       |_____________|
      * cycle 1               cycle 2
      * 
      * retrievable pairs
      * 0 1  1 2  .... 6 D
      * D 6  5 4  .... 1 0
      * 
      * Retrievable time triplets
      * 0 1 2  1 2 3  .....  5 6 D
      * 0 6 5  6 5 4  .....  2 1 D
      * 
      * Retrievable caudraplets
      * 0 1 2 3  1 2 3 4  ..... 5 6 D E
      * 0 6 5 4  6 5 4 3  ..... 2 1 D E
      * 
      * 
      * Special case:  When truck.size() == 1
      * truck: 0 D
      * call: 0 D  D 0
      * 
      * pairs: 0 D   D 0
      * triplets: NONE
      * quadruplets: NONE
      * 
      * Special case:
      * truck: 0 1 D
      * call: 0 1 D  D 1 0
      * 
      * pairs:
      * 0 1  1 D  D 1  1 0
      * triplets:
      * 0 1 D  D 1 0
      * quadruplets:
      * NONE
      * 
      */




     bool setTravelingTimesOfRoute(
             const TwBucket &truck,
             const Trashnode &dumpSite) const;




     /*!
truck: 0 1 2 3 4 5 6 D
call: 0 n 1   0 1 n 2  1 2 n 3   2 3 n 4   3 4 n 5   4 5 n 6   5 6 n D
|     |____________________________________________|    |
note: special                 cycle                           special
Retrievable time triplets
0 n 1  n 1 0         not: 1 0 1
0 1 n  1 n 2  n 2 1  not: 2 1 2
1 2 n  2 n 3  n 3 2
2 3 n  3 n 4  n 4 3
3 4 n  4 n 5  n 5 4
4 5 n  5 n 6  n 6 5
5 6 n  6 n D

Retrievable caudraplets
0 1 n 2  1 2 n 3   2 3 n 4   3 4 n 5   4 5 n 6   5 6 n D

Special case:  When truck.size() == 1
truck: 0 D
call: 0 n D
Uses TravelTime(0, n, D)

Special case:
truck: 0 1 D
call: 0 n 1  0 1 n D
triplets:
0 n 1  n 1 0  0 1 n  1 n D

*/






     bool setTravelingTimesInsertingOneNode(
             const TwBucket &truck,
             const Trashnode &dumpSite,
             const Trashnode &node) const;










     /*!
       .....  j j+1 .....
       .....  j pos j+1 ...

       pos = j+1
       min ( TT(j,node,j+1) )

       returns false when:
       no compatible node fits in the truck
       */
     bool findFastestNodeTo(
             bool first,
             const TwBucket &truck,
             TwBucket &unassigned,
             const Trashnode &dumpSite,
             POS &pos,
             Trashnode &bestNode,
             double &bestTime) const;



         ///@}

#if 0 // def USE
         /*!
          * \brief Select all nodes in a bucket from which you can not reach node id \b to.
          *
          * Given a bucket of nodes, return all the nodes from which you can not
          * reach a given node because of logical inconsistancies.
          *
          * For example, if a node is a delivery node then we can not reach the
          * pickup node associated with the delivery from the delivery node. IE:
          * you can never travel from the deliver node to the pickup node of the
          * same order.
          *
          * \param[in] nodes The source nodes from which we want to get to node \b to.
          * \param[in] to The target node id we are trying to get to.
          * \return A bucket of nodes from which you are unable to reach node to.
          */
         TwBucket getUnreachable(const TwBucket &nodes, UID to) const {
             assert(to < original.size());
             Bucket unreachable;

             for ( int i = 0; i < nodes.size(); i++ )
                 if (!isReachableIJ(nodes[i].nid(), to))
                     unreachable.push_back(nodes[i]);

             return unreachable;
         }

         /*!
          * \brief Select all nodes in a bucket that are not reachable from the given node.
          *
          * Given a bucket of nodes, return all the nodes that are not reachable
          * from the input node because of logical constraints.
          *
          * \param[in] from The source node id that we want to leave from to get to nodes in the bucket.
          * \param[in] nodes The bucket of nodes we want to get to.
          * \return A bucket of nodes that are not reachable from \b from
          */
         TwBucket getUnreachable(UID from, const TwBucket &nodes) const {
             assert(from < original.size());
             Bucket unreachable;

             for ( int i = 0; i < nodes.size(); i++ )
                 if ( !isReachableIJ(from, nodes[i].nid()) )
                     unreachable.push_back(nodes[i]);

             return unreachable;
         }

         /*! \brief Select from \b nodes from which node id \b to can be reached
          *
          * Given a bucket of nodes, return all the nodes from which we can reach
          * node is \b to without creating logical inconsistencies.
          *
          * \param[in] nodes The bucket of nodes we want to test if we can get to node id \b to
          * \param[in] to The target node id we want to get to.
          * \return A bucket of nodes from which we can get to node id \b to
          */
         TwBucket getReachable(const TwBucket &nodes, UID to) const {
             assert(to < original.size());
             Bucket reachable;

             for ( int i = 0; i < nodes.size(); i++ )
                 if ( isReachableIJ(nodes[i].nid(), to) )
                     reachable.push_back(nodes[i]);

             return reachable;
         }

         /*! \brief Select from \b nodes that can bre reached from node \b id
          *
          * Given a source node id and a bucket of nodes, return all the nodes in
          * the bucket that are directly reachable from node id \b from.
          *
          * \param[in] from The node id for the source node.
          * \param[in] nodes A bucket of target nodes.
          * \return A bucket of nodes that are reachable from node \b from
          */
         TwBucket getReachable(UID from, const TwBucket &nodes) const {
             assert(from < original.size());
             Bucket reachable;

             for (int i = 0; i < nodes.size(); i++ )
                 if (isReachableIJ(from, nodes[i].nid()) )
                     reachable.push_back(nodes[i]);

             return reachable;
         }

         /*
          * \brief Select nodes that are not compatible as predecessors to node id \b to.
          *
          * Given a bucket of nodes, return all nodes that can not be predecessors to
          * node id \b to because of time window incompatibility.
          *
          * \param[in] nodes A bucket of nodes to test for incompatiblity.
          * \param[in] to The node id that we want to get to.
          * \return A bucket of nodes that are incompatible as predecessors to node id \b to.
          */
         TwBucket getIncompatible(const TwBucket &nodes, UID to) const {
             assert(to < original.size());
             Bucket incompatible;

             for ( int i = 0; i < nodes.size(); i++ )
                 if ( !isCompatibleIJ(nodes[i].nid(), to) )
                     incompatible.push_back(nodes[i]);

             return incompatible;
         }

         /*!
          * \brief Select all nodes that are not compatible as successors to node id \b from.
          *
          * Given a bucket of nodes, return all that can not be successors to node
          * id \b from because of time window incompatibilities.
          *
          * \param[in] from The source node id.
          * \param[in] nodes A bucket of potential successor nodes.
          * \return A bucket of incompatible successor nodes.
          */
         TwBucket getIncompatible(UID from,
                 const TwBucket &nodes) const {
             assert(from < original.size());
             Bucket incompatible;

             for ( int i = 0; i < nodes.size(); i++ )
                 if ( !isCompatibleIJ(from, nodes[i].nid()) )
                     incompatible.push_back(nodes[i]);

             return incompatible;
         }

         /*!
          * \brief Select all nodes that are compatible as predecessor nodes to node id \b to.
          *
          * Given a bucket of nodes, return all can be predecessor nodes to node
          id \b to besaed on time window compatibility.
          *
          * \param[in] nodes A bucket of potential predecessor nodes.
          * \param[in] to The node id that we want to be a successor of nodes in the bucket.
          * \return A bucket of nodes that can be predecessor nodes to node id \b to.
          */
         TwBucket getCompatible(const TwBucket &nodes, UID to) const {
             assert(to < original.size());
             Bucket compatible;

             for ( int i = 0; i < nodes.size(); i++ )
                 if ( isCompatibleIJ( nodes[i].nid(), to ) )
                     compatible.push_back(nodes[i]);

             return compatible;
         }

         /*!
          * \brief Select all nodes in a bucket that are compatible as successor nodes to node id \b from.
          *
          * Given a bucket of nodes, return all nodes that are compatible as a
          * successor to node id \b from.
          *
          * \param[in] from The node id that that we are looking for successor to.
          * \param[in] nodes A bucket of potential successor nodes.
          * \return A bucket of compatible successors to node id \b from.
          */
         TwBucket getCompatible(UID from, const TwBucket &nodes) const {
             assert(from < original.size());
             Bucket compatible;

             for ( int i = 0; i < nodes.size(); i++ )
                 if (isCompatibleIJ(from, nodes[i].nid()))
                     compatible.push_back(nodes[i]);

             return compatible;
         }
#endif  // USE


         /*! \todo comments   */
 private:

#if 0 // def USE
         void
             setTravelTimeNonOsrm(UID from, UID to) const {
                 assert(from < original.size() && to < original.size());
                 double time;
                 time = original[from].distance(original[to]) / 250;

                 if ( !sameStreet(from, to) ) {
                     time = time *
                         (std::abs(std::sin(gradient(from, to)))
                          + std::abs(std::cos(gradient(from, to))));
                 }

                 travel_Time[from][to] = time;
                 setTwcij(from, to);
             }

         void setTravelTimeOsrm(UID from, UID to) const {
             assert(from < original.size() && to < original.size());
             assert(osrmi->getConnection());

             bool oldStateOsrm = osrmi->getUse();
             osrmi->useOsrm(true);
             osrmi->clear();

             // Add nodes
             std::deque< Node > call;
             // First node
             call.push_back(original[from]);
             // After node?
             auto it = mPhantomNodes.find( original[from].id() );
             if ( it!=mPhantomNodes.end() ) {
                 Twnode an = Twnode(mIPNId,mPNId,it->second.afterPNode().x(),it->second.afterPNode().y());
                 an.set_type( Twnode::kPhantomNode );
                 call.push_back(an);
             }
             // Before node?
             it = mPhantomNodes.find( original[to].id() );
             if ( it!=mPhantomNodes.end() ) {
                 Twnode bn = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
                 bn.set_type( Twnode::kPhantomNode );
                 call.push_back(bn);
             }
             // Last node
             call.push_back(original[to]);

             osrmi->addViaPoint(call);

             if (!osrmi->getOsrmViaroute()) {
                 DLOG(INFO) << "getOsrmViaroute failed";
                 osrmi->useOsrm(oldStateOsrm);
                 setTravelTimeNonOsrm(from, to);
                 return;
             }


             travel_Time[from][to] = osrmi->getOsrmTime();
#ifdef VRPMINTRACE
             DLOG(INFO) << "travel_Time[" << from << "][" << to << "]=" << time;
#endif
             osrmi->useOsrm(oldStateOsrm);
             setTwcij(from, to);
         }

 public:
         void setTravelTime(UID from, UID to) const {
             assert(travel_Time[from][to] == -1);
             if (!osrmi->getConnection()) {
                 setTravelTimeNonOsrm(from, to);
                 return;
             }
             setTravelTimeOsrm(from, to);
             return;
         }
#endif // USE


 private:
         double getTravelTime(UID from, UID to) const;



 public:
         /*!  \brief Retruns travel time from node id \b from to node id \b to.
           interfaces
           */
         //@{
         double TravelTime(UID from, UID to) const;

         double TravelTime(UID from, UID middle, UID to) const;


         double TravelTime(UID prev, UID from, UID middle, UID to) const;

         double TravelTime(const Trashnode &from, const Trashnode &to) const;

         double TravelTime(const Trashnode &from, const Trashnode &middle, const Trashnode &to) const;

         double TravelTime(const Trashnode &prev, const Trashnode &from, const Trashnode &middle, const Trashnode &to) const;
         //@}

         bool isInPath(UINT from, UINT middle, UINT to);

         bool isInPath(const Trashnode &from, const Trashnode &middle, const Trashnode& to);

















#if 0 // def USE
         /*!
          * \brief Fetch the time window compatibility value traveling from fromNid to toNid.
          *
          * Return the compatibility of servicing customer toNid directly after
          * fromNod. Higher values represent better compatibility of the two time
          * windows being considered. And incompatible time windows will have
          * negative infinity.
          *
          * \param[in] fromNid Nid of the from node.
          * \param[in] toNid Nid of the to node.
          * \return The time window compatibility of traveling directly \b fromNid to \b toNide.
          */
         double compatibleIJ(UID fromNid, UID toNid) const {
             assert(fromNid < original.size() && toNid < original.size());
             return  getTwcij(fromNid, toNid);
         }
#endif  //USE

         /*!
          * \brief Fetch a node by its node id from the original container of nodes.
          *
          * \param[in] nid The node id of the desired node.
          * \return A copy of the node from the original container of nodes.
          */
         const Trashnode &node(UID nid) const;

         /*!
          * \brief Fetch a node by its node id from the original container of nodes.
          *
          * \param[in] nid The node id of the desired node.
          * \return A copy of the node from the original container of nodes.
          */
         const Trashnode &getNode(UID nid) const;

#if 0 // def USE
         // ---------------- state -----------------------------------

         /*!
          * \brief Report if traveling fromNid to toNide is compatibly.
          *
          * \param[in] fromNid Nid of the from node.
          * \param[in] toNid Nid of the to node.
          * \return True if compatibile, false otherwise.
          */
         bool isCompatibleIJ(UID fromNid, UID toNid) const {
             assert(fromNid < original.size() && toNid < original.size());
             return !(getTwcij(fromNid, toNid)  == VRP_MIN());
         }
#endif  //USE

         /*! \brief Report toNide is logically reachable from fromNid.
          *
          * \param[in] fromNid Nid of the from node.
          * \param[in] toNid Nid of the to node.
          * \return True if reachable, false otherwise.
          */
         bool isReachableIJ(UID fromNid, UID toNid) const;


#if 0 // def USE
         /*!
          * \brief Report compatibility of traveling fromNid to middleNid to toNid.
          *
          * Check time window compatibility for traveling through a three node
          * sequence of fromNid to middleNid to toNid.
          *
          * \param[in] fromNid First node id in three node sequence to be checked.
          * \param[in] middleNid Second node id in three node sequence to be checked.
          * \param[in] toNid Third node id in three node sequence to be checked.
          * \return True if it is compatible to travel fromNid to middleNid to toNid.
          * \bug I (vicky)  dont think transitivity applies, and I think the process is more complex
          */
         bool isCompatibleIAJ(UID fromNid, UID middleNid, UID toNid) const {
             assert(fromNid < original.size() && middleNid < original.size()
                     && toNid < original.size());
             isCompatibleIJ(fromNid, middleNid);
             isCompatibleIJ(middleNid, toNid);
             return isCompatibleIJ(fromNid, middleNid)
                 && isCompatibleIJ(middleNid, toNid);
         }


         /*!
          * \brief Report compatibility of traveling through a three node sequence.
          *
          * Check time window compatibility for traveling through a three node
          * sequence of \b from node to \b middle node to \b to node.
          *
          * \param[in] from First node in three node sequence to be checked.
          * \param[in] middle Second node in three node sequence to be checked.
          * \param[in] to Third node in three node sequence to be checked.
          * \return True if it is compatible to travel fromNid to middleNid to toNid.
          */
         bool isCompatibleIAJ(const Trashnode &from, const Trashnode &middle,
                 const Trashnode &to) const {
             return isCompatibleIAJ(from.nid(), middle.nid() , to.nid());
         }

         // ----------------- The best or the worses -----------------------

         /*!
          * \brief Search a bucket of nodes for the one with the best travel time.
          *
          * Given a bucket of nodes and a from node id, search the bucket for the
          * node that gives the best travel time and is Reachable from the node id.
          * If all nodes are unReachable then just return the first node in the
          * bucket.
          *
          * \param[in] from Node id from which we want the best travel time to a node in \b nodes.
          * \param[in] nodes A bucket of nodes that we want to search.
          * \return The node with the best travel time from node id \b from.
          * \return Or the first node in the bucket if all are unReachable.
          */
    public:
         //Trashnode findBestTravelTime(UID from, const Bucket &nodes) const {
         bool findBestTravelTime(const Trashnode &from, const Bucket &nodes, Trashnode &bestNode) const {
             assert(nodes.size() && from.nid() < original.size());
             Bucket reachable = getReachable(from.nid(), nodes);

             bestNode = from;
             if ( !reachable.size() ) return false;

             double bestTime = VRP_MAX();

             for ( int i = 0; i < reachable.size(); i++ ) {
                 if (reachable[i].nid() != from.nid()
                         && TravelTime(from.nid(), reachable[i].nid()) < bestTime ) {
                     bestNode = reachable[i];
                     bestTime = TravelTime(from.nid(), reachable[i].nid());
                 }
             }

             return (bestNode != from);
         }


         /// \brief find best node to arrive to from "from"
         /*!

           \param[in] from
           \param[in] nodes: main nodes bucket
           \param[in/out] streetNodes: secundary nodes bucket
           \param[out] bestNode: Node that minimizes TravelTime[from,bestNode]
           \returns true when a bestNode was found
           */
         bool findBestTravelTimeUseStreetUseHint(
                 const Trashnode &from,
                 const Bucket &nodes,
                 Bucket &streetNodes,
                 Trashnode &bestNode) const {
             assert(nodes.size() && from.nid() < original.size());

             Bucket reachable;
             uint64_t streetId;
             if (streetNodes.size() == 0) {
                 streetId = from.streetId();
                 reachable  = getReachable(from.nid(), nodes);
                 // from the reachable nodes get the nodes that belong to the
                 // same street
                 for (int i = 0; i < reachable.size(); i++) {
                     if (reachable[i].streetId() == streetId) {
                         streetNodes.push_back(reachable[i]);
                     }
                 }
                 if (streetNodes.size() != 0) {
                     reachable = streetNodes;
                 }
             } else {
                 reachable = getReachable(from.nid(),streetNodes);
             }


             bestNode = from;
             if (!reachable.size()) return false;

             double bestTime = VRP_MAX();

             for (int i = 0; i < reachable.size(); i++) {
                 // the node we are looking for is not the node we are comming from
                 assert (reachable[i].nid() != from.nid());
                 if (reachable[i].nid() == from.nid()) continue;
                 if (reachable[i].hint() == from.hint()) {
                     bestNode = reachable[i];
                     return true;
                 }
                 if (TravelTime(from.nid(), reachable[i].nid()) < bestTime ) {
                     bestNode = reachable[i];
                     // bestTime = TravelTime(from.nid(), reachable[i].nid());
                 }
             }

             return (bestNode != from);
         }

         /*!
          * \brief Search a bucket for the node with the best travel time to node id \b to.
          *
          * Given a bucket of nodes and a to node id, search the bucket for the
          * node that gives the best travel time and is Reachable to arrive at node
          * id \b to.
          *
          * \param[in] nodes A bucket of nodes that we want to search.
          * \param[in] to Node id to which we want the best travel time from a node in \b nodes.
          * \return The node with the best travel time to node id \b to.
          * \return Or the first node in the bucket if all are unReachable.
          */
         Trashnode findBestTravelTime(const Bucket &nodes, UID to) const {
             assert(nodes.size() && to < original.size());
             Bucket reachable = getReachable(nodes, to);

             if ( !reachable.size() ) return nodes[0];

             Trashnode best = reachable[0];
             double bestTime = VRP_MAX();

             for ( int i = 0; i < reachable.size(); i++ ) {
                 if ( reachable[i].nid() != to
                         && travelTime(reachable[i].id(), to) < bestTime ) {
                     best = reachable[i];
                     bestTime = travelTime(reachable[i].id(), to);
                 }
             }

             return best;
         }

         /*!
          * \brief Search a bucket for the node with the worst travel time from node id \b from.
          *
          * Given a bucket of nodes and a from node id, search the bucket for the
          * node that gives the worst travel time and is Reachable departing from
          * node id \b from.
          *
          * \param[in] from The from node id for the search.
          * \param[in] nodes A bucket of nodes to search through.
          * \return The node with the best travel time from node id \b from.
          * \return Or the first node in the bucket if all are unReachable.
          */
         Trashnode findWorseTravelTime(UID from, const Bucket &nodes) const {
             // from the reachable nodes finds the worse
             assert(nodes.size() && from < original.size());
             Bucket reachable = getReachable(from, nodes);

             if ( !reachable.size() ) return nodes[0];

             Trashnode worse = reachable[0];
             double worseTime = VRP_MIN();

             for ( int i = 0; i < reachable.size(); i++ ) {
                 if ( reachable[i].nid() != from
                         && travelTime(from, reachable[i].id()) > worseTime ) {
                     worse = reachable[i];
                     worseTime = travelTime(from, reachable[i].id());
                 }
             }

             return worse;
         }

         /*!
          * \brief Search a bucket for the node with the worst travel time to node id \b to.
          *
          * Given a bucket of nodes and a from node id, search the bucket for the
          * node that gives the worst travel time and is Reachable arriving at
          * node id \b to.
          *
          * \param[in] nodes A bucket of nodes to search through.
          * \param[in] to The from node id for the search.
          * \return The node with the best travel time to node id \b to.
          * \return Or the first node in the bucket if all are unReachable.
          */
         Trashnode findWorseTravelTime(const Bucket &nodes, UID to) const {
             // from the reachable nodes finds the worse
             assert(nodes.size() && to < original.size());
             Bucket reachable = getReachable(nodes, to);

             if ( !reachable.size() ) return nodes[0];

             Trashnode worse = reachable[0];
             double worseTime = VRP_MIN();

             for (int i = 0; i < reachable.size(); i++) {
                 if (reachable[i].nid() != to
                         && travelTime(reachable[i].id(), to) > worseTime) {
                     worse = reachable[i];
                     worseTime = travelTime(reachable[i].id(), to);
                 }
             }

             return worse;
         }

         /*!
          * \brief Get a seed node id from a bucket of nodes for creating an new route.
          *
          * The goal is to pick a node from the bucket based on the time window
          * compatibility of that node with respect to the other nodes in the bucket.
          *
          * \warning ec2 get seed (needs revision)
          * \warning This code returns a node id, not clear based how it works!
          *
          * \param[in] foo Unused
          * \param[in] nodes A bucket of nodes from which we want to select a seed.
          * \return The node id of a seed node.
          */
         int getSeed(int foo, const Bucket &nodes) const {
             // ec2 get seed (needs revision)
             int bestId, count;
             double bestEc2;
             int Id;
             int bestCount = 0;
             bestEc2 = - VRP_MIN();

             for ( int i = 0; i < nodes.size(); i++ ) {
                 if ( i == 0 ) bestId = nodes[0].nid();

                 Id = nodes[i].nid();
                 count = 0;

                 for ( int j = 0; j < nodes.size(); j++ ) {
                     if ( i != j &&  isCompatibleIJ( Id , nodes[j].nid() ) ) count++;

                     bestCount = count;
                     bestId = Id;
                 }
             }

             return bestId;
         }

         /*!
          * \brief Get the most TW compatable node as a successor from the bucket.
          *
          * Given a bucket of nodes and from node id, search the bucket for the
          * node that is most TW compatible as a successor to node id.
          *
          * \param[in] fromNid The reference node id.
          * \param[in] nodes A bucket of nodes to search through.
          * \return The node id of the most most compatible node, or
          * \return -1 if empty bucket or there are no compatible nodes.
          */
         int  getBestCompatible(UID fromNid, const Bucket &nodes) const {
             assert(fromNid < original.size());
             UID bestId;
             UID toId;

             if ( nodes.empty() ) return -1;

             bestId = nodes[0].nid();

             for ( int j = 0; j < nodes.size(); j++ ) {
                 toId = nodes[j].nid();

                 if ( getTwcij(fromNid, toId) > getTwcij(fromNid, bestId) ) {
                     bestId = toId;
                 }
             }

             if (compat(fromNid, bestId) != VRP_MIN())
                 return bestId;
             else
                 return -1;
         }

         /*!
          * \brief Compute the total TW compatibility for node id \c at in a bucket.
          *
          * Compute the total TW compatibility for node id \b at. The node with
          * the lowest total TW compatibility should be select as the seed for
          * a new route during initial construction.
          *
          * \sa Equation 2. in reference article
          *
          * \warning Review code against reference article, because the Equation 2
          *          does not match what this code is doing.
          *
          * \param[in] at Node id to compute the total compatibility for.
          * \param[in] nodes A bucket of nodes containing node id \b at.
          * \return The total TW compatibility value for node id \b at.
          */
         double ec2(UID at, const Bucket &nodes) {
             assert(at < original.size());
             double ec2_tot = 0;

             for ( int j = 0; j < nodes.size(); j++ ) {
                 if ( !(getTwcij(at, j)  == VRP_MIN()) ) ec2_tot += getTwcij(at, j);
                 if ( !(getTwcij(j, at)  == VRP_MIN()) ) ec2_tot += getTwcij(j, at);
             }

             if (getTwcij(at, at) == VRP_MIN()) ec2_tot -= getTwcij(at, at);

             return ec2_tot;
         }


         // ---------------- counting ---------------------------------


         /*!
          * \brief Count the number of nodes that are incompatible from a node.
          *
          * \param[in] at A reference node id to be used in counting.
          * \param[in] nodes A bucket that is to be used in counting.
          * \return The number of nodes that are incompatible from node id \b at.
          */
         int countIncompatibleFrom(UID at, const Bucket &nodes) {
             assert(at < original.size());
             int count = 0;

             for ( UID j = 0; j < nodes.size(); j++ ) {
                 if ( getTwcij(at, j)  == VRP_MIN() ) count++;
             }

             return count;
         }

         /*!
          * \brief Count the number of nodes that are incompatible as sucessors of a node.
          *
          * \param[in] at A reference node id to be used in counting.
          * \param[in] nodes A bucket that is to be used in counting.
          * \return The number of nodes that are incompatibleas sucessors to node id \b at.
          */
         int countIncompatibleTo(UID at, const Bucket &nodes) {
             int count = 0;

             for ( UID j = 0; j < nodes.size(); j++ ) {
                 if ( getTwcij(j, at)  == VRP_MIN() ) count++;
             }

             return count;
         }
#endif // USE



         // ------------------------ DUMPS --------------------------

#ifdef DOVRPLOG

         void print_nodes_onTrip(size_t from, size_t to) const;

         /*! \brief Print the original nodes.  */
         void dump() const;

         /*!
          * \brief Print the nodes in the bucket.
          * \param[in] nodes A bucket of nodes to print.
          */
         void dump(const TwBucket &nodes) const;

#if 0 // def USE
         /*! \brief Print the TW Compatibility matrix for the original nodes.  */
         void dumpCompatability() const;

         /*!
          * \brief Print the TW Compatibility matrix for the input bucket.
          * \param[in] nodes  A bucket of nodes to print the TWC matrix.
          */
         void dumpCompatability(const Bucket &nodes) const;

         /*!
          * \brief Print the travel time matrix for the original nodes.
          */
         void dumpTravelTime() const {
             assert(original.size());
             dumpTravelTime(original);
         }

         /*!
          * \brief
          *
          * \param[in] nodes A bucket of nodes to print the travel time matrix for.
          */
         void dumpTravelTime(const Bucket &nodes) const;
#endif  // USE



#endif  // logs

         // ------------ go back to CALCULATED state -------------------

#if 0 // def USE
         /*!
          * \brief Recompute the TWC matrix entries for a given node.
          *
          * \param[in] nid The node id to update the TWC matrix entries for.
          */
         void recreateCompatible(UID nid) {
             assert(nid < original.size());

             for (int j = 0; j < twcij.size(); j++) {
                 twcij[nid][j] = twc_for_ij(original[nid], original[j]);
                 twcij[j][nid] = twc_for_ij(original[j], original[nid]);
             }
         }
#endif  // USE




         /*!
          * \brief Set TWC from fromNid to toNid to be incompatible & unreachable.
          *
          * \param[in] fromNid The predecessor node id.
          * \param[in] toNid The successor node id.
          */
         void setIncompatible(UID fromNid, UID toNid);

#if 0 // def USE
         /*!
          * \brief Set TWC incompatible  & unreachable from nid to all nodes in the bucket.
          *
          * \param[in] nid The from node id that we want set as incompatible.
          * \param[in] nodes A bucket of successor nodes that are incompatible from \b nid.
          */
         void setIncompatible(UID nid, const Bucket &nodes);


         /*!
          * \brief Set TWC incompatible to nid for all nodes in the bucket.
          *
          * \param[in] nodes A bucket of predecessor nodes that are incompatible with \b nid.
          * \param[in] nid The successor node id that we want set as incompatible.
          */
         void setIncompatible(const Bucket &nodes, UID &nid);


         /*!
          * \brief Set the travel time between \b fromNid and \b toNid as unReachable
          *
          * \param[in] fromNid The from node id to set.
          * \param[in] toNid The to node id to set.
          */
         void setUnreachable(UID fromNid, UID toNid);


         /*!
          * \brief Set all nodes in the bucket as unReachable from nid
          *
          * \param[in] nid The from node id we are set as unReachable.
          * \param[in] nodes A bucket of successor nodes to set as unReachable.
          */
         void setUnreachable(UID nid, const Bucket &nodes);

         /*!
          * \brief Set all nodes in the bucket as unReachable predecessors of nid.
          *
          * \param[in] nodes A bucket of predecessor nodes to set as unReachable.
          * \param[in] nid The successor node that is unRechable from the bucket nodes.
          */
         void setUnreachable(const TwBucket &nodes, UID &nid);
#endif  // USE


         /*!
          * \brief Assign the bucket of nodes as the TWC original and compute TWC.
          *
          * \param[in] _original A bucket of nodes to assign to TWC class.
          */
         void setNodes(TwBucket _original);


         /*!
          * \brief Compute the average travel time to a given node.
          *
          * \param[in] from A bucket of nodes to use as the start node.
          * \param[in] to A node to be used as the destination node.
          * \return The average travel time from start to destination.
          */
         double getAverageTime(const TwBucket &from, const Trashnode &to) const;

         /*!
          * \brief Compute the average travel time from a given node.
          *
          * \param[in] from The start node.
          * \param[in] to A bucket of destination nodes.
          * \return The average travel time from start to destination.
          */
         double getAverageTime(const Trashnode &from, const TwBucket &to) const;







         /*!
          * \brief Set tCC set average travel time  between containers in bucket picks
          * \param[in] C    average contaienr
          * \param[in] picks
          */
         void settCC(const Trashnode &C, const TwBucket &picks);




         /*!
          * \brief Test if two nodes are on the same street.
          *
          * \warning This is dependent on street ids being set.
          *
          * \param[in] i Node id 1
          * \param[in] j Node id 2
          * \return True if both nodes are on the same street.
          */
         bool sameStreet(UID i, UID j) const;

         /*!
          * \brief Compute the gradient or slope of line from node i to j
          *
          * \bug This function calls Node::gradient that might divide by zero.
          *
          * \param[in] i Node id 1
          * \param[in] j Node id 2
          * \return The gradient of the line.
          */
         double gradient(UID i, UID j) const;



         /// \brief Sets the hints and the street's id to the nodes in the bucket
         /**

           The hint & street when id stored in "original" is copyied into the bucket's
           nodes.

           To be used after "original" has being filled with the appropiate values
           */
         void setHints(TwBucket &nodes);


    private:
         void prepareTravelTime();


    private:
         void getAllHintsAndStreets();

    public:

         // Just for log. Dump travel_Time matrix.
         void dump_travel_Time();

         /*!  \brief Assign the travel time matrix to TWC from Pg

           This method is specific for PostgreSQL integration. It receives a
           ttime_t structure array containing the travel time matrix values passed
           from the database and loads them into the problem and does some
           additional needed computations.

           \param[in] ttimes The travel time data array from PosgreSQL
           \param[in] count The count of entries in \b ttimes
           \param[in] datanodes The data nodes Bucket previous loaded from PostgreSQL
           \param[in] invalid The bucket of invalid nodes generated when load the data nodes.
           */
         void loadAndProcess_distance(ttime_t *ttimes, int count,
                 const TwBucket &datanodes, const TwBucket &invalid);


    public:


         /*!
          *  @brief Load the travel time matrix from a text file and process the results.

          *  Reads @b infile and loads it into the travel time matrix and populates
          *  any missing entries we=ith an approx distance. It also computes the TWC
          *  matrix.

          *  file format (separated by spaces):
          *  ~~~~{}
          *  comment
          *  from to time
          *  ~~~~

          *  from id of the node 
          *  to   id of the node 
          *  time  time to go from node "from" to node "to"

          *  @param[in] infile: The file name to load.
          *  @param[in] datanodes: The bucket of data nodes that has already been loaded.
          *  @param[in] invalid: A bucket of invalid nodes found in the data nodes.
          */

         void loadAndProcess_travelTimes(
                 std::string infile,
                 const TwBucket &datanodes,
                 const TwBucket &invalid);



         /*! \brief Returns a constant reference to the travel time matrix. */
         const std::vector<std::vector<double> >& TravelTime();
         inline double travel_time(
                 const Trashnode &from,
                 const Trashnode &to) const {
             return travel_Time[from.nid()][to.nid()];
         }

         /*! \brief Retrieves the internal node id (NID) from the users node id (ID)
          *
          * \param[in] id A user node identifier
          * \return The internal nid corresponding to id or -1 if not found.
          */
         UID getNidFromId(UID id) const;




    private:
         // constructors
         // static TWC* p_twc;
         TWC() = default;

    public:
         static TWC& Instance() {
             static TWC ref_twc;
             return ref_twc;
         }
         TWC(const TWC&) = delete;
         TWC& operator=(const TWC &) = delete;
         TWC(TWC &&) = delete;
         TWC& operator=(TWC &&) = delete;


#if 0 // def USE
         /*!
          * \brief Fetch the time window compatibility value traveling from nids i to j.
          *
          * Return the compatibility of servicing customer toNid directly after
          * fromNod. Higher values represent better compatibility of the two time
          * windows being considered. And incompatible time windows will have
          * negative infinity.
          *
          * \param[in] i Nid of the from node.
          * \param[in] j Nid of the to node.
          * \return The time window compatibility of traveling directly \b fromNid to \b toNide.
          */
         double compat(int i, int j) const {
             assert(i < original.size() && j < original.size());
             return twcij[i][j];
         }
#endif // USE


         /*!
          * \brief The earliest arrival time at \b nj from the latest departure from \b ni
          *
          * The earliest arrival time at \b nj, given that node \b nj is visted
          * directly after \b ni, and that we departed \b ni at the latest
          * possible time.
          *
          * \param[in] ni The node we departed from.
          * \param[in] nj The node we arrived at.
          * \return The earliest arrival time at \b nj
          */
         double ajli(const Trashnode &ni, const Trashnode &nj) const;

         /*!
          * \brief The earliest arrival time at \b nj from the earliest departure from \b ni
          *
          * The earliest arrival time at \b nj, given that node \b nj is visted
          * directly after \b ni, and that we departed \b ni at the earliest
          * possible time.
          *
          * \param[in] ni The node we departed from.
          * \param[in] nj The node we arrived at.
          * \return The earliest arrival time at \b nj
          */
         double ajei(const Trashnode &ni, const Trashnode &nj) const;


         /*!
          * \brief Compute TWC from node \b ni to node \b nj
          *
          * \param[in] ni From this node
          * \param[in] nj To this node
          * \return The TWC value traveling from node \b ni directly to \b nj
          */
         double twc_for_ij(const Trashnode &ni, const Trashnode &nj) const;

#if 0 // def USE
         double getTwcij(UID i, UID j) const {  // this one makes twcij dynamical
             if  ( travel_Time[i][j] == -1 ) {
                 TravelTime(i, j);
                 twcij[i][j] = twc_for_ij(original[i], original[j]);
             }

             return twcij[i][j];
         }

         double setTwcij(UID i, UID j) const {
#ifdef VRPMAXTRACE
             DLOG( INFO ) << "twcij size: " << twcij.size() << "\n";
             DLOG( INFO ) << "original size: " << original.size() << "\n";
#endif
             if ( twcij.size() == 0 ) {
                 twcij.resize(original.size());

                 for (int k = 0; k < original.size(); k++)
                     twcij[k].resize(original.size());
             }
#ifdef VRPMAXTRACE
             DLOG( INFO ) << "twcij size: " << twcij.size() << "\n";
             DLOG( INFO ) << "original size: " << original.size() << "\n";
#endif
             twcij[i][j] = twc_for_ij(original[i], original[j]);
             return twcij[i][j];
         }
#endif  //USE


         /* public functions That are id based */


         /*!
          * \brief Compute all TWC values and populate the TWC matrix.
          */
         void twcij_calculate();

         /*!  \brief Check's that the twcij was created
           O(N) where N is the number of nodes
           */
         bool check_integrity() const;

    public:

         void set_TravelTime(UID fromId, UID toId, double time);

};  // end of class



#define twc TWC::Instance()


#endif  // SRC_BASECLASSES_TWC_H_
