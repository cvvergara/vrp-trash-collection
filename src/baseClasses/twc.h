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

// #include <iostream>
#include <fstream>
// #include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <map>
// #include <cmath>
#include <algorithm>
#include <utility>
// #include <math.h>
// #include <limits>

#ifdef DOVRPLOG
#include "./logger.h"
#endif

#ifdef OSRMCLIENT
#include "./osrmclient.h"
#endif

#ifdef DOSTATS
#include "./timer.h"
#include "./stats.h"
#endif

#include "./basictypes.h"
#include "./node.h"
#include "./twpath.h"
#include "./singleton.h"
#include "./pg_types_vrp.h"


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
template <class knode> class TWC {
 private:
  typedef TwBucket<knode> Bucket;

#ifdef OSRMCLIENT
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

  struct classcomp {
    bool operator() (const TTindex &lhs, const TTindex &rhs) const {
      return lhs.prev() < rhs.prev() ? true : lhs.from() < rhs.from() ? true : lhs.middle() <
             rhs.middle() ? true : lhs.to() < rhs.to();
    }
  };
  typedef std::map<TTindex, double, classcomp>  TT4;
  typedef typename std::map<TTindex, double, classcomp>::iterator p_TT4;
  mutable TT4 travel_Time4;
#endif

  TwBucket<knode> original;
  mutable std::vector<std::vector<double> > twcij;
  mutable std::vector<std::vector<double> > travel_Time;


 public:
  bool emptiedTruck;
  int z1Tot;
  int z2Tot;

  /*! \brief cleans all the tables, leaving them blanck for a next execution */
  void cleanUp() {
    original.clear();
    twcij.clear();
    travel_Time.clear();
#ifdef OSRMCLIENT
    travel_Time4.clear();
#endif
  }




  // -------------------  major tools  ----------------------------

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
  bool findNearestNodeTo(const TwBucket<knode> &truck,
                          const  TwBucket<knode> &unassigned,
                          POS &pos,
                          knode &bestNode,
                          double &bestDist) const {
    assert(unassigned.size());
    int flag = false;
    bestDist = VRP_MAX();   // dist to minimize
    pos = 0;        // position in path to insert
    double d;


    for ( int i = 0; i < unassigned.size(); i++ ) {
      for ( int j = 0; j < truck.size() - 1; j++ ) {
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

  bool  findNearestNodeUseExistingData(const TwBucket<knode> &truck,
                                       const  TwBucket<knode> &unassigned,
                                       POS &pos,
                                       knode &bestNode,
                                       double &bestDist) const {
    assert(unassigned.size());
    int flag = false;
    bestDist = VRP_MAX();   // dist to minimize
    pos = 0;        // position in path to insert
    double d;


    for ( int i = 0; i < unassigned.size(); i++ ) {
      for ( int j = 0; j < truck.size() - 1; j++ ) {
        if (!(j == 0)
             && ((travel_Time[truck[j].nid()][unassigned[i].nid()] == -1)
                  || (travel_Time[unassigned[i].nid()][truck[j + 1].nid()] == -1)))
          // all nodes from the depot that are missing should be calculated
          continue;

        if ( isCompatibleIAJ(truck[j], unassigned[i], truck[j + 1]) ) {
          d = truck.segmentDistanceToPoint(j , unassigned[i]);

          if ( d < bestDist ) {
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
  ///@}

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
  TwBucket<knode> getUnreachable(const TwBucket<knode> &nodes, UID to) const {
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
  TwBucket<knode> getUnreachable(UID from, const TwBucket<knode> &nodes) const {
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
  TwBucket<knode> getReachable(const TwBucket<knode> &nodes, UID to) const {
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
  TwBucket<knode> getReachable(UID from, const TwBucket<knode> &nodes) const {
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
  TwBucket<knode> getIncompatible(const TwBucket<knode> &nodes, UID to) const {
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
  TwBucket<knode> getIncompatible(UID from,
                                   const TwBucket<knode> &nodes) const {
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
  TwBucket<knode> getCompatible(const TwBucket<knode> &nodes, UID to) const {
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
  TwBucket<knode> getCompatible(UID from, const TwBucket<knode> &nodes) const {
    assert(from < original.size());
    Bucket compatible;

    for ( int i = 0; i < nodes.size(); i++ )
      if (isCompatibleIJ(from, nodes[i].nid()))
        compatible.push_back(nodes[i]);

    return compatible;
  }


  /*! \todo comments   */
 private:
  void setTravelTimeNonOsrm(UID from, UID to) const {
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

#ifdef OSRMCLIENT
  void setTravelTimeOsrm(UID from, UID to) const {
    assert(from < original.size() && to < original.size());
    double time;
    if (!osrm->getConnection()) {
      setTravelTimeNonOsrm(from, to);
      return;
    }

    bool oldStateOsrm = osrm->getUse();
    osrm->useOsrm(true);

    if (!osrm->getOsrmTime(original[from], original[to], time)) {
      setTravelTimeNonOsrm(from, to);
      return;
    }

    osrm->useOsrm(oldStateOsrm);
    travel_Time[from][to] = time;
    setTwcij(from, to);
  }
#endif

  void setTravelTime(UID from, UID to) const {
    assert(travel_Time[from][to] == -1);
    #ifndef OSRMCLIENT
    setTravelTimeNonOsrm(from, to);
    return;
    #else
    if (!osrm->getConnection()) {
      setTravelTimeNonOsrm(from, to);
      return;
    }
    setTravelTimeOsrm(from, to);
    return;
    #endif
  }

  void fillTravelTime() const {
    int siz = travel_Time.size();
    for ( int i = 0; i < siz; i++ )
      for ( int j = i; j < siz; j++ ) {
        if ( i == j ) {
          travel_Time[i][i] = 0.0;
        } else {
          if (travel_Time[i][j] == -1)
             setTravelTime(i, j);
          if (travel_Time[j][i] == -1)
             setTravelTime(j, i);
        }
      }
  }

  double getTravelTime(UID from, UID to) const {
    assert(from < original.size() && to < original.size());
    double time;
    if (travel_Time[from][to] == -1) fillTravelTime();
    return travel_Time[from][to];
  }

 public:
  /*!  \brief Retruns travel time from node id \b from to node id \b to.
   (interface)
  */
  double TravelTime(UID from, UID to) const {
    return getTravelTime(from, to);
  }

  /*! \brief Fetch the travel time from node \b from to node \b to
   (interface)
  */
  double TravelTime(const knode &from, const knode &to) const {
    return getTravelTime(from.nid(), to.nid());
  }

  /*! \todo comments   */
 private:
#ifndef OSRMCLIENT
  double getTravelTime(UID prev, UID from, UID middle, UID to) const {
    assert(prev < original.size() && from < original.size()
            && middle < original.size() && to < original.size());

    if (prev == from && from == middle)
      return getTravelTime(middle, to);
    else
      return getTravelTime(prev, from) + getTravelTime(from, middle)
                 + getTravelTime(middle, to);
  }
#else  // with OSRM

  double getTravelTime(UID prev, UID from, UID middle, UID to) const {
    assert(prev < original.size() && from < original.size()
            && middle < original.size() && to < original.size());

    if (prev == from && from == middle) return getTravelTime(middle, to);

    TTindex index(prev, from, middle, to);

    p_TT4 it = travel_Time4.find(index);

    if (it != travel_Time4.end()) {
#ifdef DOSTATS
      if ( prev == from )
        STATS->inc("TWC::getTravelTime(3 parameters) found in table");
      else  STATS->inc("TWC::getTravelTime(4 parameters) found in table");
#endif
      return it->second;
    }

    double time;

    if ( prev == from ) {  // 3 parameters
      if (osrm->getOsrmTime(original[from], original[middle], original[to]
                            , time)) {
          travel_Time4.insert(std::pair<TTindex, double>(index, time));
          return time;
      }

      time = getTravelTime(from, middle) + getTravelTime(middle, to);
      travel_Time4.insert(std::pair<TTindex,double>(index,time));
      return time;
    }
    // 4 parameters
    if (osrm->getOsrmTime(original[prev], original[from], original[middle],
                              original[to], time)) {
      travel_Time4.insert(std::pair<TTindex,double>(index,time));
      return time;
    }
    time =  getTravelTime(prev, from) + getTravelTime(from, middle)
            + getTravelTime(middle, to);
    travel_Time4.insert(std::pair<TTindex,double>(index,time));
    return time;
  }
#endif  // with OSRM



 public:
  // this one is an interface
  double TravelTime(UID from, UID middle, UID to) const {
    assert(from < original.size());
    assert(middle < original.size());
    assert(to < original.size());
    return getTravelTime(from, from, middle, to);
  }

  // this one is an interface, the other one is the one that does all the work
  double TravelTime(const knode &from, const knode &middle,
                     const knode &to) const {
    return getTravelTime(from.nid(), from.nid(), middle.nid(),
                          to.nid());
  }

  // this one is an interface, the other one is the one that does all the work
  double TravelTime(const knode &prev, const knode &from, const knode &middle,
                     const knode &to) const {
    return getTravelTime(prev.nid(), from.nid(), middle.nid(),
                          to.nid());
  }

  // this one is an interface, the other one is the one that does all the work
  double TravelTime(UID prev, UID from, UID middle, UID to) const {
    assert(prev < original.size());
    assert(from < original.size());
    assert(middle < original.size());
    assert(to < original.size());
    return getTravelTime(prev, from, middle, to);
  }


















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

  /*!
   * \brief Fetch a node by its node id from the original container of nodes.
   *
   * \param[in] nid The node id of the desired node.
   * \return A copy of the node from the original container of nodes.
   */
  const knode &node(UID nid) const {
    assert(nid < original.size());
    return original[nid];
  }

  /*!
   * \brief Fetch a node by its node id from the original container of nodes.
   *
   * \param[in] nid The node id of the desired node.
   * \return A copy of the node from the original container of nodes.
   */
  const knode &getNode(UID nid) const {
    assert(nid < original.size());
    return original[nid];
  }

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

  /*! \brief Report toNide is logically reachable from fromNid.
   *
   * \param[in] fromNid Nid of the from node.
   * \param[in] toNid Nid of the to node.
   * \return True if reachable, false otherwise.
   */
  bool isReachableIJ(UID fromNid, UID toNid) const {
    assert(fromNid < original.size() && toNid < original.size());
    return !(TravelTime( fromNid, toNid )  == VRP_MAX());
  }


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
  bool isCompatibleIAJ(const knode &from, const knode &middle,
                        const knode &to) const {
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
  knode findBestTravelTime(UID from, const Bucket &nodes) const {
    assert(nodes.size() && from < original.size());
    Bucket reachable = getReachable(from, nodes);

    if ( !reachable.size() ) return nodes[0];

    knode best = reachable[0];
    double bestTime = VRP_MAX();

    for ( int i = 0; i < reachable.size(); i++ ) {
      if ( reachable[i].nid() != from
           && travelTime(from, reachable[i].id()) < bestTime ) {
        best = reachable[i];
        bestTime = travelTime(from, reachable[i].id());
      }
    }

    return best;
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
  knode findBestTravelTime(const Bucket &nodes, UID to) const {
    assert(nodes.size() && to < original.size());
    Bucket reachable = getReachable(nodes, to);

    if ( !reachable.size() ) return nodes[0];

    knode best = reachable[0];
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
  knode findWorseTravelTime(UID from, const Bucket &nodes) const {
    // from the reachable nodes finds the worse
    assert(nodes.size() && from < original.size());
    Bucket reachable = getReachable(from, nodes);

    if ( !reachable.size() ) return nodes[0];

    knode worse = reachable[0];
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
  knode findWorseTravelTime(const Bucket &nodes, UID to) const {
    // from the reachable nodes finds the worse
    assert(nodes.size() && to < original.size());
    Bucket reachable = getReachable(nodes, to);

    if ( !reachable.size() ) return nodes[0];

    knode worse = reachable[0];
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



  // ------------------------ DUMPS --------------------------

#ifdef DOVRPLOG

  /*! \brief Print the original nodes.  */
  void dump() const  {
    assert(original.size());
    dump(original);
  }


  /*!
   * \brief Print the nodes in the bucket.
   * \param[in] nodes A bucket of nodes to print.
   */
  void dump(const Bucket &nodes) const  {
    assert(nodes.size());
    dumpCompatability(nodes);
    dumpTravelTime(nodes);
  }

  /*! \brief Print the TW Compatibility matrix for the original nodes.  */
  void dumpCompatability() const  {
    assert(original.size());
    dumpCompatability(original);
  }

  /*!
   * \brief Print the TW Compatibility matrix for the input bucket.
   * \param[in] nodes  A bucket of nodes to print the TWC matrix.
   */
  void dumpCompatability(const Bucket &nodes) const  {
    std::stringstream ss;
    assert(nodes.size());

    ss.precision(8);
    ss << "COMPATABILITY TABLE \n\t";

    for ( int i = 0; i < nodes.size(); i++ )
      ss << "nid " << nodes[i].nid() << "\t";

    ss << "\n\t";

    for ( int i = 0; i < nodes.size(); i++ )
      ss << "id " << nodes[i].id() << "\t";

    ss << "\n";

    for ( int i = 0; i < nodes.size(); i++ ) {
      ss << nodes[i].nid() << "=" << nodes[i].id() << "\t";

      for ( int j = 0; j < nodes.size(); j++ ) {
        if (twcij[i][j] !=  VRP_MIN())
          ss << twcij[i][j] << "\t";
        else
          ss << "--\t";
      }

      ss << "\n";
    }

    DLOG(INFO) << ss.str();
  }

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
  void dumpTravelTime(const Bucket &nodes) const {
    std::stringstream ss;
    assert(nodes.size());
    ss << "\n\n\n\nTRAVEL TIME TABLE \n\t";

    ss.precision(2);

    for ( int i = 0; i < nodes.size(); i++ )
      ss << "nid " << nodes[i].nid() << "\t";

    ss << "\n\t";

    for ( int i = 0; i < nodes.size(); i++ )
      ss << "id " << nodes[i].id() << "\t";

    ss << "\n";

    for ( int i = 0; i < nodes.size(); i++ ) {
      ss << nodes[i].nid() << "=" << nodes[i].id() << "\t";

      for ( int j = 0; j < nodes.size(); j++ ) {
        if ( travel_Time[i][j] !=  VRP_MAX() )
          ss << travel_Time[i][j] << "\t";
        else
          ss << "--\t";
      }

      ss << "\n";
    }

    DLOG(INFO) << ss.str();
  }


  /*!
   * \brief Print the compatibility matrix using an alternate format for the original nodes.
   */
  void dumpCompatible3() const  {
    assert(original.size());
    dumpCompatible3(original);
  }

  /*!
   * \brief Print the compatibility matrix using an alternate format for the inptu bucket.
   *
   * \param[in] nodes The bucket of nodes to print the TWC for.
   */
  void dumpCompatible3(const Bucket &nodes) const {
    std::stringstream ss;
    assert(nodes.size());

    for ( int i = 0; i < nodes.size(); i++ ) {
      for ( int j = 0; j < nodes.size(); j++ ) {
        for ( int k = 0; k < nodes.size(); k++ ) {
          ss << "\t ( " << nodes[i].nid() << " , "
             << nodes[j].nid() << " , "
             << nodes[k].nid() << ") = "
             << (isCompatibleIAJ(i, j, k) ? "COMP" : "not");
        }

        ss << "\n";
      }
    }

    DLOG(INFO) << ss.str();
  }
#endif  // logs

  // ------------ go back to CALCULATED state -------------------

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


  // ---- Functions to adjust compatability depending on problem ----

#if 0
  /*!
   * \brief Recompute the travel time matrix for a given node.
   *
   * \todo This needs to be coded
   *
   * \param[in] nid The node we want to recomute traveltime for.
   * \param[in] mode Flag to indicate mode of calcualtion
   *                 - 0 = Euclidean point to point
   *                 - 1 = Haversine distance for lat-lon values
   *                 - 2 = Call getTimeOSRM()
   */
  void recreateTravelTime(UID nid, int mode) {
    assert("needs to be re-read from file" == "");
  }
#endif

  /*!
   * \brief Set TWC from fromNid to toNid to be incompatible & unreachable.
   *
   * \param[in] fromNid The predecessor node id.
   * \param[in] toNid The successor node id.
   */
  void setIncompatible(UID fromNid, UID toNid) {
    assert(fromNid < original.size() && toNid < original.size());
    twcij[fromNid][toNid] = VRP_MIN();
    travel_Time[fromNid][toNid] =  VRP_MAX();
  }


  /*!
   * \brief Set TWC incompatible  & unreachable from nid to all nodes in the bucket.
   *
   * \param[in] nid The from node id that we want set as incompatible.
   * \param[in] nodes A bucket of successor nodes that are incompatible from \b nid.
   */
  void setIncompatible(UID nid, const Bucket &nodes) {
    assert(nid < original.size());

    for (int j = 0; j < nodes.size(); j++) {
      twcij[nid][nodes[j].nid()] =  VRP_MIN();
      travel_Time[nid][nodes[j].nid()] =  VRP_MAX();
    }
  }


  /*!
   * \brief Set TWC incompatible to nid for all nodes in the bucket.
   *
   * \param[in] nodes A bucket of predecessor nodes that are incompatible with \b nid.
   * \param[in] nid The successor node id that we want set as incompatible.
   */
  void setIncompatible(const Bucket &nodes, UID &nid) {
    assert(nid < original.size());

    for (int i = 0; i < nodes.size(); i++) {
      twcij[nodes[i].nid()][nid] =  VRP_MIN();
      travel_Time[nodes[i].nid()][nid] =  VRP_MAX();
    }
  }


  /*!
   * \brief Set the travel time between \b fromNid and \b toNid as unReachable
   *
   * \param[in] fromNid The from node id to set.
   * \param[in] toNid The to node id to set.
   */
  void setUnreachable(UID fromNid, UID toNid) {
    assert(fromNid < original.size() && toNid < original.size());
    travel_Time[fromNid][toNid] = VRP_MAX();
  }


  /*!
   * \brief Set all nodes in the bucket as unReachable from nid
   *
   * \param[in] nid The from node id we are set as unReachable.
   * \param[in] nodes A bucket of successor nodes to set as unReachable.
   */
  void setUnreachable(UID nid, const Bucket &nodes) {
    assert(nid < original.size());

    for ( int j = 0; j < nodes.size(); j++ )
      travel_Time[nid][nodes[j].nid()] =  VRP_MAX();
  }

  /*!
   * \brief Set all nodes in the bucket as unReachable predecessors of nid.
   *
   * \param[in] nodes A bucket of predecessor nodes to set as unReachable.
   * \param[in] nid The successor node that is unRechable from the bucket nodes.
   */
  void setUnreachable(const Bucket &nodes, UID &nid) {
    assert(nid < original.size());

    for ( int i = 0; i < nodes.size(); i++)
      travel_Time[nodes[i].nid()][nid] =  VRP_MAX();
  }


  /*!
   * \brief Assign the bucket of nodes as the TWC original and compute TWC.
   *
   * \param[in] _original A bucket of nodes to assign to TWC class.
   */
  void setNodes(Bucket _original) {
    original.clear();
    original = _original;
    twcij_calculate();
    assert(original == _original);
    assert(check_integrity());
  }



  /*!
   * \brief Compute the average travel time to a given node.
   *
   * \param[in] from A bucket of nodes to use as the start node.
   * \param[in] to A node to be used as the destination node.
   * \return The average travel time from start to destination.
   */
  double getAverageTime(const Bucket &from, const knode &to) const {
    assert(to.nid() < original.size());
    double time = 0;
    int j = to.nid();

    for ( int i = 0; i < from.size(); i++ ) {
      time += TravelTime(from[i].nid(), j);
    }

    time = time / from.size();
    return time;
  }

  /*!
   * \brief Compute the average travel time from a given node.
   *
   * \param[in] from The start node.
   * \param[in] to A bucket of destination nodes.
   * \return The average travel time from start to destination.
   */
  double getAverageTime(const knode &from, const Bucket &to) const {
    assert(from.nid() < original.size());
    double time = 0;
    int j = from.nid();

    for ( int i = 0; i < to.size(); i++ )
      time += travel_Time[j][to[i].nid()];

    time = time / to.size();
    return time;
  }

  /*!
   * \brief Set tCC for a given node related to a given bucket
   *
   * \todo VICKY Please explain!
   *
   * \param[in] C
   * \param[in] picks
   */
  void settCC(const knode &C, const Bucket &picks) {
    int pos = C.nid();
    travel_Time[pos][pos] = getAverageTime(C, picks);
  }

  /*!
   * \brief Test if two nodes are on the same street.
   *
   * \warning This is dependent on street ids being set.
   *
   * \param[in] i Node id 1
   * \param[in] j Node id 2
   * \return True if both nodes are on the same street.
   */
  bool sameStreet(UID i, UID j) const {
    assert(i < original.size() && j < original.size());
    return original[i].sameStreet(original[j]);
  }

  /*!
   * \brief Compute the gradient or slope of line from node i to j
   *
   * \bug This function calls Node::gradient that might divide by zero.
   *
   * \param[in] i Node id 1
   * \param[in] j Node id 2
   * \return The gradient of the line.
   */
  double gradient(UID i, UID j) const {
    assert(i < original.size() && j < original.size());
    return original[i].gradient( original[j] );
  }



#ifdef OSRMCLIENT
  void setHints(Bucket &nodes) {
#ifdef DOSTATS
    Timer timer;
#endif

    for ( int i = 0; i < nodes.size(); i++ ) {
      nodes[i].set_hint(original[nodes[i].nid()].hint());
    }

#ifdef DOSTATS
    STATS->addto("TWC::setHints Cumultaive time:", timer.duration());
#endif
  }
#endif


 private:
  void prepareTravelTime() {
    int siz = original.size();
    travel_Time.resize(siz);

    for ( int i = 0; i < siz; i++ )
      travel_Time[i].resize(siz);

    // travel_Time default value is 250m/min
    for ( int i = 0; i < siz; i++ )
      for ( int j = i; j < siz; j++ ) {
        if ( i == j ) {
          travel_Time[i][i] = 0.0;
        } else {
          travel_Time[i][j] = travel_Time[j][i] = -1.0;
#ifndef OSRMCLIENT
          travel_Time[i][j] = travel_Time[j][i] = getTravelTime(i, j);
#endif
        }
      }
  }


#ifdef OSRMCLIENT
  void getAllHints() {
#ifdef DOSTATS
    Timer timer;
#endif

    std::deque<std::string> hints;
    int total = original.size();
    int from, to;
    int i, j, k;

    for ( i = 0; (i * 100) < total; i++ ) {
      from = i * 100;
      to = std::min((i + 1) * 100, total);
      hints.clear();
      osrm->clear();

      for ( j = from; j < to ; j++ ) osrm->addViaPoint(original[j]);

      if (osrm->getOsrmViaroute() && osrm->getOsrmHints(hints)) {
        for (j = from, k = 0; j < to; j++, k++) {
          original[j].set_hint(hints[k]);
        }
      }
    }

#ifdef DOSTATS
    STATS->addto("TWC::getAllHints Cumultaive time:", timer.duration());
#endif
  }
#endif


 public:
  /*!
   * \brief Assign the travel time matrix to TWC from Pg
   * \bug TODO needs to be tested when conected to the database
   * This method is specific for PostgreSQL integration. It receives a
   * ttime_t structure array containing the travel time matrix values passed
   * from the database and loads them into the problem and does some
   * additional needed computations.
   *
   * \param[in] ttimes The travel time data array from PosgreSQL
   * \param[in] count The count of entries in \b ttimes
   * \param[in] datanodes The data nodes Bucket previous loaded from PostgreSQL
   * \param[in] invalid The bucket of invalid nodes generated when load the data nodes.
   */
  void loadAndProcess_distance(ttime_t *ttimes, int count,
                               const Bucket &datanodes, const Bucket &invalid) {
#ifdef DOVRPLOG
    DLOG(INFO) << "POSTGRES: loadAndProcess_distance needs to be TESTED";
#endif
    assert(datanodes.size());
    original.clear();
    original = datanodes;

#ifdef OSRMCLIENT
    getAllHints();
#endif

    prepareTravelTime();

    for (int i = 0; i < count; ++i) {
      int from    = ttimes[i].from_id;
      int to      = ttimes[i].to_id;
      double time = ttimes[i].ttime;

      if (invalid.hasId(from) || invalid.hasId(to)) continue;

      int fromId = getNidFromId(from);
      int toId = getNidFromId(to);

      if ( fromId == -1 || toId == -1 ) continue;

      travel_Time[fromId][toId] = time;
    }

    twcij_calculate();
    assert(original == datanodes);
    assert(check_integrity());
  }


  /*!
   * \brief Load the travel time matrix from a text file and process the results.
   *
   * Reads \b infile and loads it into the travel time matrix and populates
   * any missing entries we=ith an approx distance. It also computes the TWC
   * matrix.
   *
   * \todo Add description of file format.
   *
   * \param[in] infile The file name to load.
   * \param[in] datanodes The bucket of data nodes that has already been loaded.
   * \param[in] invalid A bucket of invalid nodes found in the data nodes.
   */
  void loadAndProcess_distance(std::string infile, const Bucket &datanodes,
                                const Bucket &invalid) {
    assert(datanodes.size());
#ifdef DOVRPLOG
    DLOG(INFO) << "COMMANDLINE: loadAndProcess_distance";
#endif

    original.clear();
    original = datanodes;
    POS siz = original.size();

    std::ifstream in(infile.c_str());
    std::string line;

#ifdef OSRMCLIENT
    getAllHints();
#endif

    prepareTravelTime();

    int fromId;
    int toId;
    int from, to;
    double time;
    int cnt = 0;

    while ( getline(in, line) ) {
      cnt++;

      // skip comment lines
      if ( line[0] == '#' ) continue;

      std::istringstream buffer(line);
      buffer >> from;
      buffer >> to;
      buffer >> time;

      if ( invalid.hasId(from) || invalid.hasId(to) ) continue;

      fromId = getNidFromId(from);
      toId = getNidFromId(to);

      if ( fromId == -1 || toId == -1 ) continue;

      travel_Time[fromId][toId] = time;
    }

    in.close();

    twcij_calculate();
    assert(original == datanodes);
    assert(check_integrity());
  }

  /*! \brief Returns a constant reference to the travel time matrix. */
  const std::vector<std::vector<double> >& TravelTime() {
    return travel_Time;
  }

  /*! \brief Retrieves the internal node id (NID) from the users node id (ID)
   *
   * \param[in] id A user node identifier
   * \return The internal nid corresponding to id or -1 if not found.
   */
  UID getNidFromId(UID id) const {
    return original.getNidFromId(id);
  }


  static TWC<knode>* Instance() {
    if ( p_twc == NULL )  // Only allow one instance of class to be generated.
      p_twc = new TWC<knode>;
    return p_twc;
  }


 private:
  // constructors
  static TWC<knode>* p_twc;
  TWC() :z1Tot(0), z2Tot(0), emptiedTruck(false) {
    cleanUp();
  };
  TWC(const TWC &) {};
  TWC& operator=(const TWC &) {}



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
  double ajli(const knode &ni, const knode &nj) const {
    return ni.closes() + ni.serviceTime() + TravelTime(ni, nj);
  }

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
  double ajei(const knode &ni, const knode &nj) const {
    return ni.opens() + ni.serviceTime() + TravelTime(ni, nj);
  }


  /*!
   * \brief Compute TWC from node \b ni to node \b nj
   *
   * \param[in] ni From this node
   * \param[in] nj To this node
   * \return The TWC value traveling from node \b ni directly to \b nj
   */
  double twc_for_ij(const knode &ni, const knode &nj) const {
    double result;
    int i = ni.nid();
    int j = nj.nid();

    if ( travel_Time[i][j] == -1 ) return  VRP_MIN();

    if ( TravelTime( i, j ) == VRP_MAX() ) return  VRP_MIN();

    if ( (nj.closes() - ajei(ni, nj)) > 0 ) {
      result = std::min(ajli(ni, nj) , nj.closes())
               - std::max(ajei(ni, nj) , nj.opens());
    } else {
      result = VRP_MIN();
    }

    return result;
  }


  double getTwcij(UID i, UID j) const {  // this one makes twcij dynamical
    if  ( travel_Time[i][j] == -1 ) {
      TravelTime(i, j);
      twcij[i][j] = twc_for_ij(original[i], original[j]);
    }

    return twcij[i][j];
  }

  double setTwcij(UID i, UID j) const {
    twcij[i][j] = twc_for_ij(original[i], original[j]);
    return twcij[i][j];
  }


  /* public functions That are id based */


  /*!
   * \brief Compute all TWC values and populate the TWC matrix.
   */
  void twcij_calculate() {
    assert(original.size() == travel_Time.size());
    twcij.resize(original.size());

    for (int i = 0; i < original.size(); i++)
      twcij[i].resize(original.size());

    for ( int i = 0; i < original.size(); i++ ) {
      for ( int j = i; j < original.size(); j++ ) {
        twcij[i][j] = twc_for_ij(original[i], original[j]);
        twcij[j][i] = twc_for_ij(original[j], original[i]);
      }
    }
  }

  /*!
   * \brief Check that a TWC matrix entry exists for all original nodes.
   */
  bool check_integrity() const {
    assert(original.size() == twcij.size());

    if ( original.size() != twcij.size() ) return false;

    for ( int i = 0; i < original.size(); i++ ) {
      assert(twcij[i].size() == original.size());

      if ( twcij[i].size() != original.size() ) return false;
    }

    return true;
  }

 public:
  void set_TravelTime(UID fromId, UID toId, double time) {
     #ifdef VRPMINTRACE
     if (!travel_Time[fromId][toId] == time)
        DLOG(INFO) << "<travel_time[" << fromId << "][" << toId << "]="
        << travel_Time[fromId][toId] << " ---> " << time;
     #endif
     travel_Time[fromId][toId] = time;
  }
};  // end of class




template <class knode>
TWC<knode>  *TWC<knode>::p_twc = NULL;

#define twc TWC<Tweval>::Instance()


#endif  // SRC_BASECLASSES_TWC_H_