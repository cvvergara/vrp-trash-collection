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
#ifndef SRC_BASECLASSES_TWBUCKET_H_
#define SRC_BASECLASSES_TWBUCKET_H_
#pragma once

#include <deque>
#include <set>
#include <string>
#include <algorithm>


#include "baseClasses/logger.h"

#include "baseClasses/basictypes.h"
#include "baseClasses/vrp_assert.h"
#include "nodes/trashnode.h"
//#include "baseClasses/twc.h"

class Prob_trash;


/*! \class TwBucket
 * \brief A template class that provides deque like container with lots of additional functionality.
 *
 * TwBucket provides a set like container. It is used by \ref Twpath for
 * storage. It also provides several un-evaluated path operations. None of
 * the manipulation at this level is evaluated.
 *
 * The class provides:
 * - basic deque container like operations
 * - set operations
 * - node id based tools of the bucket/path
 * - position based tools of the bucket/path
 * - other tools
*/



using namespace vrptc ;
using namespace vrptc::nodes;

class TwBucket {
    friend class Prob_trash;
    friend class TWC;
 protected:

    std::deque<Trashnode> path;         ///< Defines the bucket container


    /*! \class compNode
     * \brief A node comparison class for ordering nodes in a set.
     */
    class compNode {
        public:
            bool operator()(const Trashnode &n1, const Trashnode &n2) const {
                return n1.nid() < n2.nid();
            }
    };



 private:
    /*!
      \param[in] prev:   prev node
     * \param[in] from:   from node
     * \param[in] middle: middle node
     * \param[in] to:     to node
     * \param[in] travelTimePrevFrom:   travel time from "prev" to "from" nodes
     */
    double  timePCN(const Trashnode &prev, const Trashnode &from, const Trashnode &middle,
            const Trashnode &to, double travelTimePrevFrom) const;

    /*! @name timePCN = time previous-current-next 
      simulates the following order of nodes in the path: 
      prev from middle to
     *
     * \return  the time that takes to depart from "from" and arrive to "to" passing thru "middle"
     *
     * \return  ttfm + serv(m) + ttmt = arrivalTime(next) - departureTime(prev) when passing thru curr
     * \return infinity              if there is a TWV (time window violation)
     *
     * use when:
     prev from   is a section of the path and "prev" is the previous node of "from"
     from from   there is no previous container of from

     A private function does all the work
     the other ones are variations on the parameters
     */
    ///@{

 public:
    /*! \brief  the 3 nodes belong to the path */
    double  timePCN(POS from, POS middle, POS to) const;

    /*! \brief  first 2 nodes belong to the path and the third doesnt */
    double timePCN(POS from, POS middle, const Trashnode &dump) const;

    /*! \brief  \from node belong to the path and \middle and \dump dont */
    double  timePCN(POS from, const Trashnode &middle, const Trashnode &dump) const;

    /*! \brief  simulates a replacement of a node in the bucket

      previous path:
      from from+1 from+2
      simulated path:
      from middle from+2
      */
    double timePCN(POS from, const Trashnode &middle) const;
    /*! \brief  simulates an insertion of two nodes a node at the end of the bucket
      namely node and dump 

      previous path:
      last dump
      simulated path:
      last node dump
      */
    double timePCN(const Trashnode &node, const Trashnode &dump) const;
    ///@}

    private:
    /*! @name TravelTime inline functions
      \brief useful inlines to fetch the travel time from Node to Node

      2 flavors:
      parameters are nodes
      parameters are node Nid (internal node id)
      No need to be within the bucket
      */
    ///@{
   
    double TravelTime(const Trashnode &from, const Trashnode &to) const;
    double TravelTime(const Trashnode &from, const Trashnode &middle,
            const Trashnode &to) const;
    double TravelTime(const Trashnode &prev, const Trashnode &from, const Trashnode &middle,
            const Trashnode &to) const;

    double TravelTime(UID i, UID j) const;
    double TravelTime(UID i, UID j, UID k) const;
    double TravelTime(UID i, UID j, UID k, UID l) const;
    ///@}

    public:
    /*! @name getDeltaTime
     * Simulate changes of times within the path
     \todo TODO check it returns a delta
     */

    /*!
     * \brief Simulate changes in travel times within the path
     *
     * Simulates the following change of travelTimes within the path
     * - dump
     * - dump node dump2
     *
     * and checks for TWV and returns infinity if the occur at:
     * - node
     * - dump2
     *
     * \return \f$ tt_dump,node + service(node) + tt_node,dump + service(dump) \f$
     * \return infinity when there is a TWV
     */
    // NOT USED
    double getDeltaTimeAfterDump(const Trashnode &dump, const Trashnode &node) const;


    /*!
     * \brief Compute the change in time when swapping nodes in pos1 and pos2
     *
     * Simulate swapping nodes in pos1 and pos2 in the path and compute
     * the delta time impact that would have on the path.
     *
     * \param[in] pos1 Position of the node to be swapped.
     * \param[in] pos2 Position of the other node to be swapped.
     * \return The delta time or infinity if if creates a path violation.
     */
    // NOT USED
    double getDeltaTimeSwap(POS pos1, POS pos2) const;


    /*!
     * \brief Compute the change in time when swapping node with the node at pos
     *
     * If the current path looks like prev -\> pos -\> pos1 then compute the
     * the change in time of swapping node for the node at pos, so the new
     * path would look like prev -\> node -\> pos1
     *
     * \param[in] node The node to evaluate if swapped with node at pos.
     * \param[in] pos The position of the node to be swapped.
     * \param[in] pos1 The next node following pos.
     * \return The change in cost or infinity if a TWV would be generated.
     */
    // NOT USED
    double getDeltaTime(const Trashnode &node, POS pos , POS pos1) const;



    /*!
     * \brief Compute the change in time of inserting node before pos in the path.
     *
     * Simulate inserting node before pos in the path and compute the resulting
     * change in time. No TW violations are checked.
     *
     * \param[in] node The node to be inserted in the simulation.
     * \param[in] pos The position before which the node will be inserted.
     * \return The change in travel time or infinity if the move is invalid.
     */
    double  getDeltaTime(const Trashnode &node, POS pos) const;



  /*!
   * \brief Compute the change in time when swapping node into pos in the path and do additional time violation checks.
   *
   * If the current path looks like prev -\> pos -\> pos1 then compute the
   * the change in time of swapping node for the node at pos, so the new
   * path would look like prev -\> node -\> pos1
   *
   * \param[in] node The node to evaluate if swapped with node at pos.
   * \param[in] pos The position of the node to be swapped.
   * \param[in] pos1 The next node following pos.
   * \return The change in cost or infinity if a TWV would be generated.
   */
  double getDeltaTimeTVcheck(const Trashnode &node, POS pos, POS pos1) const;



    /*!
     * \brief Check all nodes from pos to upto if adding delta would cause a violation.
     *
     * \param[in] delta The change in time to evaluate.
     * \param[in] pos The position to start evaluating.
     * \param[in] upto The position to stop evaluating.
     * \return true if delta would generate a time violation.
     */
    bool deltaGeneratesTVupTo(double delta, POS pos, POS upto) const;

    /*!
     * \brief Check all nodes forward from pos if adding delta would cause a violation.
     *
     * \param[in] delta The change in time to evaluate.
     * \param[in] pos The position to start evaluating.
     * \return true if delta would generate a time violation.
     */
    // NOT USED
    bool deltaGeneratesTV(double delta, POS pos) const;
    ///@}
#endif // 0

public:
// ---------------- other tools ----------------------------------

/*! \brief \returns the distance from a point to the segmnet (\b pos, \b pos+1)

  \warning assert(pos + 1 < path.size());
  \warning assert(path.size() > 1);

  \param[in] pos Position of start of segment.
  \param[in] node The node to compute the distance to.
  \return The shortest distance from node to line segment.
  */
double segmentDistanceToPoint(POS pos, const Trashnode &node) const;


#ifdef DOVRPLOG
/*! @name Dumping
  \brief Print the contents of the Twbucket
  */
///@{
/*! \brief Using id as node identifiers with title "Twbucket". */
void dumpid() const {dumpid("Twbucket");}


/*! \brief Using id as node identifiers with user defined title.

 * \param[in] title Title to print with the output of the Twbucket.
 */
void dumpid(const std::string &title) const;

/*! \brief Using nid as node identifiers with title "Twbucket".  */
void dump() const {dump("Twbucket");}

/*! \brief Using nid as node identifiers with title "Twbucket".  
 * \param[in] title Title to print with the output of the Twbucket.
 */
void dump(const std::string &title) const;
#endif
///@}


/*! @name To have or not to have

  \return true when it has
  */
///@{


/*! \brief has a node with the given \b id?
 *
 *  \param[in] id Uses the user's \b id
 */
bool hasId(int64_t id) const;


/*! \brief has a node?
 *
 *  \param[in] node
 */
bool hasNode(const Trashnode &node) const;

///@}


/*! @name Set operations based on the internal node id (nid) */
///@{
/*!  * \brief True when \b this buckets is equal to the \b other bucket. */
bool operator ==(const TwBucket &other) const;

/*! \brief Returns \b this  UNION \b other .  */
TwBucket  operator +(const TwBucket &other) const;

/*! \brief Returns \b this INTERSECTION \b other .  */
TwBucket operator *(const TwBucket &other) const;

/*! \brief Returns \b this DIFFERENCE \b other .  */
TwBucket operator -(const TwBucket &other) const;
///@}


/*! @name End of Path tools
  The end of the path is the \b last node of the path
  */
///@{
const Trashnode& last() const;

/*! \brief \returns the total travel time of the path.  */
double getTotTravelTime() const;

/*! \brief \returns the duration of the path.  */
double duration() const;

/*! \brief \returns the total wait time of the path.  */
double totWaitTime() const;

/*! \brief \returns the total service time of the path */
double totServiceTime() const;

/*! \brief \returns the total number of dump visits of the path. */
int dumpVisits() const;

/*! \brief \returns the departure time of the last node in the path. */
double departureTime() const;

/*! \brief \returns the total number of time window violations in the path.  */
int twvTot() const;

/*! \brief \returns the total number of capacity violations in the path. */
int cvTot() const;

/*! \brief \returns the total cargo at the end of the path. */
double cargo() const;

/*! \brief True when \b last node of path is feasable. */
bool feasable() const;

/*! \brief True when \b last node of path is feasable. */
bool feasable(double cargoLimit) const;

/*! \brief True when \b last node of path has time window violation. */
bool has_twv() const;

/*! \brief True when \b last node of path has capacity violation. */
bool has_cv(double cargoLimit) const;
///@}

// ---------- ID based tools  to NID tools ---------------------------

/*!
 * \brief Get the internal node id associated with the user id.
 * \param[in] id The user id for the node.
 * \return The internal node id or -1 if user id was not found.
 * \todo TODO  put it in twc
 */
size_t getNidFromId(int64_t id) const;


/*!  * \brief Get the position in the path where id is located.

 * \param[in] id The user id for the node.
 * \return The position in the path or -1 if it is not found.
 */
POS posFromId(int64_t id) const;


/*! @name  position
  Gets the position of node in the bucket
  */
///@{
#if 1
/*!
 * \brief Get the position of node in the path
 * \param[in] node A node object that we want to locate in the path
 * \return returns the position of node in the path or 0 if it's not found.
 * \warning, if the position is 0, the user has to make sure it belongs to the bucket
 */
POS pos(const Trashnode &node) const;

/*!
 * \brief Get the position of node id in the path
 * \param[in] nid The node id we want to locate in the path
 * \return The position of node id in the path or -1 if it's not found.
 */
POS pos(UID nid) const;
///@}


/*! @name  mutators

  \warning No evaluation is done
  */
///@{
/*! \brief  Both nodes are in the bucket

 * \param[in] i First node position to swap.
 * \param[in] j Second node position to swap.
 */
void swap(POS i, POS j);

/*! \brief  other node is in other bucket
 *
 * Swap nodes nodes between two buckets
 * - bucket1.swap( b1_pos, bucket2, b2_pos );
 *
 * The node in position b1_pos of bucket1 will be swapped with the node
 * in position b2_pos of bucket2.
 *
 * \param[in] b1_pos Position of node in bucket1
 * \param[in] bucket2 other bucket
 * \param[in] b2_pos Position of node in bucket2
 * \return true
 */
bool swap(POS b1_pos, TwBucket &bucket2, POS b2_pos);


/*!  \brief Move node fromi to the new position of toj in this TwBucket */
void move(int fromi, int toj);
///@}


/*! \brief Get a deque of nids that are in the path.

 * \return A deque of the nids in the path.
 */
std::deque<int> getpath() const;


/*! @name   deque like functions
  assertions added
  Please refer to cpp deque documentation
  \returns True when the operation was completed
  */
///@{
/*! \brief Insert node into deque
 * \param[in] atPos The position it should be inserted at
 * \param[in] node The node to insert
 */
bool insert(const Trashnode &node, POS atPos);

/*! \brief Insert node into deque
 * \param[in] atPos The position it should be inserted at
 * \param[in] node The node to insert
 */
bool insert(const TwBucket &nodes, POS atPos);

/*! \brief Erase the node from deque at location atPos
 * \param[in] atPos The position of the node to be erased.
 */
bool erase(POS atPos);


/* \brief Erase node from within the path.
 * \param[in] node The node to be erased.
 */
void erase(const Trashnode &node);


#ifdef USE
/*!  * \brief Erase all nodes between fromPos and toPos.

  \param[in] fromPos Position of the start of the range to be erased.
  \param[in] toPos Position of the last in the range to be erased.

  \warning Notice that the right side of the range is not included
  when  ( fromPos < toPos )  range erased: [fromPos,toPos)
  when  ( fromPos > toPos )  range erased: [toPos,fromPos)

  \warning If fromPos and toPos are reversed it will still erase the range.
  */
bool erase(POS fromPos, POS toPos);
#endif  //USE

bool push_back(const Trashnode &node);
bool push_front(const Trashnode &node);
auto begin() {return path.begin();}
auto end() {return path.begin();}
void pop_back() {path.pop_back();}
void pop_front() {path.pop_front();}
/*! \brief disables resizing to a larger bucket */
void resize(UINT newSize) {
    assert(newSize <= path.size());
    path.resize(newSize);
}
void clear() {path.clear();}
size_t max_size() const {return path.max_size();}
size_t size() const {return path.size();}
bool empty() const {return path.empty();}
std::deque<Trashnode>& Path() {return path;}
const std::deque<Trashnode>& Path() const  {return path;}
Trashnode& operator[](POS at);

const Trashnode& operator[] (POS at) const;
Trashnode& at(POS pos);
const Trashnode& at(POS pos) const;
Trashnode& front() {return path.front();}
const Trashnode& front() const {return path.front();}
Trashnode &back() {return path.back();}
const Trashnode& back() const {return path.back();}
///@}
};


#endif  // SRC_BASECLASSES_TWBUCKET_H_


