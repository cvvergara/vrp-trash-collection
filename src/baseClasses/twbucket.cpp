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
#if 0
    typedef typename std::deque<Trashnode>::iterator iterator;
    typedef typename std::deque<Trashnode>::reverse_iterator reverse_iterator;
    typedef typename
        std::deque<Trashnode>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::deque<Trashnode>::const_iterator const_iterator;
#endif
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
#if 0
    {
        if ( from.isNotCompatibleIJ(prev)
                || middle.isNotCompatibleIJ(prev)
                || to.isNotCompatibleIJ(prev)
                || middle.isNotCompatibleIJ(from)
                || to.isNotCompatibleIJ(from)
                || to.isNotCompatibleIJ(middle))
            return VRP_MAX();


        double travelTimePrevFromMiddle = TravelTime(prev, from , middle);
        double travelTimeFromMiddle = travelTimePrevFromMiddle - travelTimePrevFrom;
        double travelTimePrevFromMiddleTo = TravelTime(prev, from , middle, to);
        double travelTimeMiddleTo  = travelTimePrevFromMiddleTo  - travelTimePrevFromMiddle;

        double arrive_from = prev.departureTime() + travelTimePrevFrom;

        if (from.lateArrival(arrive_from)) return VRP_MAX();
        if (from.earlyArrival(arrive_from)) arrive_from = from.opens();

        double depart_from = arrive_from + from.serviceTime();
        double arrive_middle = arrive_from + from.serviceTime() + travelTimeFromMiddle;

        if ( middle.lateArrival(arrive_middle) ) return VRP_MAX();
        if ( middle.earlyArrival(arrive_middle) ) arrive_middle = middle.opens();

        double arrive_to = arrive_middle + middle.serviceTime() + travelTimeMiddleTo;

        if (to.lateArrival(arrive_to)) return VRP_MAX();
        if (to.earlyArrival(arrive_to)) arrive_to = to.opens();

        return arrive_to - depart_from;
    }
#endif

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
#if 0
    {
        assert(from < path.size());
        assert(middle < path.size());
        assert(to < path.size());
        assert(middle != from);
        assert(middle != to);

        if ( from == 0 )
            return timePCN(path[from], path[from], path[middle], path[to], 0);
        else
            return timePCN(path[from - 1], path[from], path[middle], path[to],
                    path[from].travelTime());
    }
#endif // 0

    /*! \brief  first 2 nodes belong to the path and the third doesnt */
    double timePCN(POS from, POS middle, const Trashnode &dump) const;
#if 0
    {
        assert(from < path.size());
        assert(middle < path.size());
        assert(middle != from);

        if ( from == 0 )
            return timePCN(path[from], path[from], path[middle], dump, 0);
        else
            return timePCN(path[from - 1], path[from], path[middle], dump,
                    path[from].travelTime());
    }
#endif // 0

    /*! \brief  \from node belong to the path and \middle and \dump dont */
    double  timePCN(POS from, const Trashnode &middle, const Trashnode &dump) const;
#if 0
    {
        assert(from < path.size());

        if ( from == 0 )
            return timePCN(path[from], path[from], middle, dump, 0);
        else
            return timePCN(path[from - 1], path[from], middle , dump,
                    path[from].travelTime());
    }
#endif // 0

    /*! \brief  simulates a replacement of a node in the bucket

      previous path:
      from from+1 from+2
      simulated path:
      from middle from+2
      */
    double timePCN(POS from, const Trashnode &middle) const;
#if 0
    {
        assert((from + 2) < path.size());

        if ( from == 0 )
            return timePCN(path[from], path[from], middle, path[from + 2], 0 );
        else
            return timePCN(path[from - 1], path[from], middle , path[from + 2],
                    path[from].travelTime());
    }
#endif // 0
    /*! \brief  simulates an insertion of two nodes a node at the end of the bucket
      namely node and dump 

      previous path:
      last dump
      simulated path:
      last node dump
      */
    double timePCN(const Trashnode &node, const Trashnode &dump) const;
#if 0
    {
        Trashnode last = path[path.size() - 1];
        return timePCN(path.size()-1, node, dump);
    }
#endif // 0
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
   
    inline double TravelTime(const Trashnode &from, const Trashnode &to) const;
#if 0
    {
        return twc->TravelTime(from.nid(), to.nid());
    }
#endif // 0
    double TravelTime(const Trashnode &from, const Trashnode &middle,
            const Trashnode &to) const;
#if 0
    {
        return TWC::Instance()->TravelTime(from.nid(), middle.nid() , to.nid());
    }
#endif // 0
    double TravelTime(const Trashnode &prev, const Trashnode &from, const Trashnode &middle,
            const Trashnode &to) const;
#if 0
    {
        return TWC::Instance()->TravelTime(prev.nid(), from.nid(), middle.nid() , to.nid());
    }
#endif // 0

    double TravelTime(UID i, UID j) const;
#if 0
    {
        return TWC::Instance()->TravelTime(i, j);
    }
#endif // 0
    double TravelTime(UID i, UID j, UID k) const;
#if 0
    {
        return TWC::Instance()->TravelTime(i, j, k);
    }
#endif // 0
    double TravelTime(UID i, UID j, UID k, UID l) const;
#if 0
    {
        return TWC::Instance()->TravelTime(i, j, k, l);
    }
#endif // 0
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
#if 0
    {
        double nodeArrival = dump.getDepartureTime() + TravelTime(dump, node);

        if ( node.lateArrival( nodeArrival) ) return VRP_MAX();

        if ( node.earlyArrival(nodeArrival) ) nodeArrival = node.opens();

        double dumpArrival =  nodeArrival + node.getServiceTime() +
            TravelTime(node, dump);

        if ( dump.lateArrival(dumpArrival) ) return VRP_MAX();

        if ( dump.earlyArrival(dumpArrival) ) dumpArrival = dump.opens();

        double delta = dumpArrival + dump.getServiceTime() -
            dump.getDepartureTime();
        return delta;
    }
#endif // 0


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
#if 0
    {
        assert(pos1 < path.size() - 1 && pos2 < path.size());
#ifdef TESTED
        DLOG(INFO) << "Entering twBucket::getDeltaTimeSwap()";
#endif

        double delta, oldTime, newTime;

        // pos1 is the lowest
        if ( pos1 > pos2 ) {int tmp = pos1; pos1 = pos2; pos2 = tmp;}

        // special case nidPrev nid1 nid2 nidNext
        if ( pos2 == pos1 + 1 ) {
            // nids invloved
            // in the same order the nodes are in the path
            int nidPrev, nid1, nid2, nidNext;
            nidPrev = path[pos1 - 1].nid();
            nid1 = path[pos1].nid();
            nid2 = path[pos2].nid();

            if ( pos2 != size() ) nidNext = path[pos2 + 1].nid();

            //                pos1-1  pos1  pos2  pos2+1
            // newpath looks: nidPrev nid2 nid1, nidNext

            // check for TWV
            if ( path[pos1 - 1].getDepartureTime()
                    + TravelTime[nidPrev][nid2] > path[pos2].closes() )
                return VRP_MAX();

            if ( path[pos1 - 1].getDepartureTime()
                    + TravelTime[nidPrev][nid2] + path[pos1].getServiceTime()
                    + TravelTime[nid2][nid1] > path[pos1].closes() )
                return VRP_MAX();

            // locally we are ok...  no capacity Violations
            // sum (services) remains constant
            if ( pos2 + 1 == size() ) {
                // newpath looks: nidPrev nid1 nid2,  DUMP in V
                //                pos1-1  pos1  pos2  pos2+1
                // newpath looks: nidPrev nid2 nid1,  DUMP in V
                // delta = new - old
                oldTime = path[pos2].getDepartureTime();
                newTime = path[pos1 - 1].getDepartureTime()
                    + TravelTime[nidPrev][nid2] + TravelTime[nid2][nid1];
                delta = oldTime - newTime;
            } else {
                // oldpath looks: nidPrev nid1 nid2,  nidNext
                //                pos1-1  pos1  pos2  pos2+1
                // newpath looks: nidPrev nid2 nid1,  nidNext

                oldTime = path[pos2 + 1].getArrivalTime();
                newTime = path[pos1 - 1].getDepartureTime()
                    + TravelTime[nidPrev][nid2]
                    + TravelTime[nid2][nid1]
                    + TravelTime[nid1][nidNext];
                delta   =  oldTime - newTime;;
            }

            // check for TWV
            if ( pos2 + 1 < size() && deltaGeneratesTV( delta, pos2 + 1 ) )
                return VRP_MAX();

            return delta;
            // end of case when one node is after the other
        }

        // oldpath looks: nidPrev1 nid1 nidnext1    nidPrev2    nid2,  nidNext2
        //                pos1-1  pos1  pos1+1      pos2-1      pos2    pos2+1
        // newpath looks: nidPrev1 nid2 nidnext1    nidPrev2,   nid1,  nidNext2
        double delta1 = getDeltaTime(path[pos2], pos1, pos1 + 1);
        double delta2 = getDeltaTime(path[pos1], pos2, pos2 + 1);

        // check if TWV is generated
        if ((delta1 == VRP_MAX()) || (delta2 == VRP_MAX())) return VRP_MAX();

        if ( deltaGeneratesTVupTo(delta1, pos1, pos2 - 1) ) return VRP_MAX();

        if ( deltaGeneratesTV(delta1 + delta2, pos2 + 1) ) return VRP_MAX();

        // simple checks for cargo Violation
        if ((path[pos1].getdemand() == path[pos2].getdemand())
                && !path[size() - 1].hascv())
            return delta1 + delta2;

        // check for cargo Violation Missing
        // if there is no dump  on the path: return  delta1 + delta2

        // if the share the same dump  return delta1 +delta2

        return delta1 + delta2;
    }
#endif // 0


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
#if 0
    {
        assert(pos1 <= path.size());
        assert(pos > 0 && pos1 == (pos + 1));

        if ( pos == 0 && path[pos].isdepot() ) return VRP_MAX();

        int nid = path[pos].nid();
        int prev = path[pos - 1].nid();

        if ( path[pos - 1].getDepartureTime()
                + TravelTime[prev][node.nid()] > node.closes() )
            return VRP_MAX();

        if ( pos1 == size() )
            return  TravelTime[prev][node.nid()]
                + node.getServiceTime()
                - (path[pos].getDepartureTime()
                        - path[pos - 1].getDepartureTime());

        int next = path[pos1].nid();

        double delta  =  TravelTime[prev][node.nid()]
            + node.getServiceTime()
            + TravelTime[node.nid()][next]
            - (path[pos1].getArrivalTime()
                    - path[pos - 1].getDepartureTime());
        return delta;
    }
#endif // 0

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
#if 0
    double getDeltaTimeTVcheck(const Trashnode &node, POS pos, POS pos1) const;
    {
        assert(pos1 <= path.size());
        assert(pos > 0 && pos1 == (pos + 1));

        double delta = getDeltaTime(node, pos, pos1);

        if ((path[pos - 1].getDepartureTime() + TravelTime[ path[pos - 1].nid() ] [node.nid() ])
                > node.closes()) 
            return VRP_MAX();

        if (pos == size()) return delta;

        if (deltaGeneratesTV( delta, pos1 )) return VRP_MAX();

        return delta;
    }
#endif // 0


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
#if 0
    {
        assert(pos < path.size());

        if ( pos == 0 || path[pos].isDepot() ) return VRP_MAX();

        int nid = path[pos].nid();
        int prev = path[pos - 1].nid();

        if ( pos == size() )
            return  TravelTime(prev, node.nid()) + node.getServiceTime();

        return TravelTime(prev, node.nid())
            + node.getServiceTime()
            + TravelTime(node.nid(), nid)
            - TravelTime(prev, nid);
    }
#endif // 0


    /*!
     * \brief Compute the change in time of inserting node before pos in the path and check for TW violations..
     *
     * Simulate inserting node before pos in the path and compute the resulting
     * change in time and check for TW violations.
     *
     * \param[in] node The node to be inserted in the simulation.
     * \param[in] pos The position before which the node will be inserted.
     * \return The change in travel time or infinity if the move is invalid.
     */
    // NOT USED
#if 0
    double  getDeltaTimeTVcheck(const Trashnode &node, POS pos) const;
    {
        assert(pos <= path.size());
        assert(pos > 0);

        double delta = getDeltaTime(node, pos);

        // check for TWV
        if ( path[pos - 1].getDepartureTime()
                + TravelTime[ path[pos - 1].nid() ][ node.nid()]
                > node.closes() ) return VRP_MAX();

        if ( pos == size() ) return delta;

        // check for TWV
        if ( deltaGeneratesTV( delta, pos ) ) return VRP_MAX();

        return delta;

    double delta  =  TravelTime[prev][node.nid()]
                     + node.getServiceTime()
                     + TravelTime[node.nid()][next]
                     - (path[pos1].getArrivalTime()
                        - path[pos - 1].getDepartureTime());
    return delta;
  }
#endif // 0

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
#if 0
{
    assert(pos1 <= path.size());
    assert(pos > 0 && pos1 == (pos + 1));

    double delta = getDeltaTime(node, pos, pos1);

    if ((path[pos - 1].getDepartureTime() + TravelTime[ path[pos - 1].nid() ] [node.nid() ])
         > node.closes())
      return VRP_MAX();

    if (pos == size()) return delta;

    if (deltaGeneratesTV( delta, pos1 )) return VRP_MAX();

    return delta;
  }
#endif // 0



    /*!
     * \brief Check all nodes from pos to upto if adding delta would cause a violation.
     *
     * \param[in] delta The change in time to evaluate.
     * \param[in] pos The position to start evaluating.
     * \param[in] upto The position to stop evaluating.
     * \return true if delta would generate a time violation.
     */
    bool deltaGeneratesTVupTo(double delta, POS pos, POS upto) const;
#if 0
{
        assert(pos < path.size() && upto < size() && pos <= upto);
        bool flag = false;

        // checking if the delta affects any node after it
        for ( int i = pos; i <= upto; i++ )
            if ( path[i].getArrivalTime() + delta > path[i].closes() ) {
                flag = true;
                break;
            }

        return flag;
    }
#endif // 0

    /*!
     * \brief Check all nodes forward from pos if adding delta would cause a violation.
     *
     * \param[in] delta The change in time to evaluate.
     * \param[in] pos The position to start evaluating.
     * \return true if delta would generate a time violation.
     */
    // NOT USED
    bool deltaGeneratesTV(double delta, POS pos) const;
#if 0
{
        if (pos < size())
            return  deltaGeneratesTVupTo(delta, pos, size() - 1);
        else
            return false;
    }
#endif // 0
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
#if 0
{
    assert(path.size() > 1);
    assert(pos + 1 < path.size());
    return node.distanceToSegment(path[pos], path[pos + 1]);
}
#endif // 0


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
#if 0
{
    std::stringstream ss;
    ss << title;
    for (const auto e : path) ss << " " << e.id();
    DLOG(INFO) << ss.str();
}
#endif // 0

/*! \brief Using nid as node identifiers with title "Twbucket".  */
void dump() const {dump("Twbucket");}

/*! \brief Using nid as node identifiers with title "Twbucket".  
 * \param[in] title Title to print with the output of the Twbucket.
 */
void dump(const std::string &title) const;
#if 0
{
    DLOG(INFO) << title;
    for (const auto e : path) e.dump();
    DLOG(INFO) << " <----- end \n";
}
#endif // 0
#endif
///@}


/*! @name hasId

  \return true if a node with the same id is in the bucket.
  */
///@{
/*! \brief \param[in] node uses the \b id of the node*/
bool hasId(const Trashnode &node) const;
#if 0
{
    return hasId(node.id());
}
#endif // 0
/*! \brief \param[in] id Uses the \b id */

bool hasId(int64_t id) const;
#if 0
{
    return std::find_if(path.begin(), path.end(),
            [&id](const auto &item) {
        return item.id() == id;
        }) != path.end();


#if 0
    const_reverse_iterator rit = path.rbegin();

    for (const_iterator it = path.begin(); it != path.end() ; it++, ++rit) {
        if ( it->id() == id ) return true;
        if ( rit->id() == id ) return true;
    }

    return false;
#endif
}
#endif // 0
///@}


/*! @name hasNId

  \return true if a node with the same nid was found in the bucket.
  */
///@{
/*! \brief \param[in] node uses the \b nid of the node*/
bool hasNid(const Trashnode &node) const;
#if 0
{
    return hasNid(node.nid());
}
#endif // 0
/*! \brief \param[in] id Uses the \b nid */
bool hasNid(UID nid) const;
#if 0
{

    return !(
            std::find_if(path.begin(), path.end(),
                [&nid] (const Trashnode &e) 
                {return e.nid() == nid;})
            ==  path.end());

#if 0
    const_reverse_iterator rit = path.rbegin();

    for (const_iterator it = path.begin(); it != path.end() ; it++, ++rit) {
        if ( it->nid() == nid ) return true;
        if ( rit->nid() == nid ) return true;
    }
    return false;
#endif
}
#endif // 0
///@}


/*! @name Set operations based on the internal node id (nid) */
///@{
/*!  * \brief True when \b this buckets is equal to the \b other bucket. */
bool operator ==(const TwBucket &other) const;
#if 0
{
    if ( size() != other.size() ) return false;

    if ( size() == 0 && other.size() == 0 ) return true;

    if ( ((*this) - other).size() != 0 ) return false;

    if ( (other - (*this)).size() != 0 ) return false;

    return true;
}
#endif // 0


/*! \brief Returns \b this  UNION \b other .  */
TwBucket  operator +(const TwBucket &other) const;
#if 0
{
    std::set<Trashnode, compNode> a;
    a.insert(path.begin(), path.end());
    a.insert(other.path.begin(), other.path.end());
    TwBucket b;
    b.path.insert(b.path.begin(), a.begin(), a.end());
    return b;
}
#endif // 0

/*! \brief Returns \b this INTERSECTION \b other .  */
TwBucket operator *(const TwBucket &other) const;
#if 0
{
    std::set<Trashnode, compNode> s1;
    std::set<Trashnode, compNode> s2;
    std::set<Trashnode, compNode> intersect;
    s1.insert(path.begin(), path.end());
    s2.insert(other.path.begin(), other.path.end());
    std::set_intersection(s1.begin(), s1.end(), s2.begin(), s2.end(),
            std::inserter(intersect, intersect.begin()));
    TwBucket b;
    b.path.insert(b.path.begin(), intersect.begin(), intersect.end());
    return b;
}
#endif // 0

/*! \brief Returns \b this DIFFERENCE \b other .  */
TwBucket operator -(const TwBucket &other) const;
#if 0
{
    std::set<Trashnode, compNode> s1;
    std::set<Trashnode, compNode> s2;
    std::set<Trashnode, compNode> diff;
    s1.insert(path.begin(), path.end());
    s2.insert(other.path.begin(), other.path.end());
    std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(),
            std::inserter(diff, diff.begin()));
    TwBucket b;
    b.path.insert(b.path.begin(), diff.begin(), diff.end());
    return b;
}
#endif // 0
///@}


/*! @name End of Path tools
  The end of the path is the \b last node of the path
  */
///@{
const Trashnode& last() const;
#if 0
{
    assert(size());
    return  path[size() - 1];
}
#endif // 0

/*! \brief \returns the total travel time of the path.  */
double getTotTravelTime() const;
#if 0
{
    assert(size());
    return last().totTravelTime();
}
#endif // 0

/*! \brief \returns the duration of the path.  */
double duration() const;
#if 0
{
    assert(size());
    return last().duration();
}
#endif // 0

/*! \brief \returns the total wait time of the path.  */
double totWaitTime() const;
#if 0
{
    assert(size());
    return last().totWaitTime();
}
#endif // 0

/*! \brief \returns the total service time of the path */
double totServiceTime() const;
#if 0
{
    assert(size());
    return last().totServiceTime();
}
#endif // 0

/*! \brief \returns the total number of dump visits of the path. */
int dumpVisits() const;
#if 0
{
    assert(size());
    return last().dumpVisits();
}
#endif // 0

/*! \brief \returns the departure time of the last node in the path. */
double departureTime() const;
#if 0
{
    assert(size());
    return last().departureTime();
}
#endif // 0

/*! \brief \returns the total number of time window violations in the path.  */
int twvTot() const;
#if 0
{
    assert(size());
    return last().twvTot();
}
#endif // 0

/*! \brief \returns the total number of capacity violations in the path. */
int cvTot() const;
#if 0
{
    assert(size());
    return last().cvTot();
}
#endif // 0

/*! \brief \returns the total cargo at the end of the path. */
double cargo() const;
#if 0
{
    assert(size());
    return last().cargo();
}
#endif // 0

/*! \brief True when \b last node of path is feasable. */
bool feasable() const;
#if 0
{
    assert(size());
    return last().feasable();
}
#endif // 0

/*! \brief True when \b last node of path is feasable. */
bool feasable(double cargoLimit) const;
#if 0
{
    assert(size());
    return last().feasable(cargoLimit);
}
#endif // 0

/*! \brief True when \b last node of path has time window violation. */
bool has_twv() const;
#if 0
{
    assert(size());
    return last().has_twv();
}
#endif // 0

/*! \brief True when \b last node of path has capacity violation. */
bool has_cv(double cargoLimit) const;
#if 0
{
    assert(size());
    return last().has_cv(cargoLimit);
}
#endif // 0
///@}

// ---------- ID based tools  to NID tools ---------------------------

/*!
 * \brief Get the internal node id associated with the user id.
 * \param[in] id The user id for the node.
 * \return The internal node id or -1 if user id was not found.
 * \todo TODO  put it in twc
 */
UID getNidFromId(int64_t id) const;
#if 0
{

    return std::find_if(path.begin(), path.end(),
            [&id](const auto &item) {
            return item.id() == id;
            })->nid();

#if 0
    const_reverse_iterator rit = path.rbegin();

    for (const_iterator it = path.begin(); it != path.end() ; it++, ++rit) {
        if ( it->id() == id ) return it->nid();

        if ( rit->id() == id ) return rit->nid();
    }

    return 0;
#endif
}
#endif // 0


/*!  * \brief Get the position in the path where id is located.

 * \param[in] id The user id for the node.
 * \return The position in the path or -1 if it is not found.
 */
POS posFromId(int64_t id) const;
#if 0
{
    return std::find_if(path.begin(), path.end(),
            [&id](const auto &item) {
            return item.id() == id;
            }) - path.begin();

#if 0
    for ( const_iterator it = path.begin(); it != path.end() ; it++ ) {
        if ( it->id() == id ) return POS(it - path.begin());
    }

    return 0;
#endif
}
#endif // 0


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
#if 0
{return pos(node.nid());}
#endif // 0

/*!
 * \brief Get the position of node id in the path
 * \param[in] nid The node id we want to locate in the path
 * \return The position of node id in the path or -1 if it's not found.
 */
POS pos(UID nid) const;
#if 0
{
    return std::find_if(path.begin(), path.end(),
            [&nid](const auto &item) {
            return item.nid() == nid;
            }) - path.begin();
#if 0
    for ( const_iterator it = path.begin(); it != path.end() ; it++ ) {
        if ( it->nid() == nid ) return POS( it - path.begin() );
    }
    return 0;
#endif
}
#endif  // USE
///@}


/*! @name  mutators

  \warning No evaluation is done
  */
///@{
/*! \brief  Both nodes are in the bucket

 * \param[in] i First node position to swap.
 * \param[in] j Second node position to swap.
 */
void swap(POS i, POS j) {
    std::iter_swap(this->path.begin() + i, this->path.begin() + j);
}

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
#if 0
{
    assert(b1_pos < size() && b2_pos < bucket2.size());
    std::iter_swap(path.begin() + b1_pos, bucket2.path.begin() + b2_pos);
    return true;
}
#endif // 0


/*!  \brief Move node fromi to the new position of toj in this TwBucket */
void move(int fromi, int toj);
#if 0
{
    if ( fromi == toj ) return;

    if ( fromi < toj ) {
        insert(this->path[fromi], toj + 1);
        erase(fromi);
    } else {
        insert(this->path[fromi], toj);
        erase(fromi + 1);
    }
}
#endif // 0
///@}


/*! \brief Get a deque of nids that are in the path.

 * \return A deque of the nids in the path.
 */
std::deque<int> getpath() const;
#if 0
{
    std::deque<int> p;

    for (const auto e: path)
        p.push_back(e.nid());

    return p;
}
#endif // 0


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
#if 0
{
    assert(atPos <= path.size());
    path.insert(path.begin() + atPos, node);
    return true;
}
#endif // 0

/*! \brief Insert node into deque
 * \param[in] atPos The position it should be inserted at
 * \param[in] node The node to insert
 */
bool insert(const TwBucket &nodes, POS atPos);
#if 0
{
    assert(atPos <= path.size());
    for (UINT i = 0; i < nodes.size(); i++) {
        path.insert(path.begin() + atPos + i, nodes[i]);
    } 
    return true;
}

#endif // 0

/*! \brief Erase the node from deque at location atPos
 * \param[in] atPos The position of the node to be erased.
 */
bool erase(POS atPos);
#if 0
{
    assert(atPos < path.size());
    path.erase(path.begin() + atPos);
    return true;
}
#endif // 0


/* \brief Erase node from within the path.
 * \param[in] node The node to be erased.
 */
void erase(const Trashnode &node);
#if 0
{

    path.erase(
            std::remove_if(
                path.begin(),
                path.end(),
                [&node](const auto &item) {
                return item.id() == node.id();
                }),
            path.end()
            );
#if 0
    if (!hasNid(node)) return false;
    int atPos = pos(node.nid());
    assert(atPos < path.size());
    path.erase(path.begin() + atPos);
    return true;
#endif
}
#endif // 0


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
#if 0
{
    assert(fromPos < path.size());
    assert(toPos < path.size());

    if ( fromPos == toPos ) {
        path.erase(fromPos);
    } else {
        if ( fromPos < toPos ) { // [fromPos,toPos)
            path.erase(path.begin() + fromPos, path.begin() + toPos);
        } else { // [toPos,fromPos)
            path.erase(path.begin() + toPos, path.begin() + fromPos);
        }
    }
}
#endif // 0
#endif  //USE

bool push_back(const Trashnode &node);
#if 0
{
    path.push_back(node);
    return true;
}
#endif // 0
bool push_front(const Trashnode &node);
#if 0
{
    path.push_front(node);
    return true;
}
#endif // 0
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
Trashnode& operator[](POS at) {
    assert(at < path.size());
    return path[at];
}
const Trashnode& operator[] (POS at) const;
#if 0
{
    assert(at < path.size());
    return path[at];
}
#endif // 0
Trashnode& at(POS pos);
#if 0
{
    assert(pos < path.size());
    return path.at(pos);
}
#endif // 0
const Trashnode& at(POS pos) const;
#if 0
{
    assert(pos < path.size());
    return path.at( pos );
}
#endif // 0
Trashnode& front() {return path.front();}
const Trashnode& front() const {return path.front();}
Trashnode &back() {return path.back();}
const Trashnode& back() const {return path.back();}
///@}
};


#endif  // SRC_BASECLASSES_TWBUCKET_H_


