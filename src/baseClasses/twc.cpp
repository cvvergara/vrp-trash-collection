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

#include "baseClasses/twc.h"
#include "baseClasses/logger.h"
#include "baseClasses/stats.h"

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




using namespace vrptc;
using namespace vrptc::nodes;




/*! \brief cleans all the tables, leaving them blank for a next execution */
void TWC::cleanUp() {
    original.clear();
    twcij.clear();
    travel_Time.clear();
    travel_time_onTrip.clear();
    nodes_onTrip.clear();
    process_order.clear();
    process_order_far.clear();
    mPhantomNodes.clear();
}



#if 1
void TWC::initializeTravelTime() {
#ifdef VRPMINTRACE
    DLOG(INFO) << "started initializeTravelTime";
#endif
    for (UINT i = 0; i < travel_Time.size(); ++i) {
        for (UINT j = 0; i < travel_Time.size(); ++i)
            if (i != j && travel_time_onTrip[i][j] == 0)
                travel_Time[i][j] = -1;
            else {
                travel_Time[i][j] = travel_time_onTrip[i][j];
            }
    }

#ifdef VRPMINTRACE
    DLOG(INFO) << "Updated travel_Time";
    dump_travel_Time();
#endif

#ifdef VRPMINTRACE
    DLOG(INFO) << "ended initializeTravelTime";
#endif
}
#endif



void
TWC::getProcessOrder() {
    STATS_INC("getProcessOrder");
    DLOG(INFO) << "--> getProcessOrder";

    for (const auto &source : original.path) {
        for (const auto &sink : original.path) {

            /*
             * compulsory are already done
             */
            if (!source.isPickup() || !sink.isPickup()) continue;

#ifndef NDEBUG
            if (source.nid() == sink.nid()) {
                assert(travel_time(source, sink) == 0);
                assert(travel_time_onTrip[source.nid()][sink.nid()] == 0);
            }
#endif


            /*
             * processing will be symetrical
             */
            if (sink.nid() >= source.nid()) continue;

            if (original.size() < 500 && travel_Time[source.nid()][sink.nid()] == -1) {
                /*
                 * For small iamount of containers
                 * No original time given is considered far
                 * And will be processed first
                 */
                process_order_far.insert(
                        std::make_pair(std::make_pair(source.nid(), sink.nid()),
                            source.distanceToSquared(sink)));
            } else if (travel_Time[source.nid()][sink.nid()] != -1) {
                /*
                 * time given is considered near
                 */
                process_order.insert(
                        std::make_pair(std::make_pair(source.nid(), sink.nid()),
                            travel_Time[source.nid()][sink.nid()]));
            }
        }
    }
    DLOG(INFO) << "<-- getProcessOrder";
}




// TODO
void  
TWC::findBestToNodeHasMoreNodesOnPath(
        const TwBucket &assigned,
        const TwBucket &unassigned,
        UINT From, UINT &bestTo, TwBucket &subPath) const
{
    assert(unassigned.size() != 0);
    assert(From < original.size());
    subPath.clear();
    bestTo = unassigned[0].nid();
    size_t max = 0;

    for (UINT i =0; i< unassigned.size() - 1; ++i) {
        auto to = unassigned[i].nid();
        if (From == to) continue;
        assert(to < original.size());
        auto actual = actualCantNodesOnTrip(From, to, assigned);

        if ((actual > max)) {
            max = actual;
            bestTo = to;
            assert(bestTo < original.size());
        }

    }
    // build the whole subpath, nodes on trip need to be in unassigned set
#ifdef VRPMAXTRACE
    for (unsigned int i = 0; i < nodes_onTrip[From][bestTo].size(); ++i) {
        DLOG(INFO) << nodes_onTrip[From][bestTo][i];
    }
#endif

    subPath = actualNodesOnTrip(From, bestTo, assigned);
    subPath.push_back(original[bestTo]);
}

float8 
TWC::getTimeOverNodesCount(
        const Trashnode &fromNode, const Trashnode &toNode,
        const TwBucket &assigned,
        const TwBucket &subPath) const
{
    UINT from = fromNode.nid();
    UINT to = toNode.nid();
    assert(from != to);
    assert(subPath.size());
    UINT beginS = subPath[0].nid();
    UINT endS = subPath[subPath.size()-1].nid();
    float8 totalNodes = actualCantNodesOnTrip(from, beginS, assigned)
        + actualCantNodesOnTrip(beginS, endS, assigned)
        + actualCantNodesOnTrip(endS, to, assigned);
    float8 totalTime = travel_time_onTrip[from][beginS]
        + travel_time_onTrip[beginS][endS]
        + travel_time_onTrip[endS][to];
    float8 result = totalTime/(totalNodes + 2);
    return result;
}

double 
TWC::getTimeOnTrip(const Trashnode from, const Trashnode middle, const Trashnode to) {
    return travel_time_onTrip[from.nid()][middle.nid()]
        + travel_time_onTrip[middle.nid()][to.nid()];
}


void  
TWC::findBestFromNodeHasMoreNodesOnPath(
        const TwBucket &assigned,
        const TwBucket &unassigned,
        UINT &bestFrom, UINT To, TwBucket &subPath) const
{
    assert(unassigned.size() != 0);
    assert(To < original.size());
    subPath.clear();
    bestFrom = unassigned[0].nid();
    size_t max = 0;

    for (UINT i =0; i< unassigned.size() - 1; ++i) {
        UINT from = unassigned[i].nid();
        if (from == To) continue;
        assert(from < original.size());
        auto actual = actualCantNodesOnTrip(from, To, assigned);

        if ((actual > max)) {
            max = actual;
            bestFrom = from;
            assert(bestFrom < original.size());
        }

    }
    // build the whole subpath, nodes on trip need to be in unassigned set
#ifdef VRPMAXTRACE
    for (unsigned int i = 0; i < nodes_onTrip[bestFrom][To].size(); ++i) {
        DLOG(INFO) << nodes_onTrip[bestFrom][To][i];
    }
#endif

    subPath = actualNodesOnTrip(bestFrom, To, assigned);
    subPath.push_front(original[bestFrom]);
}



bool  
TWC::findPairNodesHasMoreNodesOnPath(
        const TwBucket &assigned, const TwBucket &unassigned,
        UINT &bestFrom, UINT &bestTo, TwBucket &subPath) const
{
    STATS_INC("findPairNodesHasMoreNodesOnPath");
    assert(unassigned.size() > 1);
    subPath.clear();
    bestFrom = unassigned[0].nid();
    bestTo = unassigned[1].nid();
    UINT actual = 0;
    UINT max = 0;

    for (UINT i =0; i< unassigned.size() - 1; ++i) {
        for (UINT j = 0; j < unassigned.size(); ++j) {
            if (i == j) continue;
            auto from = unassigned[i].nid();
            auto to = unassigned[j].nid();
            assert(from < original.size());
            assert(to < original.size());
            actual = actualCantNodesOnTrip(from, to, assigned);

            if ((actual > max)) {
                max = actual;
                bestFrom = from;
                bestTo = to;
                assert(bestFrom < original.size());
                assert(bestTo < original.size());
            }
        }
    }
    assert(bestFrom != bestTo);
    // build the whole subpath, nodes on trip need to be in unassigned set
#ifdef VRPMAXTRACE
    for (unsigned int i = 0; i < nodes_onTrip[bestFrom][bestTo].size(); ++i) {
        DLOG(INFO) << nodes_onTrip[bestFrom][bestTo][i];
    }
#endif

    subPath.push_back(original[bestFrom]);
    for (unsigned int i = 0; i < nodes_onTrip[bestFrom][bestTo].size(); ++i) {
        auto nid = nodes_onTrip[bestFrom][bestTo][i];
        assert(nid < original.size());
        if (assigned.hasNode(original[nid])) continue;
        subPath.push_back(original[nid]);
    }
    subPath.push_back(original[bestTo]);
    return true;
}








bool  
TWC::findNodeHasMoreNodesOnPath(const TwBucket &trip,
        const TwBucket &assigned, const TwBucket &unassigned,
        const Trashnode &dumpSite, UINT &bestNode, UINT &bestPos, TwBucket &subPath) const
{
    STATS_INC("findNodeHasMoreNodesOnPath");
    assert(unassigned.size() != 0);
    subPath.clear();
    TwBucket l_trip = trip;
    l_trip.push_back(dumpSite);
    bestNode = unassigned[0].nid();
    bestPos = 1;
    auto bestPrevNode = l_trip[0].nid();
    auto bestNextNode = l_trip[1].nid();
    size_t max = 0;



    for (UINT i =0; i< l_trip.size() - 1; ++i) {
        for (unsigned int j = 0; j < unassigned.size(); ++j) {
            auto from = l_trip[i].nid();
            auto middle = unassigned[j].nid();
            auto to = l_trip[i+1].nid();
            assert(from < original.size());
            assert(middle < original.size());
            assert(to < original.size());
            auto actual = actualCantNodesOnTrip(from, middle, assigned)
                + actualCantNodesOnTrip(middle, to, assigned) + 1;
            if (actual > max) {
                max = actual;
                bestPrevNode = from;
                bestNode = middle;
                bestNextNode = to;
                bestPos = i + 1;
                assert(bestPrevNode < original.size());
                assert(bestNode < original.size());
                assert(bestNextNode < original.size());
                assert(bestPos < l_trip.size());
            }
        }
    }
    // build the whole subpath, nodes on trip need to be in unassigned set
#ifdef VRPMAXTRACE
    for (unsigned int i = 0; i < nodes_onTrip[bestPrevNode][bestNode].size(); ++i) {
        DLOG(INFO) << nodes_onTrip[bestPrevNode][bestNode][i];
    }
#endif

    for (unsigned int i = 0; i < nodes_onTrip[bestPrevNode][bestNode].size(); ++i) {
        auto nid = nodes_onTrip[bestPrevNode][bestNode][i];
        assert(nid < original.size());
        if (assigned.hasNode(original[nid])) continue;
        subPath.push_back(original[nid]);
    }
    subPath.push_back(original[bestNode]);
    for (unsigned int i = 0; i < nodes_onTrip[bestNode][bestNextNode].size(); ++i) {
        auto nid = nodes_onTrip[bestNode][bestNextNode][i];
        if (assigned.hasNode(original[nid])) continue;
        assert(nid < original.size());
        subPath.push_back(original[nid]);
    }

    TwBucket l_subPath = subPath;
    TwBucket m_assigned = assigned;
    l_subPath = l_subPath - m_assigned;
    for (UINT i = 0; i < subPath.size(); ++i)
        if (!l_subPath.hasNode(subPath[i])) {
            subPath.erase(i);
            --i;
        }
#ifdef VRPMAXTRACE
    l_trip.dumpid("trip");
    subPath.dumpid("subpath");
    DLOG(INFO) << "bespPos " << bestPos;
#endif
    assert(subPath.size() != 0);
    return true;
}

size_t 
TWC::actualCantNodesOnTrip(UINT from, UINT to, const TwBucket &assigned) const {
    STATS_INC("actualCantNodesOnTrip");
    assert(from < original.size());
    assert(to < original.size());
    TwBucket subPath;
    TwBucket m_assigned = assigned;
    for (unsigned int i = 0; i < nodes_onTrip[from][to].size(); ++i) {
        UINT nid = nodes_onTrip[from][to][i];
        if (assigned.hasNode(original[nid])) continue;
        subPath.push_back(original[nid]);
    }
    subPath = subPath - m_assigned;
    assert(subPath.size() <= nodes_onTrip[from][to].size());
    return subPath.size();
}

TwBucket 
TWC::actualNodesOnTrip(UINT from, UINT to, const TwBucket &assigned) const {
    assert(from < original.size());
    assert(to < original.size());
    TwBucket subPath;
    TwBucket m_assigned = assigned;
    for (unsigned int i = 0; i < nodes_onTrip[from][to].size(); ++i) {
        UINT nid = nodes_onTrip[from][to][i];
        if (assigned.hasNode(original[nid])) continue;
        subPath.push_back(original[nid]);
    }
    TwBucket l_subPath = subPath;
    l_subPath = l_subPath - m_assigned;
    for (UINT i = 0; i < subPath.size(); ++i)
        if (!l_subPath.hasNode(subPath[i])) {
            subPath.erase(i);
            --i;
        }
    assert(subPath.size() <= nodes_onTrip[from][to].size());
    return subPath;
}


void 
TWC::fill_travel_time_onTrip() {
#ifdef VRPMINTRACE
    DLOG(INFO) << "strarted fill_travel_time_onTrip";
#endif

#ifdef VRPMINTRACE
    DLOG(INFO) << "fill_travel_time_onTrip to be checked: " << original.size();
#endif

    travel_time_onTrip.resize(original.size());
    nodes_onTrip.resize(original.size());

    for (auto &row : travel_time_onTrip) row.resize(original.size());
    for (auto &row : nodes_onTrip) row.resize(original.size());


    compulsory_fill();

    getProcessOrder();


    DLOG(INFO) << "process_order_far size =" << process_order_far.size();
    DLOG(INFO) << "process_order_near size =" << process_order.size();
    fill_travel_time_onTrip_work(process_order_far);
    fill_travel_time_onTrip_work(process_order);

    STATS_PRINT("AFTER TWC::fill_travel_time_onTrip");
    assert(true == false);
}


void 
TWC::fill_travel_time_onTrip_work(
        Ordering &process_order) {
#ifdef VRPMINTRACE
    DLOG(INFO) << "started fill_travel_time_onTrip_work";
#endif

    /*
     * Nothing to do
     */
    if (process_order.empty()) return;


#ifdef VRPMINTRACE
    DLOG(INFO) << "fill_travel_time_onTrip doing " <<  " size " << process_order.size() << "\n";
#endif

    while (!process_order.empty()) {
        STATS_INC("TWC:: Processing pairs");

        UINT source = process_order.begin()->first.first;
        UINT target = process_order.begin()->first.second;


#ifdef VRPMINTRACE
        if ((process_order.size() % 200) == 0)
            DLOG(INFO) << "fill_travel_time_onTrip " << source << " size " << process_order.size() << " working with " << original[source].id() << "," << original[target].id()
                << " source,j onTrip time" << travel_time_onTrip[source][target]
                << " source,j on data time" << travel_Time[source][target]
                << " source,j onTrip time" << travel_time_onTrip[target][source]
                << " source,j on data time" << travel_Time[target][source] << "\n";
#endif

        process_order.erase(process_order.begin());

        if (source == target) {
            STATS_INC("TWC:: Found source == target");
            assert(travel_time_onTrip[source][target] == 0);
            assert(travel_Time[source][target] == 0);
            continue;
        }
        process_pair_onPath(original[source],original[target]);
        process_pair_onPath(original[target],original[source]);
    } // while


#ifdef VRPMINTRACE
    DLOG(INFO) << "end fill_travel_time_onTrip_work";
#endif
    return;
}

double
TWC::fn_travel_time_onTrip(Trashnode from, Trashnode to) const {
    return travel_time_onTrip[from.nid()][to.nid()];
} 


void 
TWC::process_pair_onPath(Trashnode from, Trashnode to) const
{
    STATS_INC("--> process_pair_onPath(Trashnode from, Trashnode to)");
    assert(travel_time_onTrip.size() == original.size());

    /*
     * nothing to do
     */
    if (from.nid() == to.nid()) {
        assert(fn_travel_time_onTrip(from, to) == 0);
        return;
    }


    /*
     * from a dumpSite: connot go to dumpSite
     */
    if (from.isDump() && to.isDump()) {
        travel_time_onTrip[from.nid()][to.nid()] = -1;
        assert(nodes_onTrip[from.nid()][to.nid()].empty());
        return;
    }

    /*
     * from a Depot Site: connot go to dumpSite or depot Site
     */
    if (from.isDepot() && (to.isDump() || to.isDepot())) {
        travel_time_onTrip[from.nid()][to.nid()] = -1;
        assert(nodes_onTrip[from.nid()][to.nid()].empty());
        return;
    }

    /*
     * from a pickup Site(container): connot go to depot Site
     */
    if (from.isPickup() && to.isDepot()) {
        travel_time_onTrip[from.nid()][to.nid()] = -1;
        assert(nodes_onTrip[from.nid()][to.nid()].empty());
        return;
    }

    TwBucket trip;
    if (fn_travel_time_onTrip(from, to) == 0) {

        assert(nodes_onTrip[from.nid()][to.nid()].empty());

        /*
         * get the nodes that dont have a (from, node) value and set to unassigned
         */
        TwBucket unnasigned;
        for (const auto &node : original.path) {
            if (node.isPickup()
                    && node.nid() != from.nid() &&  node.nid() != to.nid()
                    && travel_time_onTrip[from.nid()][node.nid()] == 0) {
                unnasigned.push_back(node);
            }
        }

        DLOG(INFO) << "unnassigned: " << unnasigned.size();


        auto nodesOnPath = getNodesOnPathInclusive(from, to, unnasigned);
        assert(fn_travel_time_onTrip(from, to) == 0);

        fill_times(nodesOnPath);
    }
}



// the values for non containers to/from containers should be filled
void 
TWC::compulsory_fill() {
    SET_TIMER(timer);
    STATS_INC("compulsory_fill");
    DLOG(INFO) << "started compulsory_fill";

    assert(!original.empty());



    auto destinations = original;
    for (auto &special_node : original.path) {
        /*
         * Cycle only on dump start ending and average nodes
         */
        if (special_node.isPickup()) continue;

        /*
         *  work with ordered original from furthest to closest to special node
         */


        std::sort(destinations.path.begin(), destinations.path.end(),
                [&special_node] (const Trashnode &left, const Trashnode &right) {
                return special_node.distanceToSquared(left) > special_node.distanceToSquared(right);
                });

        for (auto &container : destinations.path) {
            process_pair_onPath(special_node, container);
            process_pair_onPath(container, special_node);
        }
    }

    DLOG(INFO) << "Ended compulsory_fill";
    STATS_ADDTO("TWC::compulsory_fill Cumultaive time:", timer.duration());
}





void 
TWC::fill_times(const TwBucket nodesOnPath) const {
    STATS_INC(" --> fill_times");

    // using osrm
    bool oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);
    osrmi->clear();

    osrmi->addViaPoint(nodesOnPath.path);
    if (!osrmi->getOsrmViaroute()) {
        DLOG(WARNING) << "getOsrmViaroute failed";
        osrmi->useOsrm(oldStateOsrm);
        return;
    }

    auto times = osrmi->getOsrmTimes();
    if ((times.size() + 1) != nodesOnPath.path.size()){
        DLOG(WARNING) << "getOsrmTimes failed";
        osrmi->useOsrm(oldStateOsrm);
        return;
    }


    osrmi->useOsrm(oldStateOsrm);



    // fills the 2D table

    auto i = 0;
    for (auto i_ptr = nodesOnPath.path.begin();
            i_ptr != nodesOnPath.path.end();
            ++i, ++i_ptr) {
        auto j = i + 1;
        for (auto j_ptr = nodesOnPath.path.begin() + j;
                j_ptr != nodesOnPath.path.end();
                ++j, ++j_ptr) {

            auto from = i_ptr->nid();
            auto to = j_ptr->nid();

            /*
             * Nothing to do
             */
            assert(i_ptr != j_ptr);
            assert(from != to);

            if (travel_time_onTrip[from][to] != 0) continue;

            // assert(nodes_onTrip[from][to].empty());

            /*
             *   - there is no previouus information
             *   - or there are more nodes
             */
            STATS_INC("TWC::fillTimes total times updated");


            /*
             *  update to found sequence of nodes
             */
            auto k = i;
            double total_time = times[k];
            for (auto k_ptr = i_ptr + 1; k_ptr != j_ptr; ++k, ++k_ptr) {
                /*
                 * Only inserting containers
                 */
                if (k_ptr->isPickup()) {
                    nodes_onTrip[from][to].push_back(k_ptr->nid());
                }
                total_time += times[k + 1];
            }

            /*
             * Update the times
             */
            travel_Time[from][to]
                = travel_time_onTrip[from][to]
                = total_time;
        }
    }
}




TwBucket 
TWC::getNodesOnPath(
        const TwBucket &truck,
        const Trashnode &dumpSite,
        const TwBucket &unassigned) const {
    TwBucket orderedStreetNodes;

    STATS_INC("TWC::getNodesOnPath");


    /*
     * Nothing to do
     */
    if (unassigned.empty()) return orderedStreetNodes;



    /*
     * forcing use of osrm
     */
    auto oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);
    osrmi->clear();



    /*
     * building call
     */
    auto call = truck;
    call.push_back(dumpSite);

    osrmi->addViaPoint(call.path);



    /*
     * calling osrm
     */
    if (!osrmi->getOsrmViaroute()) {
        DLOG(WARNING) << "getNodesOnPath getOsrmViaroute failed";
        osrmi->useOsrm(oldStateOsrm);
        return orderedStreetNodes;
    }



    /*
     * getting the geometry
     */
    std::deque<Node> geometry;
    if (!osrmi->getOsrmGeometry(geometry)) {
        DLOG(WARNING) << "getNodesOnPath getOsrmGeometry failed";
        osrmi->useOsrm(oldStateOsrm);
        return orderedStreetNodes;
    }



    /*
     * Nothing to do:
     *   the geometry returns the 2 points entered
     */
    if (geometry.size() == 2) return orderedStreetNodes;



    /*
     * Not modifying unassigned
     */
    auto streetNodes = unassigned;



    /* TODO(vicky) have this value as a parameter to the problem

       Node::positionAlongSegment() is doing Euclidean calcuations
       so this needs to be set in degrees or meters depending on the
       underlying projection that the node x,y values are in.

       Approximate meters in degrees longitude at equator
       0.00009 degrees === 10 meters
       0.00027 degrees === 30 meters
       */
    const double tolerance = 0.00014;



    /*
     * position, nid
     */
    typedef std::tuple<double, size_t> Storage;


    auto prev_node = geometry.front();
    for(auto &node : geometry) {
        /*
         * skip first. 
         */
        if (node == geometry.front()) continue;

        /******************************
         *
         * working with segment (prev_node -> node)
         *
         ******************************/

        /*
         * data has the (position, nid) of the nodes that belong to the segment
         */
        std::deque<Storage> data;



        /*
         * Find all nodes that are on the segment
         */
        for (auto s_node : streetNodes.path) {
            /*
             * only pickups are searched for
             */
            if (!s_node.isPickup()) continue;
            auto pos = s_node.positionAlongSegment(prev_node, node, tolerance);
            if (pos > 0) {
                data.push_back(std::make_tuple(pos, s_node.nid()));
            }
        }


        /*
         * preparing for next cycle
         */
        prev_node = node;



        /*
         * no nodes on the way
         */
        if (data.empty()) continue; 



        /*
         * sort them by position
         */
        std::sort(data.begin(), data.end(),
                [](auto const &left,
                    auto  const &right) {
                return std::get<0>(left) < std::get<0>(right);
                });



        /*
         * Adding the ordered nodes of the segment to the result
         */
        for (const auto &d : data) {
            orderedStreetNodes.push_back(original[std::get<1>(d)]);
        }



        /*
         * erase from the streetNodes the nodes that have being used
         */
        for (const auto &d : data) {
            for (auto it = streetNodes.path.begin(); it != streetNodes.path.end(); ++it) {
                if ((*it).nid() == std::get<1>(d)) {
                    streetNodes.path.erase(it);
                    break;
                }
            }
        }

    }  //  next segment



    /************************************************************/
    osrmi->useOsrm(oldStateOsrm);

    return orderedStreetNodes;
}



TwBucket 
TWC::getNodesOnPathInclusive(
        const Trashnode &from,
        const Trashnode &to,
        const TwBucket &unassigned) const {
    assert(fn_travel_time_onTrip(from, to) == 0);

    TwBucket trip;
    trip.push_back(from);

    auto nodesOnPath = getNodesOnPath(trip, to, unassigned);
    assert(fn_travel_time_onTrip(from, to) == 0);

    nodesOnPath.push_front(from);
    nodesOnPath.push_back(to);

    assert(nodesOnPath.size() > 1);
    assert(fn_travel_time_onTrip(from, to) == 0);
    return nodesOnPath; 
}



bool 
TWC::setTravelingTimesOfRoute(
        const TwBucket &truck,
        const Trashnode &dumpSite) const {

    bool oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);  //forcing osrm usage

    osrmi->clear();

    /*
     * dumpsite < nodes >
     */
    auto call = truck;
    call.push_back(dumpSite);
    std::reverse(call.begin(), call.end());


    /*
     * <nodes> dumpsite  dumpsite < nodes >
     */
    call.push_front(dumpSite);
    call.path.insert(call.path.begin(), truck.path.begin(), truck.path.end());

    // process osrm
    osrmi->addViaPoint(call.path);
    if (!osrmi->getOsrmViaroute()) {
        DLOG(INFO) << "getOsrmViaroute failed";
        osrmi->useOsrm(oldStateOsrm);  
        return false;
    }

    auto times = osrmi->getOsrmTimes();
    if (times.size() != call.size()){
        DLOG(INFO) << "getOsrmTimes failed";
        osrmi->useOsrm(oldStateOsrm);  
        return false;
    }


    auto i = 0;
    for (auto i_ptr = call.path.begin();
            i_ptr != call.path.end();
            ++i, ++i_ptr) {
        if ( (*i_ptr) == call.path.back()) continue;
        travel_Time[i_ptr->nid()][(i_ptr + 1)->nid()] = times[i + 1] - times[i];
    }

    osrmi->useOsrm(oldStateOsrm);  
    return true;
}







bool 
TWC::setTravelingTimesInsertingOneNode(
        const TwBucket &truck,
        const Trashnode &dumpSite,
        const Trashnode &node) const {


    bool oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);  //forcing osrm usage

    if (truck.size() == 1) {
        TravelTime(truck[0], node, dumpSite);
        osrmi->useOsrm(oldStateOsrm);  
        return false;
    }

    // buld call
    unsigned int tSize = truck.size();
    osrmi->clear();

    TwBucket call;
    // special case  0 n 1
    call.push_back(truck[0]);
    call.push_back(node);
    call.push_back(truck[1]);
    // cycle:
    if (tSize > 2) {
        for (unsigned int i= 0; i < tSize - 3; ++i) {
            call.push_back(truck[i]);
            call.push_back(truck[i+1]);
            call.push_back(node);
            call.push_back(truck[i+2]);
        }
    }
    // special case 5 6 n   // 0 1 n D
    call.push_back(truck[tSize - 2]);
    call.push_back(truck[tSize - 1]);
    call.push_back(node);
    call.push_back(dumpSite);

    // process osrm
    osrmi->addViaPoint(call.path);
    if (!osrmi->getOsrmViaroute()) {
        DLOG(INFO) << "getOsrmViaroute failed";
        osrmi->useOsrm(oldStateOsrm);  
        return false;
    }

    auto times = osrmi->getOsrmTimes();
    if ((times.size() + 1) != call.size()) {
        DLOG(INFO) << "getOsrmTimes failed";
        osrmi->useOsrm(oldStateOsrm);  
        return false;
    }


    // lets have a peek
#ifdef VRPMAXTRACE 
    DLOG(INFO) << "squential";
    for (unsigned int i= 0; i < call.size(); ++i) {
        DLOG(INFO) << call[i].id() << "," << times[i];
    }
#endif 


#ifdef VRPMAXTRACE 
    DLOG(INFO) << "pairs";
#endif 
    for (unsigned int i = 0; i < call.size()-1; ++i) {
        TravelTime(call[i].nid(), call[i+1].nid());
        travel_Time[call[i].nid()][call[i+1].nid()] = times[i+1]-times[i];
#ifdef VRPMAXTRACE 
        DLOG(INFO) << call[i].id() << " -> " 
            << call[i+1].id() << " = " << times[i+1] - times[i];
#endif 
    }

    osrmi->useOsrm(oldStateOsrm);  
    return true;
}










bool 
TWC::findFastestNodeTo(
        bool first,
        const TwBucket &truck,
        TwBucket &unassigned,
        const Trashnode &dumpSite,
        POS &pos,
        Trashnode &bestNode,
        double &bestTime) const
{
#ifdef VRPMINTRACE
    DLOG(INFO) << "started findFastestNodeTo";
#endif
    assert(unassigned.size());
    bool flag = false;
    bestTime = VRP_MAX();   // time to minimize
    pos = 0;        // position in path to insert
    double tAdd;
    double tSubs;
    double deltaTime;
    int bestIndex;

    for ( size_t j = 0; j < truck.size(); j++ ) {
        for ( size_t i = 0; i < unassigned.size(); i++ ) {
            if (j == 0) {
                setTravelingTimesInsertingOneNode(truck, dumpSite, unassigned[i]);
            }
            // special case
            if (j ==  truck.size()-1) {
                if (j == 0) {
                    tAdd = TravelTime(truck[j], unassigned[i], dumpSite);
                    tSubs = TravelTime(truck[j], dumpSite);
                } else {
                    tAdd = TravelTime(truck[j-1], truck[j], unassigned[i], dumpSite);
                    tSubs = TravelTime(truck[j-1], truck[j], dumpSite);
                }
            } else {
                if (j == 0) {
                    tAdd = TravelTime(truck[j], unassigned[i], truck[j+1]);
                    tSubs = TravelTime(truck[j], truck[j+1]);
                } else {
                    tAdd = TravelTime(truck[j-1], truck[j], unassigned[i], truck[j+1]);
                    tSubs = TravelTime(truck[j-1], truck[j], truck[j+1]);
                }

            }
            deltaTime = tAdd - tSubs;
#ifdef VRPMAXTRACE
            if (j==truck.size()-1)
                DLOG(INFO) << "+(" << truck[j].id() << " " << unassigned[i].id() << " " << dumpSite.id() << ")" << "time= " <<tAdd;
            else
                DLOG(INFO) << "+(" << truck[j].id() << " " << unassigned[i].id() << " " << truck[j+1].id() << ")" << "time= " <<tAdd;

            if (j==truck.size()-1)
                DLOG(INFO) << "-(" << truck[j].id() << " "  << dumpSite.id() << ")" << "time= " << tSubs;
            else
                DLOG(INFO) << "-(" << truck[j].id() << " "  << truck[j+1].id() << ")" << "time= " << tSubs;

            DLOG(INFO) << "delta Time= " << deltaTime;
#endif
            if (first && ((-tAdd) < bestTime)) {
                // if ((truck.size() == 1) && ((-tAdd) < bestTime)) {
                bestTime = -tAdd;
                pos = j + 1;
                bestNode = unassigned[i];
                bestIndex = i;
                flag = true;
            }
            if (!first && (deltaTime < bestTime)) {
                //if ((truck.size() > 1) && (deltaTime < bestTime)) {
                bestTime = deltaTime;
                pos = j + 1;
                bestNode = unassigned[i];
                bestIndex = i;
                flag = true;
                if (bestTime < 0.00005) break;
            }
            // }
            }  //for i
            if (bestTime < 0.00005) break;
        }  // for j
        // before returning all i < bestIndex place them at end of unassigned
        for (int i = 0; i < bestIndex; i++) {
            unassigned.push_back(unassigned[0]);
            unassigned.erase(unassigned[0]);
        }
#ifdef VRPMINTRACE
        DLOG(INFO) << "ended findFastestNodeTo";
#endif
        return flag;
    }



    ///@}



double 
TWC::getTravelTime(UID from, UID to) const {
    assert(from < original.size() && to < original.size());
    if (travel_Time[from][to] == -1) {
        STATS_INC("TWC::extra process_pair_onPath");

        process_pair_onPath(original[from],original[to]);
        process_pair_onPath(original[to],original[from]);
    }
    return travel_Time[from][to];
}



/*!  \brief Retruns travel time from node id \b from to node id \b to.
  interfaces
  */
//@{
double 
TWC::TravelTime(UID from, UID to) const {
    return getTravelTime(from,to);
}

double 
TWC::TravelTime(UID from, UID middle, UID to) const {
    assert(from < original.size());
    assert(middle < original.size());
    assert(to < original.size());
    TravelTime(from,to);
    return  TravelTime(from, middle) + TravelTime(middle, to);
}


double 
TWC::TravelTime(UID prev, UID from, UID middle, UID to) const {
    assert(prev < original.size());
    assert(from < original.size());
    assert(middle < original.size());
    assert(to < original.size());
    TravelTime(prev,to);
    TravelTime(from,to);
    TravelTime(prev, middle);
    return  TravelTime(prev, from) +  TravelTime(from, middle)
        + TravelTime(middle, to);
}

double 
TWC::TravelTime(const Trashnode &from, const Trashnode &to) const {
    return TravelTime(from.nid(), to.nid());
}

double 
TWC::TravelTime(const Trashnode &from, const Trashnode &middle, const Trashnode &to) const {
    return TravelTime(from.nid(), from.nid(), middle.nid(), to.nid());
}

double 
TWC::TravelTime(const Trashnode &prev, const Trashnode &from, const Trashnode &middle, const Trashnode &to) const {
    return TravelTime(prev.nid(), from.nid(), middle.nid(), to.nid());
}
//@}

bool 
TWC::isInPath(UINT from, UINT middle, UINT to) {
#if 0
    TravelTime(from,to);
    if (travel_time_onTrip[from][to] == 0)
        process_pair_onPath(from, to);

    if (std::find(nodes_onTrip[from][to].begin(),
                nodes_onTrip[from][to].end(), middle)
            != nodes_onTrip[from][to].end()) return false;
    return true;
#endif
    //if (travel_time_onTrip[from][to] == 0)
    process_pair_onPath(original[from], original[to]);
    //if (travel_time_onTrip[from][middle] == 0)
    process_pair_onPath(original[from], original[middle]);
    //if (travel_time_onTrip[middle][to] == 0)
    process_pair_onPath(original[middle], original[to]);
#if 0
    DLOG(INFO) << from << "," << middle <<"," << to;
    DLOG(INFO) <<
        travel_time_onTrip[from][to]  << " ?? " <<  travel_time_onTrip[from][middle] << " + " <<travel_time_onTrip[middle][to];
#endif
    return travel_time_onTrip[from][to] >= (travel_time_onTrip[from][middle] + travel_time_onTrip[middle][to]);
}

bool 
TWC::isInPath(const Trashnode &from, const Trashnode &middle, const Trashnode& to) {
    // DLOG(INFO) << from.nid() << "," << middle.nid() <<"," << to.nid();
    // DLOG(INFO) << TravelTime(from,to) << " ?? " << TravelTime(from,middle) << " + " << TravelTime(middle,to);
    return isInPath(from.nid(),middle.nid(),to.nid());
    //return TravelTime(from,to) >= (TravelTime(from,middle) + TravelTime(middle,to));
}


















/*!
 * \brief Fetch a node by its node id from the original container of nodes.
 *
 * \param[in] nid The node id of the desired node.
 * \return A copy of the node from the original container of nodes.
 */
const Trashnode &
TWC::node(UID nid) const {
    assert(nid < original.size());
    return original[nid];
}

/*!
 * \brief Fetch a node by its node id from the original container of nodes.
 *
 * \param[in] nid The node id of the desired node.
 * \return A copy of the node from the original container of nodes.
 */
const Trashnode &
TWC::getNode(UID nid) const {
    assert(nid < original.size());
    return original[nid];
}


/*! \brief Report toNide is logically reachable from fromNid.
 *
 * \param[in] fromNid Nid of the from node.
 * \param[in] toNid Nid of the to node.
 * \return True if reachable, false otherwise.
 */
bool 
TWC::isReachableIJ(UID fromNid, UID toNid) const {
    assert(fromNid < original.size() && toNid < original.size());
    return !(TravelTime( fromNid, toNid )  == VRP_MAX());
}





// ------------------------ DUMPS --------------------------

#ifdef DOVRPLOG

void 
TWC::print_nodes_onTrip(size_t from, size_t to) const {
    std::stringstream ss;
    for (const auto node : nodes_onTrip[from][to]) {
        ss << original[node].id() << " ";
    }
    DLOG(INFO) << ss.str();
}

/*! \brief Print the original nodes.  */
void 
TWC::dump() const  {
    assert(original.size());
    dump(original);
}

/*!
 * \brief Print the nodes in the bucket.
 * \param[in] nodes A bucket of nodes to print.
 */
void 
TWC::dump(const TwBucket &nodes) const
{
    assert(nodes.size());
    original.dump("original");
}




#endif  // logs






/*!
 * \brief Set TWC from fromNid to toNid to be incompatible & unreachable.
 *
 * \param[in] fromNid The predecessor node id.
 * \param[in] toNid The successor node id.
 */
void 
TWC::setIncompatible(UID fromNid, UID toNid) {
    assert(fromNid < original.size() && toNid < original.size());
    twcij[fromNid][toNid] = VRP_MIN();
    travel_Time[fromNid][toNid] =  VRP_MAX();
}




/*!
 * \brief Assign the bucket of nodes as the TWC original and compute TWC.
 *
 * \param[in] _original A bucket of nodes to assign to TWC class.
 */
void 
TWC::setNodes(TwBucket _original) {
    original.clear();
    original = _original;
    twcij_calculate();
    assert(original == _original);
    assert(check_integrity());

    setPhantomNodes();
}



/*!
 * \brief Compute the average travel time to a given node.
 *
 * \param[in] from A bucket of nodes to use as the start node.
 * \param[in] to A node to be used as the destination node.
 * \return The average travel time from start to destination.
 */
double 
TWC::getAverageTime(const TwBucket &from, const Trashnode &to) const
{
    DLOG(INFO) << "getAverageTime to" << to.nid();
    assert(to.nid() < original.size());
    double time = 0;
    auto j = to.nid();
    auto count = from.size();

    for ( size_t i = 0; i < from.size(); i++ ) {
        if (TravelTime(from[i].nid(), j) < 0) {
            DLOG(INFO) << "found a negative";
            travel_Time[from[i].nid()][j] = 0.00001;
            travel_time_onTrip[from[i].nid()][j] = 0.00001;
            --count;
            continue;
        }
        time += TravelTime(from[i].nid(),j);
    }

    time = time / count;
    DLOG(INFO) << "time = " << time;
    return time;
}

/*!
 * \brief Compute the average travel time from a given node.
 *
 * \param[in] from The start node.
 * \param[in] to A bucket of destination nodes.
 * \return The average travel time from start to destination.
 */
double 
TWC::getAverageTime(const Trashnode &from, const TwBucket &to) const
{
    DLOG(INFO) << "getAverageTime from" << from.nid();
    assert(from.nid() < original.size());
    double time = 0;
    auto j = from.nid();
    auto count = to.size();

    for (size_t i = 0; i < to.size(); i++) {
        if (TravelTime(j, to[i].nid()) < 0) {
            DLOG(INFO) << "found a negative";
            travel_Time[j][to[i].nid()] = 1;
            travel_time_onTrip[j][to[i].nid()]= 1;
            --count;
            continue;
        }
        time += TravelTime(j, to[i].nid());
    }
    time = time / count;
    DLOG(INFO) << "time = " << time;
    return time;
}







/*!
 * \brief Set tCC set average travel time  between containers in bucket picks
 * \param[in] C    average contaienr
 * \param[in] picks
 */
void 
TWC::settCC(const Trashnode &C, const TwBucket &picks) {
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
bool 
TWC::sameStreet(UID i, UID j) const {
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
double 
TWC::gradient(UID i, UID j) const {
    assert(i < original.size() && j < original.size());
    return original[i].gradient( original[j] );
}



/// \brief Sets the hints and the street's id to the nodes in the bucket
/**

  The hint & street when id stored in "original" is copyied into the bucket's
  nodes.

  To be used after "original" has being filled with the appropiate values
  */
    void 
TWC::setHints(TwBucket &nodes)
{
    SET_TIMER(timer);

    for (size_t i = 0; i < nodes.size(); i++ ) {
        nodes[i].set_hint(original[nodes[i].nid()].hint());
        nodes[i].set_streetId(original[nodes[i].nid()].streetId());
    }

    STATS_ADDTO("TWC::setHints Cumultaive time:", timer.duration());
}



void 
TWC::prepareTravelTime() {
    auto n = original.size();

    travel_Time = std::vector<std::vector<double>>(
            n, std::vector<double>(n, -1));

    for (size_t i = 0; i < n; i++) 
        travel_Time[i][i] = 0.0;
}




void 
TWC::getAllHintsAndStreets() {
    SET_TIMER(timer);

#ifdef VRPMAXTRACE
    DLOG(INFO) << "getAllHintsAndStreets\n";
#endif
    std::deque<std::string> hints;
    std::deque<std::string> streets;
    std::map< std::string, int>::const_iterator street_ptr;
    int total = original.size();
    int from, to;
    int i, j, k;

    for ( i = 0; (i * 100) < total; i++ ) {
        from = i * 100;
        to = std::min((i + 1) * 100, total);
        hints.clear();
        osrmi->clear();

        for ( j = from; j < to ; j++ ) osrmi->addViaPoint(original[j]);

        if (   osrmi->getOsrmViaroute()
                && osrmi->getOsrmHints(hints)
                && osrmi->getOsrmStreetNames(streets)) {
            for (j = from, k = 0; j < to; j++, k++) {
                // setting the hint
                original[j].set_hint(hints[k]);
                // setting the street
                street_ptr = streetNames.find(streets[k]);
                if (street_ptr == streetNames.end()) {
                    int newStreetId = streetNames.size();
                    streetNames[streets[k]] = newStreetId;
                    original[j].set_streetId(newStreetId);
                    assert(streetNames.find(streets[k]) != streetNames.end());
                } else {
                    original[j].set_streetId(street_ptr->second);
                }

            }
        } else {
#ifdef VRPAMXTRACE
            DLOG(INFO) << "NO HINTS WERE FOUND\n";
#endif
        }
    }

    STATS_ADDTO("TWC::getAllHintsAndStreets Cumultaive time:", timer.duration());
}



// Just for log. Dump travel_Time matrix.
void 
TWC::dump_travel_Time() {
    int rcSize = original.size();
    DLOG(INFO) << "Begin travel_Time matrix";
    for ( int i = 0; i < rcSize; i++ ) {
        std::stringstream row;
        for ( int j = 0; j < rcSize; j++ ) {
            row  << "\t" << travel_Time[i][j];
        }
        DLOG(INFO) << row.str() << std::endl;
    }
    DLOG(INFO) << "End travel_Time matrix";
}




void 
TWC::loadAndProcess_travelTimes(
        std::string infile,
        const TwBucket &datanodes,
        const TwBucket &invalid) {
    STATS_INC("TWC::loadAndProcess_travelTimes");
    SET_TIMER(timer);
    DLOG(INFO) << "loading travel times of " << datanodes.size() << " nodes"; 

    assert(!datanodes.empty());


    original.clear();
    original = datanodes;

    getAllHintsAndStreets();

    prepareTravelTime();


    std::ifstream in(infile.c_str());
    std::string line;



    int64_t from, to;
    double time;
    int cnt = 0;

    while (getline(in, line)) {
        cnt++;

        // skip comment lines
        if ( line[0] == '#' ) continue;

        std::istringstream buffer(line);
        buffer >> from;
        buffer >> to;
        buffer >> time;

        if (invalid.hasId(from) || invalid.hasId(to)) continue;
        if (!original.hasId(from) || !original.hasId(to)) continue;

        auto fromId = getNidFromId(from);
        auto toId = getNidFromId(to);

        STATS_INC("TWC::loadAndProcess_travelTimes total data read");
        /*
         * Reading data is considered symetrical
         */
        travel_Time[fromId][toId] = time;
        travel_Time[toId][fromId] = time;
    }

    in.close();

    SET_TIMER(timer1);
    fill_travel_time_onTrip();
    STATS_ADDTO("TWC::loadAndProcess_travelTimes fill Cumultaive time:", timer1.duration());

    setPhantomNodes();

    SET_TIMER(timer2);
    twcij_calculate();
    STATS_ADDTO("TWC::loadAndProcess_travelTimes twcij Cumultaive time:", timer2.duration());

    assert(original == datanodes);
    assert(check_integrity());
}



/*! \brief Returns a constant reference to the travel time matrix. */
const std::vector<std::vector<double> >& 
TWC::TravelTime() {
    return travel_Time;
}

/*! \brief Retrieves the internal node id (NID) from the users node id (ID)
 *
 * \param[in] id A user node identifier
 * \return The internal nid corresponding to id or -1 if not found.
 */
UID 
TWC::getNidFromId(UID id) const {
    return original.getNidFromId(id);
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
double 
TWC::ajli(const Trashnode &ni, const Trashnode &nj) const {
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
double 
TWC::ajei(const Trashnode &ni, const Trashnode &nj) const {
    return ni.opens() + ni.serviceTime() + TravelTime(ni, nj);
}


/*!
 * \brief Compute TWC from node \b ni to node \b nj
 *
 * \param[in] ni From this node
 * \param[in] nj To this node
 * \return The TWC value traveling from node \b ni directly to \b nj
 */
double 
TWC::twc_for_ij(const Trashnode &ni, const Trashnode &nj) const {
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



/* public functions That are id based */


/*!
 * \brief Compute all TWC values and populate the TWC matrix.
 */
void 
TWC::twcij_calculate() {
    assert(original.size() == travel_Time.size());
    twcij.resize(original.size());

    for (auto &row : twcij) row.resize(original.size());

    for (auto &from : original ) {
        for (auto &to : original ) {
            if (from.nid() == to.nid()) continue;
            twcij[from.nid()][to.nid()] = twc_for_ij(from, to);
        }
    }
}

/*!  \brief Check's that the twcij was created
  O(N) where N is the number of nodes
  */
bool 
TWC::check_integrity() const {
    assert(original.size() == twcij.size());

    for (const auto row : twcij) assert(row.size() == original.size());

    return true;
}

/********************************
 * Fernandos code for rewrite
 * *******************************
 */


void 
TWC::set_TravelTime(UID fromId, UID toId, double time) {
#ifdef VRPMINTRACE
    if (!(travel_Time[fromId][toId] == time))
        DLOG(INFO) << "<travel_time[" << fromId << "][" << toId << "]="
            << travel_Time[fromId][toId] << " ---> " << time;
#endif
    travel_Time[fromId][toId] = time;
}




    void 
TWC::setPhantomNodes()
{

#if 0 //REWRITE
    // TODO(fernanado) rewrite with current OSRM 
    // Multiplier for before and after
    double mb = 0.95;
    double ma = 1.05;
    // Delete previous
    mPhantomNodes.clear();
#ifdef VRPMINTRACE
    DLOG(INFO) << "mPhantonNodes cleared!";
    DLOG(INFO) << "original have " << original.size() << " elements!";
#endif

    // Variables
    bool oldStateOsrm;
    double pnlon, pnlat; // PhantomNode
    double fnlon, fnlat; // Fisical Node
    unsigned int one_way;
    unsigned int fw_id, rv_id, fw_wt, rv_wt, street_id;

    // Backup OSRM state
    oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);  //forcing osrm usage
    osrmi->clear();

    int pncount = 0;
    for (UINT i = 0; i < original.size(); i++) {
        if ( original[i].isPickup() ) {
#ifdef VRPMINTRACE
            std::cout.precision(6);
            DLOG(INFO) << original[i].id() << " is pickup!";
#endif
            one_way = 100;
            // TODO(fernanado) 
            /*
             * https://github.com/Project-OSRM/osrm-backend/blob/master/docs/http.md#service-nearest
             * GET "http://localhost:5000/nearest/v1/driving/-56.1743363,-34.9137291"

             * ~~~~{.c}
             * {
             *     "waypoints": [
             *         {
             *             "hint": "VTIAgMe0AIDwJwAAFAAAADAAAAAAAAAAAAAAAGGPAACz7wAAEgAAAAjZpvwAQuv9ANmm_D9C6_0AAAEBwvX4oQ==",
             *             "distance": 7.045144,
             *             "name": "Doctor Luis Piera",
             *             "location": [
             *                 -56.174328,
             *                 -34.913792
             *             ]
             *         }
             *     ],
             *     "code": "Ok"
             * }
             * ~~~~
             */


            Node phantomNode;
            double distance;
            std::string street;
            osrmi->getOsrmNearest(original,  phantomNode, distance, street);
            // TODO this is wrong now
            osrmi->getOsrmNearest( original[i].x(), original[i].y(), pnlon, pnlat, one_way, fw_id, rv_id, fw_wt, rv_wt, street_id);
            if (one_way == 0) {
#ifdef VRPMINTRACE
                DLOG(INFO) << original[i].id() << " [lon,lat] " << original[i].x() << original[i].y() << " is in two way street!";
#endif
                // Two way street
                PhantomNode pn = PhantomNode(pncount, pnlon, pnlat, fw_id, rv_id, fw_wt, rv_wt, street_id);
                // Get nearest fisical OSRM node (edge intersection) of phantom
                osrmi->getOsrmLocate(pnlon, pnlat, fnlon, fnlat);
                // Add before and after to pn
                double alon, alat, blon, blat;
                // WARNING: longitude and latitude!!!!!!
                // Only valid for very short distances
                //
                // I think
                //
                // FN---------PN
                //            |
                //         original[i]
                //
                // Before
                blon = fnlon + mb * (pnlon - fnlon);
                blat = fnlat + mb * (pnlat - fnlat);
                // After
                alon = fnlon + ma * (pnlon - fnlon);
                alat = fnlat + ma * (pnlat - fnlat);
#ifdef VRPMINTRACE
                std::cout << std::setprecision(8) << "PN: (" << pnlon << "," << pnlat << ")" << std::endl;
                std::cout << "FN: (" << fnlon << "," << fnlat << ")" << std::endl;
                std::cout << "Before: (" << blon << "," << blat << ")" << std::endl;
                std::cout << "After: (" << alon << "," << alat << ")" << std::endl;
#endif
                bool ret = original[i].isRightToSegment(
                        Node(fnlon,fnlat),
                        Node(pnlon,pnlat)
                        );
                Point pb, pa;
                if (ret) {
                    pb = Point(blon,blat);
                    pa = Point(alon,alat);
                    pn.setBeforePNode( pb );
                    pn.setAfterPNode( pa );
                } else {
                    // Not as you think!!!
                    //
                    //         original[i]
                    //            |
                    // FN---------PN
                    //
                    pb = Point(alon,alat);
                    pa = Point(blon,blat);
                    pn.setBeforePNode( pb );
                    pn.setAfterPNode( pa );
                }
#ifdef VRPMINTRACE
                std::cout << std::setprecision(8) << "PhantomNode" << std::endl;
                std::cout << pn << std::endl;
#endif
                // Add pn to de map
                f = pn;
                pncount++;
            }
#ifdef VRPMINTRACE
            if (one_way == 1) {
                DLOG(INFO) << original[i].id() << " is in one way street!";
            }
#endif
        }
    }
    osrmi->useOsrm(oldStateOsrm);

#ifdef VRPMINTRACE
    DLOG(INFO) << "Begin PhantomNodes for pickups sites";
    DLOG(INFO) << "CONID" << "\t" << "COID" << "\t" << "COLON" << "\t" << "COLAT" << "\t" << "PNID" << "\t" << "PNLON" << "\t" << "PNLAT" << "\t"
        << "BELON" << "\t" << "BELAT" << "\t" << "AFLON" << "\t" << "AFLAT";
    for (UINT i = 0; i < original.size(); i++) {
        UID id = original[i].id();
        auto it = mPhantomNodes.find( id );
        if ( it!=mPhantomNodes.end() ) {
            DLOG(INFO) << std::setprecision(8) << original[i].nid() << "\t" << original[i].id() << "\t" << original[i].x() << "\t"  << original[i].y() << "\t"
                << it->second.id() << "\t" << it->second.point().x() << "\t" << it->second.point().y() << "\t"
                << it->second.beforePNode().x() << "\t" << it->second.beforePNode().y() << "\t"
                << it->second.afterPNode().x() << "\t" << it->second.afterPNode().y();
        }
    }
    DLOG(INFO) << "End PhantomNodes for pickups sites";
#endif



#endif  // REWRITE
}


#if 0 // REWRITE
/*
 * NEW CODE BY FERNANDO
 * uses phantomnodes
 *
 *
 */

void 
TWC::fill_times(const TwBucket nodesOnPath) const {

#ifdef VRPMINTRACE
    DLOG(INFO) << "strated fill_times";
#endif  // VRPMINTRACE

#ifdef VRPMINTRACE
    nodesOnPath.dump("nodesOnPath");
#endif  // VRPMINTRACE

    //get all the times using osrm
    bool oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);  //forcing osrm usage
    osrmi->clear();

    // To build the call
    std::deque< Twnode > call;
    UID id;
    for (unsigned int i = 0; i < nodesOnPath.size(); ++i) {
        id = nodesOnPath[i].id();
        auto it = mPhantomNodes.find( id );
        if ( i==0 ) {
            call.push_back( nodesOnPath[i] );
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.afterPNode().x(),it->second.afterPNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
        } else {
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
            call.push_back( nodesOnPath[i] );
        }
        // call.push_back(nodesOnPath[i]);
#ifdef VRPMINTRACE
        DLOG(INFO) << "Nodos en el path:" << i << " ID: " << nodesOnPath[i].id() << " x: " << nodesOnPath[i].x() << " y: " << nodesOnPath[i].y();
#endif  // VRPMINTRACE
    }

    // Add point to the call
    osrmi->addViaPoint(call);
    if (!osrmi->getOsrmViaroute()) {
#ifdef VRPMINTRACE
        DLOG(INFO) << "getOsrmViaroute failed";
#endif  // VRPMINTRACE
        osrmi->useOsrm(oldStateOsrm);
        return;
    }

    // To store time returned
    std::deque< double > times;
    if (!osrmi->getOsrmTimes(times)){
#ifdef VRPMINTRACE
        DLOG(INFO) << "getOsrmTimes failed";
#endif  // VRPMINTRACE
        osrmi->useOsrm(oldStateOsrm);
        return;
    }

    // lets have a peek
#ifdef VRPMINTRACE
    DLOG(INFO) << "sequential for the times";
    for (unsigned int i= 0; i < call.size(); ++i) {
        DLOG(INFO) << call[i].type() << "," << call[i].nid() << "," << call[i].id() << "," << times[i];
    }
#endif  // VRPMINTRACE

    // Restore previous
    osrmi->useOsrm(oldStateOsrm);


    // fills the 2D table
    // Lo nuevo
    for (int i = 0; i < call.size()-1; ++i) {
        for (int j = i + 1; j < call.size(); ++j) {

            THROW_ON_SIGINT

                if ( !call[i].isPhantomNode() ) {
                    UINT from = call[i].nid();
                    if ( !call[j].isPhantomNode() ) {
                        UINT to = call[j].nid();
#ifdef VRPMINTRACE
                        DLOG(INFO) << "(i,j)=(" << i << "," << j << ")";
                        DLOG(INFO) << "(from,to)=(" << from << "," << to << ")";
#endif  // VRPMINTRACE
                        if (from != to) {
                            if (travel_time_onTrip[from][to] == 0) {
                                // Set
                                travel_time_onTrip[from][to] = times[j]-times[i];
                                travel_Time[from][to] = times[j]-times[i];
                                // Recalculate
                                nodes_onTrip[from][to].clear();
                                for (int k = i + 1; k < j; ++k) {
                                    if ( !call[k].isPhantomNode() ) {
                                        UINT nodeOnPath = call[k].nid();
                                        assert (nodeOnPath < original.size());
                                        if ( call[k].isPickup() ) {
#ifdef VRPMINTRACE
                                            DLOG(INFO) << "k=" << k;
                                            DLOG(INFO) << "nodeOnPath=" << nodeOnPath;
#endif  // VRPMINTRACE
                                            nodes_onTrip[from][to].push_back(nodeOnPath);
                                        }
                                    }
                                }

#ifdef VRPMINTRACE
                                for (unsigned int i = 0; i < nodes_onTrip[from][to].size(); ++i) {
                                    DLOG(INFO) << nodes_onTrip[from][to][i];
                                }
#endif  // VRPMINTRACE

                                continue;
                            }

                            if (travel_time_onTrip[from][to] > (times[j]-times[i])) {
#ifdef VRPMINTRACE
                                DLOG(INFO) << from << "," << to << " -> ";
                                DLOG(INFO) << " old value " << travel_time_onTrip[from][to];
                                DLOG(INFO) << " new value " << times[j]-times[i];
                                DLOG(INFO) << " ----> changed ";
#endif  // VRPMINTRACE

                                travel_time_onTrip[from][to] = times[j]-times[i];
                                travel_Time[from][to] = times[j]-times[i];

                                nodes_onTrip[from][to].clear();
                                for (int k = i + 1; k < j; ++k) {
                                    if ( !call[k].isPhantomNode() ) {
                                        UINT nodeOnPath = call[k].nid();
                                        assert (nodeOnPath < original.size());
                                        if ( call[k].isPickup() ) {
                                            nodes_onTrip[from][to].push_back(nodeOnPath);
                                        }
                                    }
                                }


#ifdef VRPMINTRACE
                                for (unsigned int i = 0; i < nodes_onTrip[from][to].size(); ++i) {
                                    DLOG(INFO) << nodes_onTrip[from][to][i];
                                }
#endif  // VRPMINTRACE
                            }
                        }
                    }
                }
        }
    }


#ifdef VRPMINTRACE
    DLOG(INFO) << "Updated travel_Time";
    int rcSize = original.size();
    DLOG(INFO) << "Begin travel_Time matrix";
    for ( int i = 0; i < rcSize; i++ ) {
        std::stringstream row;
        for ( int j = 0; j < rcSize; j++ ) {
            row << "\t"<< travel_Time[i][j];
        }
        DLOG(INFO) << row.str() << std::endl;
    }
    DLOG(INFO) << "End travel_Time matrix";
#endif  // VRPMINTRACE

#ifdef VRPMINTRACE
    DLOG(INFO) << "ended fill_times";
#endif  // VRPMINTRACE

}









/*
 *
 *
 * NEW CODE
 *   use phantomnodes
 */
bool 
TWC::setTravelingTimesInsertingOneNode(
        const TwBucket &truck,
        const Trashnode &dumpSite,
        const Trashnode &node) const {
#ifdef VRPMINTRACE
    DLOG(INFO) << "started setTravelingTimesInsertingOneNode";
#endif



    bool oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);  //forcing osrm usage

    if (truck.size() == 1) {
        TravelTime(truck[0], node, dumpSite);
        osrmi->useOsrm(oldStateOsrm);
        return false;
    }

    // buld call
    unsigned int tSize = truck.size();
    osrmi->clear();

    std::deque< Twnode > call;
    std::deque< double > times;
    int id;

    // special case  0 n 1
    call.push_back(truck[0]);
    // Phantom
    id = truck[0].id();
    auto it = mPhantomNodes.find( id );
    if ( it!=mPhantomNodes.end() ) {
        // Put mPNId for Node
        Twnode n = Twnode(mIPNId,mPNId,it->second.afterPNode().x(),it->second.afterPNode().y());
        n.set_type( Twnode::kPhantomNode );
        call.push_back(n);
    }

    // Got phantom
    id = node.id();
    it = mPhantomNodes.find( id );
    if ( it!=mPhantomNodes.end() ) {
        // Put mPNId for Node
        Twnode n = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
        n.set_type( Twnode::kPhantomNode );
        call.push_back(n);
    }
    call.push_back(node);

    // Got phantom
    id = truck[1].id();
    it = mPhantomNodes.find( id );
    if ( it!=mPhantomNodes.end() ) {
        // Put mPNId for Node
        Twnode n = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
        n.set_type( Twnode::kPhantomNode );
        call.push_back(n);
    }
    call.push_back(truck[1]);

    // cycle:
    if (tSize > 2) {
        for (unsigned int i= 0; i < tSize - 3; ++i) {
            call.push_back(truck[i]);
            call.push_back(truck[i+1]);
            call.push_back(node);
            call.push_back(truck[i+2]);
        }
    }
    // special case 5 6 n   // 0 1 n D
    call.push_back(truck[tSize - 2]);
    call.push_back(truck[tSize - 1]);
    call.push_back(node);
    call.push_back(dumpSite);

    // process osrm
    osrmi->addViaPoint(call);
    if (!osrmi->getOsrmViaroute()) {
        DLOG(INFO) << "getOsrmViaroute failed";
        osrmi->useOsrm(oldStateOsrm);
        return false;
    }
    if (!osrmi->getOsrmTimes(times)){
        DLOG(INFO) << "getOsrmTimes failed";
        osrmi->useOsrm(oldStateOsrm);
        return false;
    }


    // lets have a peek
#ifdef VRPMAXTRACE
    DLOG(INFO) << "squential";
    for (unsigned int i= 0; i < call.size(); ++i) {
        DLOG(INFO) << call[i].id() << "," << times[i];
    }
#endif


#ifdef VRPMAXTRACE
    DLOG(INFO) << "pairs";
#endif
    for (unsigned int i = 0; i < call.size()-1; ++i) {
        TravelTime(call[i].nid(), call[i+1].nid());
        travel_Time[call[i].nid()][call[i+1].nid()] = times[i+1]-times[i];
#ifdef VRPMAXTRACE
        DLOG(INFO) << call[i].id() << " -> "
            << call[i+1].id() << " = " << times[i+1] - times[i];
#endif
    }


    osrmi->useOsrm(oldStateOsrm);

#ifdef VRPMINTRACE
    DLOG(INFO) << "ended setTravelingTimesInsertingOneNode";
#endif

}



/*
 * NEW CODE use phantomnodes
 */

bool 
TWC::setTravelingTimesOfRoute(
        const TwBucket &truck,
        const Trashnode &dumpSite) const {
#ifdef VRPMINTRACE
    DLOG(INFO) << "started setTravelingTimesOfRoute";
#endif


    bool oldStateOsrm = osrmi->getUse();
    osrmi->useOsrm(true);  //forcing osrm usage

    // buld call
    unsigned int tSize = truck.size();
    std::deque< Node > call;
    std::deque< double > times;
    osrmi->clear();

    // cycle 1:
    UID id;
    for (unsigned int i = 0; i < tSize; ++i) {
        id = truck[i].id();
        auto it = mPhantomNodes.find( id );
        if ( i==0 ) {
            call.push_back(truck[i]);
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.afterPNode().x(),it->second.afterPNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
        } else {
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
            call.push_back(truck[i]);
        }
        // call.push_back(truck[i]);
    }

    //Push Dump Site
    call.push_back(dumpSite);

    // cycle 2:

    //Push Dump Site
    call.push_back(dumpSite);
    for (int i = tSize - 1; i >= 0; --i) {
        id = truck[i].id();
        auto it = mPhantomNodes.find( id );
        if ( i == (tSize - 1) ) {
            call.push_back(truck[i]);
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.afterPNode().x(),it->second.afterPNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
        } else {
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
            call.push_back(truck[i]);
        }
        // call.push_back(truck[i]);
    }

    //Push Departure Site
    call.push_back(truck[0]);

    // process osrm
    osrmi->addViaPoint(call);
    if (!osrmi->getOsrmViaroute()) {
        DLOG(INFO) << "getOsrmViaroute failed";
        osrmi->useOsrm(oldStateOsrm);
        return false;
    }
    if (!osrmi->getOsrmTimes(times)){
        DLOG(INFO) << "getOsrmTimes failed";
        osrmi->useOsrm(oldStateOsrm);
        return false;
    }

    // lets have a peek
#ifdef VRPMINTRACE
    DLOG(INFO) << "squential";
    for (unsigned int i= 0; i < call.size(); ++i) {
        DLOG(INFO) << call[i].id() << "->" << times[i];
    }
#endif

#ifdef VRPMINTRACE
    DLOG(INFO) << "setting travel_Time (pairs). call.size = " << call.size();
#endif
    // Take only nodes not PhantomNodes
    for (int i = 0; i < call.size()-1; ++i) {
        //TravelTime(call[i].nid(), call[i+1].nid());
        if ( !call[i].isPhantomNode() ) {
            for (int j = i + 1; j < call.size(); ++j) {
#ifdef VRPMINTRACE
                DLOG(INFO) << "(i,j)=(" << i << "," << j << ") | time=" << times[j] - times[i];
#endif
                if ( !call[j].isPhantomNode() ) {
#ifdef VRPMINTRACE
                    DLOG(INFO) << "call[" << i << "].nid() = " << call[i].nid() << "|" "call[" << j << "].nid() = " << call[j].nid();
#endif
                    travel_Time[call[i].nid()][call[j].nid()] = times[j]-times[i];
#ifdef VRPMINTRACE
                    DLOG(INFO) << "travel_Time from point " << call[i].id() << " to point " << call[j].id() << " is " << times[j] - times[i];
#endif
                }
            }
        }
    }
    osrmi->useOsrm(oldStateOsrm);
}

void foo () {
    /***************************************************************
     *
     *  THIS CODE NEEDS REWRITE
     *
     ***************************************************************/


    // En calle de dos vias
    // Para el truck[0] debo imponer que vaya luego al afterPNode
    // Para el resto que pase por beforePNode
    //
#ifdef VRPMAXTRACE
    DLOG(INFO) << "mPhantomNodes tiene " << mPhantomNodes.size() << " elementos";
#endif  // VRPMAXTRACE
    UID id;
    for (unsigned int i = 0; i < truck.size(); ++i) {
        id = truck[i].id();
        auto it = mPhantomNodes.find( id );
        if ( i==0 ) {
            call.push_back(truck[i]);
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.afterPNode().x(),it->second.afterPNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
        } else {
            if ( it!=mPhantomNodes.end() ) {
                // Put mPNId for Node
                Twnode n = Twnode(mIPNId,mPNId,it->second.beforePNode().x(),it->second.beforePNode().y());
                n.set_type( Twnode::kPhantomNode );
                call.push_back(n);
            }
            call.push_back(truck[i]);
        }
        // call.push_back(truck[i]);
#ifdef VRPMAXTRACE
        std::ostringstream strs;
        strs << "Nodos en el truck:" << i << " ID: " << truck[i].id() << " x: " << truck[i].x() << " y: " << truck[i].y();
        DLOG(INFO) << strs.str();
#endif  // VRPMAXTRACE
    }

    // Add dumpsite
    call.push_back(dumpSite);

    // Add points to osrm
    osrmi->addViaPoint(call);
}


#endif  // DOREWRITE


