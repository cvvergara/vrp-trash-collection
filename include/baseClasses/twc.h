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
#include <cmath>
#include <algorithm>
#include <utility>
// #include <math.h>
// #include <limits>
#include <iomanip>      // std::setprecision

#ifdef DOVRPLOG
#include "logger.h"
#endif

#ifdef OSRMCLIENT
#include "osrmclient.h"
#include "phantomnode.h"
#endif

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif

#include "basictypes.h"
#include "node.h"
#include "twnode.h"
#include "twpath.h"
#include "singleton.h"
#include "pg_types_vrp.h"
#include "signalhandler.h"

typedef std::map<UID,PhantomNode> PhantomNodes;   ///< Type definiton for PahntomNodes.

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
        // CompareSecond class
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

        struct classcomp {
            bool operator() (const TTindex &lhs, const TTindex &rhs) const {
                return lhs.prev() < rhs.prev() ? true : lhs.from() < rhs.from() ? true : lhs.middle() <
                    rhs.middle() ? true : lhs.to() < rhs.to();
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
        void setPhantomNodes() {

#ifdef REWRITE
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


        typedef TwBucket<knode> Bucket;
        TwBucket<knode> original;                                 ///< The nodes. Set in loadAndProcess_distance and setNodes (Not used anywhere!)
        std::map< std::string, int> streetNames;                  ///< Map with street name and id
        mutable std::vector< std::vector<double> > twcij;
        mutable std::vector< std::vector<double> > travel_Time;   ///< Travel time matrix
        mutable std::vector< std::vector<double> > travel_time_onTrip;
        mutable std::vector< std::vector< std::deque< size_t > > > nodes_onTrip;  ///< The nodes on trip

        typedef std::pair< std::pair <UINT, UINT>,  double> id_time;
        // typedef pair<UINT, double>::iterator i_id_time;
        std::set<id_time, CompareSecond< id_time > >  process_order;
        std::set<id_time, CompareSecond< id_time > >  process_order_far;

    public:
        /*! \brief cleans all the tables, leaving them blank for a next execution */
        void cleanUp() {
            original.clear();
            twcij.clear();
            travel_Time.clear();
            travel_time_onTrip.clear();
            nodes_onTrip.clear();
            process_order.clear();
            process_order_far.clear();
            mPhantomNodes.clear();
        }


        bool emptiedTruck = false;
        int z1Tot = 0;
        int z2Tot = 0;

        void initializeTravelTime() {
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

        void getProcessOrder() {
            DLOG(INFO) << "--> getProcessOrder";

            for (const auto &source : original.path) {
                for (const auto &sink : original.path) {

                    if (source.nid() == sink.nid()) assert(travel_Time[source.nid()][sink.nid()] == 0);

                    if (travel_Time[source.nid()][sink.nid()] == -1) {
                        process_order_far.insert(
                                std::make_pair(std::make_pair(source.nid(), sink.nid()),
                                    30));  // source.haversineDistance(sink) / 250));
                    } else {
                        process_order.insert(
                                std::make_pair(std::make_pair(source.nid(), sink.nid()),
                                    travel_Time[source.nid()][sink.nid()]));
                    }
                }
            }
            DLOG(INFO) << "<-- getProcessOrder";
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
    public:
        bool findNearestNodeTo(const TwBucket<knode> &truck,
                const  TwBucket<knode> &unassigned,
                POS &pos,
                knode &bestNode,
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
        bool  findNearestNodeUseExistingData(const TwBucket<knode> &truck,
                const  TwBucket<knode> &unassigned,
                POS &pos,
                knode &bestNode,
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

            if (!flag) return findNearestNodeTo(truck, unassigned, pos, bestNode
                    , bestDist);

            return flag;
        }


        // TODO
    public:
        void  findBestToNodeHasMoreNodesOnPath(
                const TwBucket<knode> &assigned,
                const TwBucket<knode> &unassigned,
                UINT From, UINT &bestTo, TwBucket<knode> &subPath) const {
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

    public:
        float8 getTimeOverNodesCount(
                const knode &fromNode, const knode &toNode,
                const TwBucket<knode> &assigned,
                const TwBucket<knode> &subPath) const {
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

    public:
        float8 getTimeOnTrip(const knode from, const knode middle, const knode to) {
            return travel_time_onTrip[from.nid()][middle.nid()]
                + travel_time_onTrip[middle.nid()][to.nid()];
        }

    public:
        void  findBestFromNodeHasMoreNodesOnPath(
                const TwBucket<knode> &assigned,
                const TwBucket<knode> &unassigned,
                UINT &bestFrom, UINT To, TwBucket<knode> &subPath) const {
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

                if ((actual > max) || (max < 0)) {
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


    public:
        bool  findPairNodesHasMoreNodesOnPath(
                const TwBucket<knode> &assigned, const TwBucket<knode> &unassigned,
                UINT &bestFrom, UINT &bestTo, TwBucket<knode> &subPath) const {
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

                    if ((actual > max) || (max < 0)) {
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
                if (assigned.hasNid(nid)) continue;
                subPath.push_back(original[nid]);
            }
            subPath.push_back(original[bestTo]);
            return true;
        }







    public:
        bool  findNodeHasMoreNodesOnPath(const TwBucket<knode> &trip,
                const TwBucket<knode> &assigned, const TwBucket<knode> &unassigned,
                const knode &dumpSite, UINT &bestNode, UINT &bestPos, TwBucket<knode> &subPath) const {
            STATS_INC("findNodeHasMoreNodesOnPath");
            assert(unassigned.size() != 0);
            subPath.clear();
            TwBucket<knode> l_trip = trip;
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
                if (assigned.hasNid(nid)) continue;
                subPath.push_back(original[nid]);
            }
            subPath.push_back(original[bestNode]);
            for (unsigned int i = 0; i < nodes_onTrip[bestNode][bestNextNode].size(); ++i) {
                auto nid = nodes_onTrip[bestNode][bestNextNode][i];
                if (assigned.hasNid(nid)) continue;
                assert(nid < original.size());
                subPath.push_back(original[nid]);
            }

            TwBucket<knode> l_subPath = subPath;
            TwBucket<knode> m_assigned = assigned;
            l_subPath = l_subPath - m_assigned;
            for (UINT i = 0; i < subPath.size(); ++i)
                if (!l_subPath.hasNid(subPath[i])) {
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

        auto actualCantNodesOnTrip(UINT from, UINT to, const TwBucket<knode> &assigned) const {
            STATS_INC("actualCantNodesOnTrip");
            assert(from < original.size());
            assert(to < original.size());
            TwBucket<knode> subPath;
            TwBucket<knode> m_assigned = assigned;
            for (unsigned int i = 0; i < nodes_onTrip[from][to].size(); ++i) {
                UINT nid = nodes_onTrip[from][to][i];
                if (assigned.hasNid(nid)) continue;
                subPath.push_back(original[nid]);
            }
            subPath = subPath - m_assigned;
            assert(subPath.size() <= nodes_onTrip[from][to].size());
            return subPath.size();
        }

        TwBucket<knode> actualNodesOnTrip(UINT from, UINT to, const TwBucket<knode> &assigned) const {
            assert(from < original.size());
            assert(to < original.size());
            TwBucket<knode> subPath;
            TwBucket<knode> m_assigned = assigned;
            for (unsigned int i = 0; i < nodes_onTrip[from][to].size(); ++i) {
                UINT nid = nodes_onTrip[from][to][i];
                if (assigned.hasNid(nid)) continue;
                subPath.push_back(original[nid]);
            }
            TwBucket<knode> l_subPath = subPath;
            l_subPath = l_subPath - m_assigned;
            for (UINT i = 0; i < subPath.size(); ++i)
                if (!l_subPath.hasNid(subPath[i])) {
                    subPath.erase(i);
                    --i;
                }
            assert(subPath.size() <= nodes_onTrip[from][to].size());
            return subPath;
        }

    public:
        void fill_travel_time_onTrip() {
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

            if (original.size() < 500)
                fill_travel_time_onTrip_work(process_order_far);

            fill_travel_time_onTrip_work(process_order);
#ifdef VRPMINTRACE
            DLOG(INFO) << "ended fill_travel_time_onTrip";
#endif
        }

        void fill_travel_time_onTrip_work(std::set<id_time, CompareSecond< id_time > >  &process_order) {
#ifdef VRPMINTRACE
            DLOG(INFO) << "started fill_travel_time_onTrip_work";
#endif
            TwBucket <knode> trip;
            TwBucket <knode> nodesOnPath;
#ifdef VRPMINTRACE
            DLOG(INFO) << "fill_travel_time_onTrip doing " <<  " size " << process_order.size() << "\n";
#endif

            while (process_order.size() > 0) {
                UINT i = process_order.begin()->first.first;
                UINT j = process_order.begin()->first.second;
#ifdef VRPMINTRACE
                if ((process_order.size() % 200) == 0)
                    DLOG(INFO) << "fill_travel_time_onTrip " << i << " size " << process_order.size() << " working with " << original[i].id() << "," << original[j].id()
                        << " onTrip time" << travel_time_onTrip[i][j]
                        << " on data time" << travel_Time[i][j]
                        << " onTrip time" << travel_time_onTrip[j][i]
                        << " on data time" << travel_Time[j][i] << "\n";
#endif

                process_order.erase(process_order.begin());

                if (i == j) {
                    travel_time_onTrip[i][j] = 0;
                    continue;
                }
                process_pair_onPath(original[i],original[j]);
                process_pair_onPath(original[j],original[i]);
            } // while

            // assert(true==false);
#ifdef VRPMINTRACE
            int count=0;
            for (int ii = 0; ii < travel_time_onTrip.size(); ++ii) {
                for (int jj = 0; jj < travel_time_onTrip.size(); ++jj) {
                    if (travel_time_onTrip[ii][jj] != 0) count++;
                    if (travel_time_onTrip[ii][jj] != travel_Time[ii][jj]) {
                        DLOG(INFO) << original[ii].id() << "," << original[jj].id() << " -> " << travel_time_onTrip[ii][jj]
                            << " with " << nodes_onTrip[ii][jj].size();
                        DLOG(INFO) << original[ii].id() << "," << original[jj].id() << " -> tt " << travel_Time[ii][jj];
                    }
                }
            }
            DLOG(INFO) << "total times values got: " << count;
#endif

#ifdef VRPMINTRACE
            DLOG(INFO) << "end fill_travel_time_onTrip_work";
#endif
            return;
        }

    private:
        double
            fn_travel_time_onTrip(knode from, knode to) const {
                return travel_time_onTrip[from.nid()][to.nid()];
            } 


    private:
        void process_pair_onPath(knode from, knode to) const {
            STATS_INC("--> process_pair_onPath(knode from, knode to)");

            /*
             * nothing to do
             */
            if (from.nid() == to.nid()) return;

            assert(travel_time_onTrip.size() == original.size());

            TwBucket <knode> trip;
            TwBucket <knode> nodesOnPath;
            if (fn_travel_time_onTrip(from, to) == 0) {
                trip.clear();
                nodesOnPath.clear();

                trip.push_back(from);

                getNodesOnPath(trip, to, original, nodesOnPath);

                if (nodesOnPath.size() == 0 || nodesOnPath.front().nid() != from.nid()) {
                    nodesOnPath.push_front(from);
                }

                if (nodesOnPath.back().nid() != to.nid()) {
                    nodesOnPath.push_back(to);
                }

                fill_times(nodesOnPath);
            }
        }


    private:
        // the values for non containers to/from containers should be filled
        void compulsory_fill() {
            STATS_INC("compulsory_fill");
            DLOG(INFO) << "started compulsory_fill";

            assert(!original.empty());

            for (auto &special_node : original.path) {
                /*
                 * Cycle only on dump start ending and average nodes
                 */
                if (special_node.isPickup()) continue;

                for (auto &container : original.path) {
                    /*
                     * Cycle only on pickups (containers)
                     */
                    if (!container.isPickup()) continue;

                    process_pair_onPath(special_node, container);
                    process_pair_onPath(container, special_node);
                }
            }

            DLOG(INFO) << "Ended compulsory_fill";
        }










#ifdef REWRITE
        /*
         * NEW CODE BY FERNANDO
         * uses phantomnodes
         *
         *
         */
    private:
        void fill_times(const TwBucket<knode> nodesOnPath) const {

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








#else  //REWRITE








        /******************************
         *
         *  old code
         *
         ******************************/


    private:
        void fill_times(const TwBucket<knode> nodesOnPath) const {
            STATS_INC(" --> fill_times");

            // using osrm
            bool oldStateOsrm = osrmi->getUse();
            osrmi->useOsrm(true);
            osrmi->clear();

            // build call
            std::deque< Node > call;
            for (const auto &n : nodesOnPath.path) call.push_back(n);

            osrmi->addViaPoint(call);
            if (!osrmi->getOsrmViaroute()) {
                DLOG(WARNING) << "getOsrmViaroute failed";
                osrmi->useOsrm(oldStateOsrm);
                return;
            }
            std::deque< double > times;
            if (!osrmi->getOsrmTimes(times)){
                DLOG(WARNING) << "getOsrmTimes failed";
                osrmi->useOsrm(oldStateOsrm);
                return;
            }


            osrmi->useOsrm(oldStateOsrm);


            // fills the 2D table
            for (size_t i = 0; i < nodesOnPath.size() - 1; ++i) {
                for (size_t  j = i + 1; j < nodesOnPath.size(); ++j) {

                    THROW_ON_SIGINT;

                    auto from = nodesOnPath[i].nid();
                    auto to = nodesOnPath[j].nid();

                    /*
                     * A time lapse can not be negative
                     */
                    if (times[j] - times[i] < 0) continue;

                    /*
                     * Nothing to do
                     */
                    if (from == to)  continue;

                    /*
                     *   - there is no previouus information
                     *   - or the calcualated time is better
                     */
                    if ((travel_time_onTrip[from][to] == 0) 
                            || (travel_time_onTrip[from][to] > (times[j]-times[i]))) {
                        /*
                         * Update the times
                         */
                        travel_Time[from][to] = travel_time_onTrip[from][to] = 
                            times[j] - times[i];

                        /*
                         *  update to found sequence of nodes
                         */
                        nodes_onTrip[from][to].clear();

                        for (size_t k = i + 1; k < j; ++k) {
                            /*
                             * Onnly inserting containers
                             */
                            if (nodesOnPath[k].isPickup())
                                nodes_onTrip[from][to].push_back(nodesOnPath[k].nid());
                        }
                    }
                }
            }
        }


#endif  // REWRITE  end of old code


    public:

        /*!
          From the unassigned bucket all nodes that are in the truck's path
          will be placed on streetNodes Bucket

          The caller is resposible of the contents of orderedStreetNodes
          */
        void getNodesOnPath(
                const TwBucket<knode> &truck,
                const knode &dumpSite,
                const TwBucket<knode> &unassigned,
                TwBucket<knode> &orderedStreetNodes) const {

            STATS_INC("USING getNodesOnPath");


            /*
             * Nothing to do
             */
            if (unassigned.empty()) return;



            /*
             * forcing use of osrm
             */
            auto oldStateOsrm = osrmi->getUse();
            osrmi->useOsrm(true);
            osrmi->clear();





            /***************************************************************
             *
             *  THIS CODE NEEDS REWRITE
             *
             ***************************************************************/


#ifdef REWRITE
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



#else  // ndef DOREWRITE

            /*
             * "original" code
             * VRPMAXTRACE:
             *   - generates http queries to be tested on the localhost
             *   - a sequence of original "id" to be ploted on a frontend
             */


            /*
             * building call
             */
            std::deque<Node> call;


#ifdef VRPMAXTRACE
            std::stringstream webcall;
            std::stringstream maproute;
            webcall << std::setprecision(10)
                << "http://localhost:5000/route/v1/driving/";
#endif  // VRPMAXTRACE
            for (const auto &n : truck.path) {
                call.push_back(n);
#ifdef VRPMAXTRACE
                webcall <<n.x() << "," << n.y() << ";";
                maproute << n.id() << " ";
#endif  // VRPMAXTRACE
            }

            /*
             * dumpSite
             */
            call.push_back(dumpSite);

#ifdef VRPMAXTRACE
            webcall <<dumpSite.x() << "," << dumpSite.y();
            webcall << "?geometries=geojson\n";
            maproute << dumpSite.id(); 

            STATS_INC(webcall.str());
            STATS_INC(maproute.str());
#endif  // VRPMAXTRACE

            osrmi->addViaPoint(call);

#endif  // DOREWRITE






            /*
             * calling osrm
             */
            if (!osrmi->getOsrmViaroute()) {
                DLOG(WARNING) << "getNodesOnPath getOsrmViaroute failed";
                osrmi->useOsrm(oldStateOsrm);
                return;
            }



            /*
             * getting the geometry
             */
            std::deque<Node> geometry;
            if (!osrmi->getOsrmGeometry(geometry)) {
                DLOG(WARNING) << "getNodesOnPath getOsrmGeometry failed";
                osrmi->useOsrm(oldStateOsrm);
                return;
            }



            /*
             * Nothing to do:
             *   the geometry returns the 2 points entered
             */
            if (geometry.size() == 2) return;



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
                    for (auto it = streetNodes.path.begin(); it != streetNodes.path.begin(); ++it) {
                        if ((*it).nid() == std::get<1>(d)) {
                            streetNodes.path.erase(it);
                            break;
                        }
                    }
                }

            }  //  next segment


            /************************************************************/
            osrmi->useOsrm(oldStateOsrm);
            osrmi->clear();
        }







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


#ifdef REWRITE


        /*
         * NEW CODE use phantomnodes
         */

        bool setTravelingTimesOfRoute(
                const TwBucket<knode> &truck,
                const knode &dumpSite) const {
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
#else  // REWRITE


        /*
         *
         * old code
         *
         */


        bool setTravelingTimesOfRoute(
                const TwBucket<knode> &truck,
                const knode &dumpSite) const {

            bool oldStateOsrm = osrmi->getUse();
            osrmi->useOsrm(true);  //forcing osrm usage

            // buld call
            unsigned int tSize = truck.size();
            std::deque< Node > call;
            std::deque< double > times;
            osrmi->clear();

            // cycle 1:
            for (const auto &n : truck.path)  call.push_back(n);
            call.push_back(dumpSite);

            // cycle 2:
            call.push_back(dumpSite);
            for (int i = tSize - 1; i >= 0; --i) {
                call.push_back(truck[i]);
            }

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



            for (unsigned int i = 0; i < call.size()-1; ++i) {
                travel_Time[call[i].nid()][call[i+1].nid()] = times[i+1]-times[i];
            }

            osrmi->useOsrm(oldStateOsrm);  
            return true;
        }



#endif //REWRITE


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



#ifdef REWRITE

        /*
         *
         *
         * NEW CODE
         *   use phantomnodes
         */
        bool setTravelingTimesInsertingOneNode(
                const TwBucket<knode> &truck,
                const knode &dumpSite,
                const knode &node) const {
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

#else  //REWRITE



        /*
         * old code
         */
        bool setTravelingTimesInsertingOneNode(
                const TwBucket<knode> &truck,
                const knode &dumpSite,
                const knode &node) const {


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

            std::deque< Node > call;
            std::deque< double > times;    
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
            return true;
        }





#endif //REWRITE





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
                const TwBucket<knode> &truck,
                TwBucket<knode> &unassigned,
                const knode &dumpSite,
                POS &pos,
                knode &bestNode,
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

            void setTravelTimeOsrm(UID from, UID to) const {
                assert(from < original.size() && to < original.size());
                double time;
                if (!osrmi->getConnection()) {
                    setTravelTimeNonOsrm(from, to);
                    return;
                }

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
                    return false;
                }

                if ( !osrmi->getOsrmTime(time) ){
                    DLOG(INFO) << "getOsrmTime failed";
                    osrmi->useOsrm(oldStateOsrm);
                    setTravelTimeNonOsrm(from, to);
                    return false;
                }

                osrmi->useOsrm(oldStateOsrm);

                travel_Time[from][to] = time;
#ifdef VRPMINTRACE
                DLOG(INFO) << "travel_Time[" << from << "][" << to << "]=" << time;
#endif
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


    private:
            double getTravelTime(UID from, UID to) const {
                assert(from < original.size() && to < original.size());
                if (travel_Time[from][to] == -1) {
#ifdef DOSTATS
                    STATS->inc("TWC::extra process_pair_onPath");
#endif

                    process_pair_onPath(original[from],original[to]);
                    process_pair_onPath(original[to],original[from]);
                }
                return travel_Time[from][to];
            }



    public:
            /*!  \brief Retruns travel time from node id \b from to node id \b to.
              interfaces
              */
            //@{
            double TravelTime(UID from, UID to) const {
                return getTravelTime(from,to);
            }

            double TravelTime(UID from, UID middle, UID to) const {
                assert(from < original.size());
                assert(middle < original.size());
                assert(to < original.size());
                TravelTime(from,to);
                return  TravelTime(from, middle) + TravelTime(middle, to);
            }


            double TravelTime(UID prev, UID from, UID middle, UID to) const {
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

            double TravelTime(const knode &from, const knode &to) const {
                return TravelTime(from.nid(), to.nid());
            }

            double TravelTime(const knode &from, const knode &middle, const knode &to) const {
                return TravelTime(from.nid(), from.nid(), middle.nid(), to.nid());
            }

            double TravelTime(const knode &prev, const knode &from, const knode &middle, const knode &to) const {
                return TravelTime(prev.nid(), from.nid(), middle.nid(), to.nid());
            }
            //@}

            bool isInPath(UINT from, UINT middle, UINT to) {
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

            bool isInPath(const knode &from, const knode &middle, const knode& to) {
                // DLOG(INFO) << from.nid() << "," << middle.nid() <<"," << to.nid();
                // DLOG(INFO) << TravelTime(from,to) << " ?? " << TravelTime(from,middle) << " + " << TravelTime(middle,to);
                return isInPath(from.nid(),middle.nid(),to.nid());
                //return TravelTime(from,to) >= (TravelTime(from,middle) + TravelTime(middle,to));
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
    public:
            //knode findBestTravelTime(UID from, const Bucket &nodes) const {
            bool findBestTravelTime(const knode &from, const Bucket &nodes, knode &bestNode) const {
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
                    const knode &from,
                    const Bucket &nodes,
                    Bucket &streetNodes,
                    knode &bestNode) const {
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

            void print_nodes_onTrip(size_t from, size_t to) const {
                std::stringstream ss;
                for (const auto node : nodes_onTrip[from][to]) {
                    ss << original[node].id() << " ";
                }
                DLOG(INFO) << ss.str();
            }

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
                original.dump("original");
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

#ifdef OSRMCLIENT
                setPhantomNodes();
#endif

            }



            /*!
             * \brief Compute the average travel time to a given node.
             *
             * \param[in] from A bucket of nodes to use as the start node.
             * \param[in] to A node to be used as the destination node.
             * \return The average travel time from start to destination.
             */
            double getAverageTime(const Bucket &from, const knode &to) const {
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
            double getAverageTime(const knode &from, const Bucket &to) const {
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



            /// \brief Sets the hints and the street's id to the nodes in the bucket
            /**

              The hint & street when id stored in "original" is copyied into the bucket's
              nodes.

              To be used after "original" has being filled with the appropiate values
              */
            void setHints(Bucket &nodes) {
#ifdef DOSTATS
                Timer timer;
#endif

                for (size_t i = 0; i < nodes.size(); i++ ) {
                    nodes[i].set_hint(original[nodes[i].nid()].hint());
                    nodes[i].set_streetId(original[nodes[i].nid()].streetId());
                }

#ifdef DOSTATS
                STATS->addto("TWC::setHints Cumultaive time:", timer.duration());
#endif
            }


    private:
            void prepareTravelTime() {
                int siz = original.size();
                travel_Time.resize(siz);
                //process_order.resize(siz);

                for ( int i = 0; i < siz; i++ )
                    travel_Time[i].resize(siz);

                // travel_Time default value is 250m/min
                for ( int i = 0; i < siz; i++ )
                    for ( int j = i; j < siz; j++ ) {
                        if ( i == j ) {
                            travel_Time[i][i] = 0.0;
                        } else {
                            travel_Time[i][j] = travel_Time[j][i] = -1.0;
                        }
                    }
            }


    private:
            void getAllHintsAndStreets() {
#ifdef DOSTATS
                Timer timer;
#endif  //DOSTATS

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

#ifdef DOSTATS
                STATS->addto("TWC::getAllHintsAndStreets Cumultaive time:", timer.duration());
#endif
            }


    public:

            // Just for log. Dump travel_Time matrix.
            void dump_travel_Time() {
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
                    const Bucket &datanodes, const Bucket &invalid) {
                assert(datanodes.size());
                // Delete previous
                original.clear();

                // datanodes is pickups + otherlocs
                original = datanodes;

#ifdef VRPMINTRACE
                original.dump("Data now loaded in original!");
#endif


                getAllHintsAndStreets();

                // Initialize travel_Time
                prepareTravelTime();

                // Read the data
                for (int i = 0; i < count; ++i) {
                    int from    = ttimes[i].from_id;
                    int to      = ttimes[i].to_id;
                    double time = ttimes[i].ttime;

                    if (invalid.hasId(from) || invalid.hasId(to)) continue;

                    // User pass Id, then, get internal nodeId (nid)
                    int fromId = getNidFromId(from);
                    int toId = getNidFromId(to);

                    if ( fromId == -1 || toId == -1 ) continue;

                    travel_Time[fromId][toId] = time;
                }

                // Print the actual travel_Time
#ifdef VRPMINTRACE
                DLOG(INFO) << "First travel_Time (reading from ttime_t)";
                dump_travel_Time();
#endif

                // ?????
                twcij_calculate();

                assert(original == datanodes);
                assert(check_integrity());

                // If OSRM, need phantom nodes!
#ifdef OSRMCLIENT
                setPhantomNodes();
#endif
            }


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
                    const Bucket &datanodes,
                    const Bucket &invalid) {
                STATS_INC("TWC::loadAndProcess_travelTimes");
                DLOG(INFO) << "loading travel times of " << datanodes.size() << " nodes"; 
                assert(!datanodes.empty());

#ifdef DOVRPLOG
                DLOG(INFO) << "TWC::loadAndProcess_travelTimes";
#endif

                original.clear();
                original = datanodes;
                getAllHintsAndStreets();
                DLOG(INFO) << "TWC::loadAndProcess_travelTimes";
                prepareTravelTime();
                DLOG(INFO) << "TWC::loadAndProcess_travelTimes";


                std::ifstream in(infile.c_str());
                std::string line;



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

                    if (invalid.hasId(from) || invalid.hasId(to)) continue;

                    fromId = getNidFromId(from);
                    toId = getNidFromId(to);

                    if ( fromId == -1 || toId == -1 ) continue;

                    travel_Time[fromId][toId] = time;
                }

                in.close();

                fill_travel_time_onTrip();

                setPhantomNodes();

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
            TWC()  {
                cleanUp();
            }
            TWC(const TWC &) {}
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


            /* public functions That are id based */


            /*!
             * \brief Compute all TWC values and populate the TWC matrix.
             */
            void twcij_calculate() {
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
            bool check_integrity() const {
                assert(original.size() == twcij.size());

                for (const auto row : twcij) assert(row.size() == original.size());

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
