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

#include <deque>
#include <set>
#include <string>
#include <algorithm>


#include "baseClasses/logger.h"

#include "baseClasses/basictypes.h"
#include "baseClasses/vrp_assert.h"
#include "nodes/trashnode.h"
#include "baseClasses/twc.h"
#include "baseClasses/twbucket.h"

class Prob_trash;




using namespace vrptc ;
using namespace vrptc::nodes;



double  
TwBucket::timePCN(const Trashnode &prev, const Trashnode &from, const Trashnode &middle,
        const Trashnode &to, double travelTimePrevFrom) const
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

double  
TwBucket::timePCN(POS from, POS middle, POS to) const
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

double 
TwBucket::timePCN(POS from, POS middle, const Trashnode &dump) const
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

double  
TwBucket::timePCN(POS from, const Trashnode &middle, const Trashnode &dump) const
{
    assert(from < path.size());

    if ( from == 0 )
        return timePCN(path[from], path[from], middle, dump, 0);
    else
        return timePCN(path[from - 1], path[from], middle , dump,
                path[from].travelTime());
}

double 
TwBucket::timePCN(POS from, const Trashnode &middle) const
{
    assert((from + 2) < path.size());

    if ( from == 0 )
        return timePCN(path[from], path[from], middle, path[from + 2], 0 );
    else
        return timePCN(path[from - 1], path[from], middle , path[from + 2],
                path[from].travelTime());
}

double 
TwBucket::timePCN(const Trashnode &node, const Trashnode &dump) const
{
    Trashnode last = path[path.size() - 1];
    return timePCN(path.size()-1, node, dump);
}

inline double 
TwBucket::TravelTime(const Trashnode &from, const Trashnode &to) const
{
    return twc.TravelTime(from.nid(), to.nid());
}
double 
TwBucket::TravelTime(const Trashnode &from, const Trashnode &middle,
        const Trashnode &to) const
{
    return twc.TravelTime(from.nid(), middle.nid() , to.nid());
}
double 
TwBucket::TravelTime(const Trashnode &prev, const Trashnode &from, const Trashnode &middle,
        const Trashnode &to) const
{
    return twc.TravelTime(prev.nid(), from.nid(), middle.nid() , to.nid());
}

double 
TwBucket::TravelTime(UID i, UID j) const
{
    return twc.TravelTime(i, j);
}
double 
TwBucket::TravelTime(UID i, UID j, UID k) const
{
    return twc.TravelTime(i, j, k);
}
double 
TwBucket::TravelTime(UID i, UID j, UID k, UID l) const
{
    return twc.TravelTime(i, j, k, l);
}

double 
TwBucket::segmentDistanceToPoint(POS pos, const Trashnode &node) const
{
    assert(path.size() > 1);
    assert(pos + 1 < path.size());
    return node.distanceToSegment(path[pos], path[pos + 1]);
}


#ifdef DOVRPLOG
void 
TwBucket::dumpid(const std::string &title) const
{
    std::stringstream ss;
    ss << title;
    for (const auto e : path) ss << " " << e.id();
    DLOG(INFO) << ss.str();
}


void 
TwBucket::dump(const std::string &title) const
{
    DLOG(INFO) << title;
    for (const auto e : path) e.dump();
    DLOG(INFO) << " <----- end \n";
}
#endif



/*
 * To have or not to have
 */
bool 
TwBucket::hasId(int64_t id) const {
    return std::find_if(path.begin(), path.end(),
            [&id](const auto &item) {
            return item.id() == id;
            }) != path.end();

}

bool 
TwBucket::hasNode(const Trashnode &node) const { return !(
        std::find_if(path.begin(), path.end(),
            [&node] (const Trashnode &e) 
            {return e.nid() == node.nid();})
        ==  path.end());
}

/*
*/

bool 
TwBucket::operator ==(const TwBucket &other) const
{
    if ( size() != other.size() ) return false;

    if ( size() == 0 && other.size() == 0 ) return true;

    if ( ((*this) - other).size() != 0 ) return false;

    if ( (other - (*this)).size() != 0 ) return false;

    return true;
}


TwBucket  
TwBucket::operator +(const TwBucket &other) const
{
    std::set<Trashnode, compNode> a;
    a.insert(path.begin(), path.end());
    a.insert(other.path.begin(), other.path.end());
    TwBucket b;
    b.path.insert(b.path.begin(), a.begin(), a.end());
    return b;
}

TwBucket 
TwBucket::operator *(const TwBucket &other) const
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

TwBucket 
TwBucket::operator -(const TwBucket &other) const
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
const Trashnode& 
TwBucket::last() const
{
    assert(size());
    return  path[size() - 1];
}

double 
TwBucket::getTotTravelTime() const
{
    assert(size());
    return last().totTravelTime();
}

double 
TwBucket::duration() const
{
    assert(size());
    return last().duration();
}

double 
TwBucket::totWaitTime() const
{
    assert(size());
    return last().totWaitTime();
}

double 
TwBucket::totServiceTime() const
{
    assert(size());
    return last().totServiceTime();
}

int 
TwBucket::dumpVisits() const
{
    assert(size());
    return last().dumpVisits();
}

double 
TwBucket::departureTime() const
{
    assert(size());
    return last().departureTime();
}

int 
TwBucket::twvTot() const
{
    assert(size());
    return last().twvTot();
}

int 
TwBucket::cvTot() const
{
    assert(size());
    return last().cvTot();
}

double 
TwBucket::cargo() const
{
    assert(size());
    return last().cargo();
}

bool 
TwBucket::feasable() const
{
    assert(size());
    return last().feasable();
}

bool 
TwBucket::feasable(double cargoLimit) const {
    assert(size());
    return last().feasable(cargoLimit);
}

bool
TwBucket::has_twv() const {
    assert(size());
    return last().has_twv();
}

bool 
TwBucket::has_cv(double cargoLimit) const {
    assert(size());
    return last().has_cv(cargoLimit);
}

size_t 
TwBucket::getNidFromId(int64_t id) const {

    return std::find_if(path.begin(), path.end(),
            [&id](const auto &item) {
            return item.id() == id;
            })->nid();

}

POS 
TwBucket::posFromId(int64_t id) const
{
    return std::find_if(path.begin(), path.end(),
            [&id](const auto &item) {
            return item.id() == id;
            }) - path.begin();

}


POS 
TwBucket::pos(const Trashnode &node) const
{return pos(node.nid());}

POS 
TwBucket::pos(UID nid) const
{
    return std::find_if(path.begin(), path.end(),
            [&nid](const auto &item) {
            return item.nid() == nid;
            }) - path.begin();
}


void 
TwBucket::swap(POS i, POS j) {
    std::iter_swap(this->path.begin() + i, this->path.begin() + j);
}

    bool 
TwBucket::swap(POS b1_pos, TwBucket &bucket2, POS b2_pos)
{
    assert(b1_pos < size() && b2_pos < bucket2.size());
    std::iter_swap(path.begin() + b1_pos, bucket2.path.begin() + b2_pos);
    return true;
}


    void 
TwBucket::move(int fromi, int toj)
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

std::deque<int> 
TwBucket::getpath() const
{
    std::deque<int> p;

    for (const auto e: path)
        p.push_back(e.nid());

    return p;
}


    bool 
TwBucket::insert(const Trashnode &node, POS atPos)
{
    assert(atPos <= path.size());
    path.insert(path.begin() + atPos, node);
    return true;
}

    bool 
TwBucket::insert(const TwBucket &nodes, POS atPos)
{
    assert(atPos <= path.size());
    for (UINT i = 0; i < nodes.size(); i++) {
        path.insert(path.begin() + atPos + i, nodes[i]);
    } 
    return true;
}


    bool 
TwBucket::erase(POS atPos)
{
    assert(atPos < path.size());
    path.erase(path.begin() + atPos);
    return true;
}


    void 
TwBucket::erase(const Trashnode &node)
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
}


#ifdef USE
    bool 
TwBucket::erase(POS fromPos, POS toPos)
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
#endif  //USE

    bool 
TwBucket::push_back(const Trashnode &node)
{
    path.push_back(node);
    return true;
}
    bool 
TwBucket::push_front(const Trashnode &node)
{
    path.push_front(node);
    return true;
}
Trashnode& 
TwBucket::operator[](POS at) {
    assert(at < path.size());
    return path[at];
}
const Trashnode&
TwBucket::operator[] (POS at) const
{
    assert(at < path.size());
    return path[at];
}
    Trashnode& 
TwBucket::at(POS pos)
{
    assert(pos < path.size());
    return path.at(pos);
}
const Trashnode& 
TwBucket::at(POS pos) const
{
    assert(pos < path.size());
    return path.at( pos );
}
