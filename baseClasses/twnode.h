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
#ifndef TWNODE_H
#define TWNODE_H

#include <string>

#include "node.h"

class Twnode: public Node {
  protected:
    int    type;
    double demand;
    double tw_open;
    double tw_close;
    double serviceTime;
    int streetid;

  public:
    // accessors
    double opens() const {return tw_open;};
    double closes() const {return tw_close;};
    double getdemand() const{ return demand;};
    double getservicetime() const{  return serviceTime;};
    double windowlength() const { return  tw_close - tw_open; };
    int ntype() const {return type;};
    int streetId() const {return streetid;};
    void dump() const;

    // state
    bool isvalid() const;
    bool hasdemand() const { return getdemand()>0; };
    bool hassupply() const { return getdemand()<0; };
    bool hasnogoods() const { return getdemand()==0; };
    bool earlyarrival(const double arrivalTime) const { return arrivalTime < tw_open; };
    bool latearrival(const double arrivalTime) const { return arrivalTime > tw_close; };
    bool sameStreet (const Twnode &other) const {return streetid==other.streetid; };

    // mutators
    void set(int _nid, int _id, double _x, double _y, int _demand,
             int _tw_open, int _tw_close, int _service);
    void setDemand(int _demand) { demand = _demand; };
    void setType(int _type) { type = _type; };
    void setOpens(int _tw_open) { tw_open = _tw_open; };
    void setCloses(int _tw_close) { tw_close = _tw_close; };
    void setServiceTime(int _service) { serviceTime = _service; };
    void setStreetId(int _sid) { streetid = _sid; };

    // structors

    Twnode() {
        Node();
        type=-1;
        demand = 0;
        tw_open = 0;
        tw_close = 0;
        serviceTime = 0;
    };

    Twnode(std::string line);

    ~Twnode() {};

};
#endif
