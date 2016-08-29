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
#include <map>
#include <math.h>

#include "solverTrash/tripVehicle.h"
#include "baseClasses/twpath.h"
#include "baseClasses/twc.h"
#include "nodes/phantomnode.h"

using namespace vrptc;
using namespace vrptc::nodes;


class Prob_trash {

// Children class can acces to protected!
protected:
  typedef unsigned long int UID;
  typedef unsigned long int POS;
  typedef unsigned long int UINT;
  inline double _MAX() {return (std::numeric_limits<double>::max()); }
  inline double _MIN() {return (-std::numeric_limits<double>::max()); }

  //    Trashnode depot;
  Twpath datanodes;              ///< Pickups + otherlocs ()
  TwBucket otherlocs;                         ///< Dumps and depts information.
  TwBucket dumps;                             ///< Dumps sites.
  TwBucket depots;                            ///< PahntomNode information for pickups. UID is the id of pickups nodes.
  TwBucket pickups;                           ///< Clients. Containers.
  std::map<UID,PhantomNode> phantomNodes;   ///< PahntomNode information for pickups. UID is the id of pickups nodes.
  TwBucket endings;
  TwBucket invalid;
  std::deque<Vehicle> trucks;               ///< Veihcles information
  std::deque<Vehicle> invalidTrucks;
  Trashnode C;                              ///< Nodo que tiene valores medios de coordenadas cargadas

  std::string datafile;


public:
  void clear() {
    otherlocs.clear();
    dumps.clear();
    depots.clear();
    pickups.clear();
    endings.clear();
    invalid.clear();
    trucks.clear();
    invalidTrucks.clear();
  }

  //    Trashnode getdepot() const { return depot;};
  Prob_trash() {}
  Prob_trash(const char *infile);
  Prob_trash(const std::string &infile);
  void loadProblem(const std::string &infile);

  unsigned int getNodeCount() const { return datanodes.size(); }

  bool checkIntegrity() const;


  double distance(int n1, int n2) const;
  double nodeDemand(int i) const;
  double nodeServiceTime(int i) const;
  bool earlyArrival(int nid, double D) const;
  bool lateArrival(int nid, double D) const;

  void twcijDump() const;
  /*!
   * \brief Get the phantom node for a pickup site
   *
   * \param[in] Node id of the pickup site
   * \param[out] The Phatom Node.
   * \return true on succes.
   */
  bool pickupHasPhantomNode (UID nodeId, PhantomNode &oPhantomNode) {
      bool ret = false;
      if( phantomNodes.find(nodeId) != phantomNodes.end()) {
          ret = true;
      }
      oPhantomNode = phantomNodes[nodeId];
      return ret;
  }

#ifdef DOVRPLOG
  void nodesdump() const;
  void nodesdumpeval() const;
  void dump() const;
#if 0
  void dumpdataNodes() const;
  void dumpDepots() const;
  void dumpDumps() const;
  void dumpPickups() const;
#endif
#endif

#ifdef DOPLOT
  void plot(Plot<Trashnode> &graph);
#endif


private:
  void load_depots( std::string infile );
  void load_dumps( std::string infile );
  void load_pickups( std::string infile );
  void load_endings( std::string infile );
  void load_otherlocs( std::string infile );
  void load_trucks( std::string infile );
  /*!
   * \brief Set phantom nodes for pickups sites
   *
   * From pickup sites, this function use nearest and locate -from OSRM-
   * and set the phantom node for pickups sites (in two ways segments) and
   * before and after points to force right side pickup.
   *
   */
  void setPhantomNodesForPickups();
};

#endif
