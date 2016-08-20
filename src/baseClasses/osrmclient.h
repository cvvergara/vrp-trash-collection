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
#ifndef VRP_OSRMCLIENT_H
#define VRP_OSRMCLIENT_H

#include <string>
#include <deque>
#include <vector>

#include <osrm/coordinate.hpp>
#include <osrm/osrm.hpp>
#include <engine/api/route_parameters.hpp>
#include <osrm/json_container.hpp>

#ifdef DOVRPLOG
#include "logger.h"
#endif

#if 0
#include "node.h"
#include "tweval.h"
#endif
// #include "twnode.h"


// load our assert to throw macros and tell rapidjson to use them
#include "vrp_assert.h"
#define RAPIDJSON_ASSERT assert
#include <rapidjson/document.h>

#include "timer.h"
#include "stats.h"

class Tweval;
class Node;

/*! \class OsrmClient
 * \brief This class provides a shared memory connection to OSRM.
 *
 * This class interfaces with OSRM via a shared memory connection and wraps
 * the interface into a simple class to abstract the features we need
 * access to. This interface is approximately 50 time faster than using
 * the URL based interface.
 *
 * \todo This iterface style receives OSRM results as json text documents
 *       and we have to parse them. I might be worth the effort to dig deeper
 *       into the OSRM code to avoid this step.
 */

#if 0
class osrm::OSRM;
#endif

class OsrmClient {

 private:

     osrm::engine::api::RouteParameters route_parameters;   ///< The OSRM request structure
     int status;          ///< Current state of the object
     std::string err_msg; ///< An error message if an error is reported.
     osrm::json::Object jsonResult;   ///< the json response document
     static bool connectionAvailable; ///< once set to false, it doesnt try to make a connection
     static osrm::OSRM  *routing_machine;
     static OsrmClient *p_osrm;
     OsrmClient();
     OsrmClient(const OsrmClient &other) = delete;
     OsrmClient &operator=(const OsrmClient &) = delete;
     bool use;

 public:

     /*! @brief Only allow one instance of class to be generated.  */
     static OsrmClient *Instance() {
         if (!p_osrm) {
             p_osrm = new OsrmClient;
         } else {
         }
         return p_osrm;
     }

     bool getOsrmViaroute();
     void useOsrm(bool desition) {use = desition;}
     bool getUse() const {return use;}


     /*!
      *  @brief Reset the OsrmClient to a clean state.
      */
     void clear();



     /*!
      * \brief Add a location in WGS84 to the OSRM request.
      * \param[in] node of type @ref Node
      */
     void addViaPoint(const Node &node);
     void addViaPoint(const std::deque<Node> &path);


     /*!
      * \brief Get the OSRM travel time for the requested route.
      * \param[out] time The OSRM travel time in decimal minutes.
      * \return True if an error was encountered and err_msg will be set. False if ok.
      */
     bool getOsrmTime(double &time);
     bool getOsrmTime(const Node &node1, const Node &node2, double &time);
     bool getOsrmTimes(std::deque<double> &times);

     /*!
      * \brief Extract the geometry from the OSRM response.
      * \param[out] geom A std::deque<Node> with each point in the path set as a \ref Node.
      * \return True if an error was encountered and err_msg will be set. False if ok.
      */
     bool getOsrmGeometry(std::deque<Node> &geom);

     /*!
      * \brief Extract the geometry from the OSRM response.
      * \param[out] hints A std::deque<getOsrmGeometry> with each point in the path set as a \ref Node.
      * \return True if an error was encountered and err_msg will be set. False if ok.
      */
     bool getOsrmHints(std::deque<std::string> &hints);

     bool getOsrmStreetNames(std::deque<std::string> &names);
#if 1
     bool getOsrmNamesOnRoute(std::deque<std::string> &names);
#endif
     int getStatus() const { return status; }
     int getConnection() const {return connectionAvailable; }
     std::string getErrorMsg() const { return err_msg; }
     osrm::json::Object getHttpContent() const { return jsonResult; }
     bool testOsrmClient(
             double x1, double y1,
             double x2, double y2,
             double x3, double y3);

    /*!  @brief Get coordinates of the nearest point (virtual node) in the nearest edge (OSRM) for a point and edege name
      
       Get node coordinates from return json and set
       GET "http://localhost:5000/nearest/v1/driving/-56.1743363,-34.9137291"

       ~~~~{.c}
       {
           "waypoints": [
               {
                   "hint": "VTIAgMe0AIDwJwAAFAAAADAAAAAAAAAAAAAAAGGPAACz7wAAEgAAAAjZpvwAQuv9ANmm_D9C6_0AAAEBwvX4oQ==",
                   "distance": 7.045144,
                   "name": "Doctor Luis Piera",
                   "location": [
                       -56.174328,
                       -34.913792
                   ]
               }
           ],
           "code": "Ok"
       }
       ~~~~
       

       @param[in] iNode Point @ref Node.
       @param[out] oNode TODO verify what it returns  type: @ref Node.
       @param[out] distance in meters
       @param[out] street name
       */

     bool getOsrmNearest(
             const Tweval &iNode,
             Node &oNode,
             double &distance,
             std::string street);

 private:

     void addViaPoint(double lat, double lon);

     bool getOsrmTime(
             double lat1, double lon1 ,
             double lat2, double lon2,
             const std::string &hint1, const std::string &hint2,
             double &time);

     bool getOsrmTime(
             double lat1, double lon1,
             double lat2, double lon2,
             double &time);


public:


#ifdef DOVRPLOG
     void dump() {
         DLOG(INFO) << "----- OsrmClient ----------"
             << "\nstatus: " << status
             << "\nerr_msg: " << err_msg
             << "\ncoordinates.size(): " << route_parameters.coordinates.size();
#if 0
         << "\nhttpContent: " << httpContent;
#endif
     }
#endif
};

#define osrmi OsrmClient::Instance()

#endif
