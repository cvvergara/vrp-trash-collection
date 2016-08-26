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

#include "baseClasses/osrmclient.h"

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "engine/hint.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <iomanip>
#include <exception>
#include <iostream>
#include <string>
#include <utility>


#include "baseClasses/node.h"
#include "baseClasses/logger.h"
#include "baseClasses/timer.h"
#include "baseClasses/stats.h"

OsrmClient *OsrmClient::p_osrm = NULL;
osrm::OSRM *OsrmClient::routing_machine = NULL;
bool OsrmClient::connectionAvailable = true;

#if 0
OsrmClient::OsrmClient(const OsrmClient &other) {
  connectionAvailable = other.connectionAvailable;
}

OsrmClient &OsrmClient::operator=(const OsrmClient &other) {
  connectionAvailable = other.connectionAvailable;
}
#endif

/*!  @brief The OsrmClient constructor.  */
OsrmClient::OsrmClient() {
    DLOG(INFO) << "Entering Constructor";

    try {

        osrm::engine::EngineConfig config;
        config.storage_config = {"/home/vicky/osrm-data/uruguay-latest"};
        config.use_shared_memory = true;

        routing_machine = new osrm::OSRM{config};

    } catch (std::exception &e) {
        DLOG(FATAL) << "OsrmClient::Exception caught\n" << e.what();
        status = -1;
        err_msg = std::string("OsrmClient::OsrmClient caught exception:\n") + e.what();
        DLOG(FATAL) << "OsrmClient Constructor caught exception:\n" << e.what();
        connectionAvailable = false;
        return;
    };

    status = 0; 
    use = false;
    connectionAvailable = true;
    DLOG(INFO) << "connectionAvailable == true";
}


void OsrmClient::clear() {
    route_parameters.coordinates.clear();
    route_parameters.hints.clear();
    jsonResult.values.clear();
    status = 0;
}



/*******************************************************************
 *
 * OsrmClient::getOsrmTime
 * OsrmClient::getOsrmTimes (times of each leg of the path)
 *
 ******************************************************************/
void OsrmClient::addViaPoint(const Node &node) {
    STATS_INC("OsrmClient::addViaPoint(const Node &node)");

    route_parameters.coordinates.push_back({
            osrm::util::FloatLongitude{node.x()},
            osrm::util::FloatLatitude{node.y()} });
    route_parameters.hints.push_back(osrm::engine::Hint::FromBase64(node.hint()));
    assert(route_parameters.coordinates.size() == route_parameters.hints.size());
}


/*!
 * \brief Add a path of \ref Node as locations to a the OSRM request.
 * \param[in] path A std::deque<Node> that you want to add.
 */
void OsrmClient::addViaPoint(const std::deque<Node> &path) {
    STATS_INC("OsrmClient::addViaPoint(std::deque<Node> &)");

    for (auto const &p : path) addViaPoint(p);
#if 0
=======
void OsrmClient::addViaPoints(const std::deque<Node> &path )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

  std::deque<Node>::const_iterator it;

  for ( it = path.begin(); it != path.end(); ++it )
    addViaPoint( *it );
>>>>>>> origin/right-side-montevideo
#endif
}

/*******************************************************************
 *
 * OsrmClient::getOsrmTime
 * OsrmClient::getOsrmTimes (times of each leg of the path)
 *
 ******************************************************************/

bool OsrmClient::getOsrmTime(
        const Node &node1,
        const Node &node2,
        double &time) {
    STATS_INC("OsrmClient::getOsrmTime (2 nodes)");

    time = 0;
    if (!connectionAvailable || !use) {
        return false;
    }
    clear();
    addViaPoint(node1);
    addViaPoint(node2);

    if (getOsrmViaroute()) return  getOsrmTime(time);
    return false;
}



bool OsrmClient::getOsrmTime(double &time) {
    STATS_INC("OsrmClient::getOsrmTime(double &time)");

    time = 0;
    if (!connectionAvailable || !use) {
        return false;
    }

    /*
     * extracting the duration
     */
    auto &routes = jsonResult.values["routes"].get<osrm::json::Array>();
    auto &route = routes.values.at(0).get<osrm::json::Object>();


    /*
     * setting the value
     */
    time = route.values["duration"].get<osrm::json::Number>().value;

    return true;
}


bool OsrmClient::getOsrmTimes(std::deque<double> &times) {
    STATS_INC("OsrmClient::getOsrmTimes(td::deque<double> &times");

    times.clear();
    if (!connectionAvailable || !use) {
        return false;
    }

    /*
     * extracting the durations of each leg
     */
    auto &routes = jsonResult.values["routes"].get<osrm::json::Array>();
    auto &route = routes.values.at(0).get<osrm::json::Object>();
    auto &leg = route.values["legs"].get<osrm::json::Array>();

    /*
     * Saving in container
     */
    for (const auto & leg_element: leg.values) {
        auto leg_object = leg_element.get<osrm::json::Object>();
        times.push_back(leg_object.values["duration"].get<osrm::json::Number>().value);
    }

    assert(leg.values.size() == times.size());
    return true;
}

#if 0
<<<<<<< HEAD
=======
bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              const Node &node3, const Node &node4, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;
>>>>>>> origin/right-side-montevideo
#endif

/*******************************************************************
 *
 * OsrmClient::getOsrmViaroute
 *
 ******************************************************************/

/*!
 * \brief Connect to the OSRM engine, issue the request and save the json response back in the object.
 * \return False if an error happened and err_msg will be set. True if ok.
 */
bool OsrmClient::getOsrmViaroute() {
    STATS_INC("OsrmClient::getOsrmViaroute()");

    assert(connectionAvailable);
    assert(use);
    assert(status != -1);
    DLOG_IF(WARNING, route_parameters.coordinates.size() < 2)
        << "route_parameters.coordinates.size()"
        << route_parameters.coordinates.size();

    if (route_parameters.coordinates.size() < 2) {
        status = -1;
        return false;
    }
    route_parameters.geometries = osrm::engine::api::RouteParameters::GeometriesType::GeoJSON;

    try {

        auto status = routing_machine->Route(route_parameters, jsonResult);
        assert(status == osrm::engine::Status::Ok);

    } catch (std::exception &e) {
        err_msg = std::string("OsrmClient:getOsrmViaRoute caught exception: ")
            + e.what();

        DLOG(FATAL) << "OsrmClient::getOsrmViaroute() caught exception:\n" << e.what();
        clear();
        return false;
    }

    status = jsonResult.values["code"].get<osrm::json::String>().value == "Ok";
    return true;
}


bool
OsrmClient::getOsrmGeometry(std::deque<Node> &geom) {
    STATS_INC("OsrmClient::getOsrmGeometry (std::deque<Node> &geom)");

    geom.clear();
    if (!connectionAvailable || !use) {
        return false;
    }

    /*
     * extracting the geomtery
     */
    auto &routes = jsonResult.values["routes"].get<osrm::json::Array>();
    auto &route = routes.values.at(0).get<osrm::json::Object>();
    auto &geometry = route.values["geometry"].get<osrm::json::Object>();
    auto &coordinates = geometry.values["coordinates"].get<osrm::json::Array>();

    /*
     * Saving in container
     */
    for (const auto &c: coordinates.values) {
        auto &coordinate = c.get<osrm::json::Array>();
        auto lat = coordinate.values.at(1).get<osrm::json::Number>().value;
        auto lon = coordinate.values.at(0).get<osrm::json::Number>().value;
        Node n(lon, lat);
        geom.push_back(n);
    }
    return true;
}



bool
OsrmClient::getOsrmHints(std::deque<std::string> &hints) {
    STATS_INC("OsrmClient::getOsrmHints(std::deque<std::string> &hints)");

    hints.clear();
    if (!connectionAvailable || !use) {
        return false;
    }

    /*
     * extracting the hints
     */
    auto &waypoints = jsonResult.values["waypoints"].get<osrm::json::Array>();

    /*
     * Saving in container
     */
    for (const auto way : waypoints.values) {
        auto way_object = way.get<osrm::json::Object>();
        hints.push_back(way_object.values["hint"].get<osrm::json::String>().value);
    }

    assert(waypoints.values.size() == hints.size());
    return true;
}



bool OsrmClient::getOsrmStreetNames(std::deque<std::string> &names) {
    STATS_INC("OsrmClient::getOsrmStreetNames(std::deque<std::string> &names)");

    names.clear();
    if (!connectionAvailable || !use) {
        return false;
    }


    /*
     * extracting the names
     */
    auto &waypoints = jsonResult.values["waypoints"].get<osrm::json::Array>();

    /*
     * Saving in container
     */
    for (const auto way : waypoints.values) {
        auto way_object = way.get<osrm::json::Object>();
        names.push_back(way_object.values["name"].get<osrm::json::String>().value);
    }

    assert(waypoints.values.size() == names.size());
    return true;
}


#if 0
bool OsrmClient::getOsrmNamesOnRoute(std::deque<std::string> &names) {

    if (not connectionAvailable) return false;
    if (not use) return false;

#ifdef DOSTATS
    Timer timer;
    STATS->inc("OsrmClient::getOsrmNamesOnRoute (interface) ");
#endif

    if (status != 1 or jsonResult.values.size() == 0) {
        err_msg = "OsrmClient::getOsrmNamesOnRoute (interface) does not have a valid OSRM response!";
#ifdef DOSTATS
        STATS->inc(err_msg);
#endif
        return false;
    }

    // TODO figure out how to get the names
#if 0
    rapidjson::Document jsondoc;
    jsondoc.Parse(jsonResult.c_str());

    if (jsondoc.HasParseError()) {
        err_msg = "OsrmClient:getOsrmNamesOnRoute (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
        STATS->inc(err_msg);
#endif
        return false;
    }

    if (not getNamesOnRoute(jsondoc, names)) {
        return false;
    }
#endif
    return true;
}
#endif

//! testOsrmClient
/*!
  returns false when something failes
#1 use: must be true
#2 connection
*/
bool OsrmClient::testOsrmClient(
        double x1, double y1,
        double x2, double y2,
        double x3, double y3) {

    /* testing with vlaues
       -34.87198931958, -56.190261840820305,
       -34.88156563691107, -56.17189407348633,
       -34.90634621832085 -56.16365432739258,
       */

    DLOG(INFO) << "testing OsrmClient class:";

    auto oldUse = use;
    use = true;
    DLOG(INFO) << "#1 OsrmClient set to Use";

    if (!connectionAvailable) return false;
    DLOG(INFO) << "#2 OsrmClient Connection available";

    if (getStatus() == -1) return false;
    DLOG(INFO) << "#3 Status != -1";


    double time;
    std::string hint1;
    std::string hint2;

    // test 4
    {
        DLOG(INFO) << std::setprecision(10)
            << "GET \"http://localhost:5000/route/v1/driving/"
            << y1 << "," << x1 << ";"
            << y2 << "," << x2
            << "?overview=false&alternatives=true&steps=true\"\n";

        if (getOsrmTime(x1, y1, x2, y2, time) == false) {
            DLOG(INFO) << "#4 test time FAIL";
            return false;
        }
        DLOG(INFO) << "#4 test time:" << time;
    }

    // test 5
    std::deque<std::string> hints;
    {
        if (getOsrmHints(hints) == false) {
            DLOG(INFO) << "#5 get stored hints FAIL";
            return false;
        }
        DLOG(INFO) << "#5 get stored hints";
        DLOG(INFO) << "#5 hint1" << hints[0];
        DLOG(INFO) << "#5 hint2" << hints[1] << "\n";
    }

    // test 6
    {
        if (getOsrmTime(x1, y1, x2, y2, hints[0], hints[1], time) == false) {
            DLOG(INFO) << "#6 getOsrmTime  FAIL";
            return false;
        }
        DLOG(INFO) << "#6 get time:" << time;
        DLOG(INFO) << "#6 time:" << time;
        DLOG(INFO) << "#6 hint1" << hint1;
        DLOG(INFO) << "#6 hint2" << hint2;
    }

    clear();

    DLOG(INFO) << std::setprecision(10)
        << "GET \"http://localhost:5000/route/v1/driving/"
        << y1 << "," << x1 << ";"
        << y2 << "," << x2 << ";"
        << y3 << "," << x3 << ";"
        << y1 << "," << x1
        << "?overview=false&alternatives=true&steps=true\"\n";
    // more tests

    // test 7
    addViaPoint(x1, y1);
    addViaPoint(x2, y2);
    addViaPoint(x3, y3);
    addViaPoint(x1, y1);

#if 0
<<<<<<< HEAD
=======
bool OsrmClient::getOsrmLocate(double ilon, double ilat, double &olon, double &olat)
{
    std::string oldService; //! backup service
    std::stringstream tmpSS; //!
    // std::string rContent; //!
    std::string errorMsg; //!
    rapidjson::Document jsonDoc;
    osrm::json::Object jsonResult; //! json result
    // TODO: Is backup realy necesary
    std::vector<FixedPointCoordinate> coordBackup;
>>>>>>> origin/right-side-montevideo
#endif

    // test 7 (with four points)
    if (!getOsrmViaroute()) {
        DLOG(INFO) << "#7 getOsrmViaroute Failed!" << std::endl;
        return false;
    }
    DLOG(INFO) << "#7 getOsrmViaroute OK" << std::endl;


    //test 8 (times array)
    {
        std::deque<double> times;
        if (getOsrmTimes(times) == false) {
            DLOG(INFO) << "#8 getOsrmTimes Failed!\n";
            return false;
        }
        DLOG(INFO) << "#8 Times:" << std::endl;
        double time_agg(0);

        for (const auto& time : times) {
            DLOG(INFO) << "time: " << time << std::endl;
            time_agg += time;
        }
        getOsrmTime(time);
        DLOG(INFO) << "Total time: " << time << std::endl;
        assert(time == time_agg);
    }

    //test 9 (hints array)
    {
        if (getOsrmHints(hints) == false) {
            DLOG(INFO) << "#9 getOsrmHints Failed!" << std::endl;
            return false;
        }
        DLOG(INFO) << "#9 Hints:" << std::endl;
        for (const auto hint : hints) {
            DLOG(INFO) << "hint: " << hint << std::endl;
        }
    }

    //test 9 (names array)
    {
        std::deque<std::string> names;
        if (getOsrmStreetNames(names) == false) {
            DLOG(INFO) << "#10 getOsrmStreetNames Failed!" << std::endl;
            return false;
        }
        DLOG(INFO) << "#10 StreetNames:" << std::endl;
        for (const auto &name : names) {
            DLOG(INFO) << "name: " << name << std::endl;
        }
    }


    //test 10 (geometries array)
    {
        std::deque<Node> geom;
        if (getOsrmGeometry(geom) == false) {
            DLOG(INFO) << "#10 getOsrmGeometry Failed!" << std::endl;
            return false;
        }
        DLOG(INFO) << "#10 getOsrmGeometry:" << std::endl;
        for (const auto &g : geom) {
            DLOG(INFO) << "geometry: " << g.x() << ", " << g.y() << std::endl;
        }
    }
#if 0
=======
bool OsrmClient::getOsrmNearest(double ilon, double ilat, double &olon, double &olat,
    unsigned int &one_way, unsigned int &forward_id, unsigned int &reverse_id, unsigned int &forward_wt, unsigned int &reverse_wt, unsigned int &street_id)
{
    std::string oldService; //! backup service
    std::stringstream tmpSS; //!
    // std::string rContent; //!
    std::string errorMsg; //!
    rapidjson::Document jsonDoc;
    osrm::json::Object jsonResult; //! json result
    // TODO: Is backup realy necesary
    std::vector<FixedPointCoordinate> coordBackup;
>>>>>>> origin/right-side-montevideo
#endif

    do {
        Node v(1.0, 1.0);
        Node w(3.0, 2.0);
        Node f0(0, 0);
        Node p1(0.9, 0.9);
        Node p2(2.0, 1.5);
        Node p3(1.1, 1.1);
        Node p4(3.1, 2.1);
        Node f5(4.0, 4.0);
        Node f6(2.5, 0.0);

        double tol = 0.2;

        DLOG(INFO) << "f0(0,0): " << f0.positionAlongSegment(v, w, tol);
        DLOG(INFO) << "p1(0.9,0.9): " << p1.positionAlongSegment(v, w, tol);
        DLOG(INFO) << "p2(2.0,1.5): " << p2.positionAlongSegment(v, w, tol);
        DLOG(INFO) << "p3(1.1,1.1): " << p3.positionAlongSegment(v, w, tol);
        DLOG(INFO) << "p4(3.1,2.1): " << p4.positionAlongSegment(v, w, tol);
        DLOG(INFO) << "f5(4.0,4.0): " << f5.positionAlongSegment(v, w, tol);
        DLOG(INFO) << "f6(2.5,0.0): " << f6.positionAlongSegment(v, w, tol);
    } while (false);



#if 0
    //test 11 (names on route)
    {
        std::deque<std::string> names;
        names.clear();
        if (osrmi->getOsrmNamesOnRoute(names)) {
            DLOG(INFO) << "#11 NamesOnRoute:" << std::endl;
            for (int i=0; i<names.size(); i++)
                DLOG(INFO) << "i: " << i << ", name: " << names[i] << std::endl;
        } else {
            DLOG(INFO) << "#11 getOsrmNamesOnRoute Failed!" << std::endl;
            return false;
        }
    }
#endif

    use = oldUse;
    return true;

}

bool OsrmClient::getOsrmNearest(
        const Node &iNode,
        Node &oNode,
        double &distance,
        std::string street) {
    STATS_INC("OsrmClient::getOsrmNearest");

    oNode = Node(iNode.x(), iNode.y());
    street ="";
    if (!connectionAvailable || !use) {
        return false;
    }

    osrm::json::Object nearJsonResult;

    try {

        osrm::engine::api::NearestParameters nearest_parameters;
        nearest_parameters.coordinates.push_back({
                osrm::util::FloatLongitude{iNode.x()},
                osrm::util::FloatLatitude{iNode.y()} });
        routing_machine->Nearest(nearest_parameters, jsonResult);

        if (jsonResult.values["code"].get<osrm::json::String>().value != "Ok") {
            return false;
        }

    } catch (std::exception &e) {
        connectionAvailable = false;
        err_msg = static_cast<std::string>("OsrmClient:getOsrmNearest caught exception: ") + e.what();
        STATS_INC(err_msg);
        return false;
    }


    /*
     * extracting the information
     */
    auto &waypoints = nearJsonResult.values["waypoints"].get<osrm::json::Array>();
    auto &way = waypoints.values.at(0).get<osrm::json::Object>();
    auto &location = way.values["location"].get<osrm::json::Array>();

    /*
     * Saving results
     */
    distance = way.values["distance"].get<osrm::json::Number>().value;
    street = way.values["name"].get<osrm::json::String>().value;
    auto lat = location.values.at(0).get<osrm::json::Number>().value;
    auto lon = location.values.at(1).get<osrm::json::Number>().value;
    oNode = Node(lon, lat);

    return true;
}














/*******************************************************************
 *******************************************************************
 *
 * PRIVATE SECTION
 *
 *******************************************************************
 ******************************************************************/

/*******************************************************************
 *
 * OsrmClient::getOsrmTime
 *
 * For testing purposes
 *
 ******************************************************************/

void OsrmClient::addViaPoint(double lat, double lon) {
    STATS_INC("OsrmClient::addViaPoint(double lat, double lon)");

    route_parameters.coordinates.push_back({osrm::util::FloatLongitude{lon}, osrm::util::FloatLatitude{lat}});
}

bool OsrmClient::getOsrmTime(
        double lat1, double lon1,
        double lat2, double lon2,
        double &time) {
    STATS_INC("OsrmClient::getOsrmTime (2 points)");

    clear();
    addViaPoint(lat1, lon1);
    addViaPoint(lat2, lon2);

    if (getOsrmViaroute()) return getOsrmTime(time);

    return false;
}

bool OsrmClient::getOsrmTime(
        double lat1, double lon1,
        double lat2, double lon2,
        const  std::string &hint1, const std::string &hint2,
        double &time) {
    STATS_INC("OsrmClient::getOsrmTime (2 points and giving hints)");

    if (not connectionAvailable) return false;
    if (not use) return false;

    clear();
    addViaPoint(lat1, lon1);
    addViaPoint(lat2, lon2);
    route_parameters.hints.push_back(osrm::engine::Hint::FromBase64(hint1));
    route_parameters.hints.push_back(osrm::engine::Hint::FromBase64(hint2));

    if (getOsrmViaroute()) return getOsrmTime(time);

    return false;
}

