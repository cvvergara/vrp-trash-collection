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
#include <osrm/osrm.hpp>
#include <osrm/libosrm_config.hpp>
#include "renderer.hpp"

#include <climits>
#include <sstream>
#include <string>
#include <cstring>

#ifdef DOVRPLOG
#include "logger.h"
#endif


#include "osrmclient.h"

#ifdef DOSTATS
#include "timer.h"
#include "stats.h"
#endif

OsrmClient *OsrmClient::p_osrm = NULL;
OSRM *OsrmClient::routing_machine = NULL;
bool OsrmClient::connectionAvailable = true;

OsrmClient::OsrmClient(const OsrmClient &other) {
  connectionAvailable = other.connectionAvailable;
}

OsrmClient &OsrmClient::operator=(const OsrmClient &other) {
  connectionAvailable = other.connectionAvailable;
}

/*!  * \brief The OsrmClient constructor.  */
OsrmClient::OsrmClient() {
  if ( not connectionAvailable ) return;

#ifdef DOSTATS
  Timer timer;
#endif

  try {
    // No server paths needed (shared memory)
    ServerPaths server_paths;
    // Develop branch
    libosrm_config losrm_config;
    losrm_config.server_paths = server_paths;
    losrm_config.use_shared_memory = true;
    // Default
    losrm_config.max_locations_distance_table = 100;
    // Default
    losrm_config.max_locations_map_matching = -1;
    OsrmClient::routing_machine = new OSRM( losrm_config );
  } catch ( std::exception &e ) {
    status = -1;
    err_msg = std::string( "OsrmClient::OsrmClient caught exception: " ) + e.what();
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::OsrmClient Cumulative time", timer.duration() );
#endif
    connectionAvailable = false;
    return;
  };

  route_parameters.zoom_level = 18;
  route_parameters.print_instructions = true;
  route_parameters.alternate_route = false;
  route_parameters.geometry = false;
  route_parameters.compression = false;
  route_parameters.check_sum = UINT_MAX;
  route_parameters.service = "viaroute";
  route_parameters.output_format = "json";
  route_parameters.jsonp_parameter = "";
  route_parameters.language = "";

  status = 0;
  use = false;
  addPenalty = false;

#ifdef DOSTATS
  STATS->addto( "OsrmClient::OsrmClient Cumulative time", timer.duration() );
#endif

}

/*!
 * \brief Clear out any old points and reset the OsrmClient to a clean state.
 */
void OsrmClient::clear()
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

#ifdef DOSTATS
  Timer timer;
#endif
  route_parameters.coordinates.clear();
  route_parameters.hints.clear();
  route_parameters.geometry = false;
  route_parameters.compression = false;
  httpContent = "";
  err_msg = "";

  if ( status > 0 ) status = 0;

#ifdef DOSTATS
  STATS->addto( "OsrmClient::Clear Cumulative time", timer.duration() );
#endif
}


/*!
 * \brief Add a location in WGS84 to the OSRM request.
 * \param[in] lat The latitude of the location you want to add.
 * \param[in] lon The Longitude of the location you want to add.
 * \note route_parameters.coordinates is defined as std::vector<FixedPointCoordinate> in OSRM.h. COORDINATE_PRECISION is also defined in OSRM.h.
 */
void OsrmClient::addViaPoint( double lat, double lon )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

#ifdef DOSTATS
  Timer timer;
#endif
  FixedPointCoordinate p( lat * COORDINATE_PRECISION,
                          lon * COORDINATE_PRECISION );
  route_parameters.coordinates.push_back( p );
#ifdef DOSTATS
  STATS->addto( "OsrmClient::addViaPoint Cumulative time", timer.duration() );
#endif
}


/*!
 * \brief Add a \ref Node as a location to the OSRM request.
 * \param[in] node The Node to add, assumed to be in WGS84.
 */
void OsrmClient::addViaPoint( const Node &node )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

#ifdef DOSTATS
  Timer timer;
#endif
  addViaPoint( node.y(), node.x() );
  route_parameters.hints.push_back( node.hint() );
#ifdef DOSTATS
  STATS->addto( "OsrmClient::addViaPoint Cumulative time", timer.duration() );
#endif
}


/*!
 * \brief Add a path of \ref Node as locations to a the OSRM request.
 * \param[in] path A std::deque<Node> that you want to add.
 */
void OsrmClient::addViaPoints( const std::deque<Node> &path )
{
  if ( not connectionAvailable ) return;

  if ( not use ) return;

  std::deque<Node>::const_iterator it;

  for ( it = path.begin(); it != path.end(); ++it )
    addViaPoint( *it );
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 , double lat2,
                              double lon2, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (2 points) " );
#endif
  clear();
  addViaPoint( lat1, lon1 );
  addViaPoint( lat2, lon2 );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( double lat1, double lon1 , double lat2,
                              double lon2, const  std::string &hint1, const std::string &hint2,
                              double &time )
{
  if ( not connectionAvailable ) return false;
  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (2 points) " );
#endif
  clear();
  addViaPoint( lat1, lon1 );
  addViaPoint( lat2, lon2 );
  route_parameters.hints.push_back( hint1 );
  route_parameters.hints.push_back( hint2 );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (2 nodes) " );
#endif
  clear();
  addViaPoint( node1 );
  addViaPoint( node2 );
  route_parameters.hints.push_back( node1.hint() );
  route_parameters.hints.push_back( node2.hint() );

  if ( getOsrmViaroute() ) return  getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              const Node &node3, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (3 nodes) " );
#endif
  clear();
  addViaPoint( node1 );
  addViaPoint( node2 );
  addViaPoint( node3 );
  route_parameters.hints.push_back( node1.hint() );
  route_parameters.hints.push_back( node2.hint() );
  route_parameters.hints.push_back( node3.hint() );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

bool OsrmClient::getOsrmTime( const Node &node1, const Node &node2,
                              const Node &node3, const Node &node4, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  STATS->inc( "OsrmClient::getOsrmTime (4 nodes) " );
#endif
  clear();
  addViaPoint( node1 );
  addViaPoint( node2 );
  addViaPoint( node3 );
  addViaPoint( node4 );
  route_parameters.hints.push_back( node1.hint() );
  route_parameters.hints.push_back( node2.hint() );
  route_parameters.hints.push_back( node3.hint() );
  route_parameters.hints.push_back( node4.hint() );

  if ( getOsrmViaroute() ) return getOsrmTime( time );

  return false;
}

/*!
 * \brief Connect to the OSRM engine, issue the request and save the json response back in the object.
 * \return False if an error happened and err_msg will be set. True if ok.
 */
bool OsrmClient::getOsrmViaroute()
{
  if ( not connectionAvailable )
  {
        #ifdef OSRMCLIENT
          DLOG(INFO) << "Not connectionAvailable";
        #endif
      return false;
  }

  if ( not use )
  {
        #ifdef OSRMCLIENT
          DLOG(INFO) << "Not used";
        #endif
      return false;
  }

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmViaRoute (does the work) " );
#endif

  if ( route_parameters.coordinates.size() < 2 ) {
    err_msg = "OsrmClient:getOsrmViaroute must be called with two ro more viapoints!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                  timer.duration() );
#endif
    return false;
  }

  if ( status == -1 ) {
    err_msg = "OsrmClient:getOsrmViaroute Failed to connect to server!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                  timer.duration() );
#endif
    return false;
  }

  err_msg = "";
  //http::Reply osrm_reply;
  osrm::json::Object json_result;

  try {
    routing_machine->RunQuery( route_parameters, json_result );
  } catch ( std::exception &e ) {
    err_msg = std::string( "OsrmClient:getOsrmViaRoute caught exception: " )
              + e.what();
    connectionAvailable = false;
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                  timer.duration() );
#endif
    return false;
  }

  httpContent = "";

  std::stringstream tmp_ss;
  tmp_ss << json_result;
  httpContent = tmp_ss.str();

  status = 1;
#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmViaRoute (does the work) Cumulative time",
                timer.duration() );
#endif
  return true;
}

/*!
 * \brief Get the OSRM travel time for the requested route.
 * \param[out] time The OSRM travel time in decimal minutes.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmTime( double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmTime (does the work)" );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient:getOsrmTime does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTime (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmTime invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTime (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

  if ( not getTime( jsondoc, time ) ) {
#ifdef DOSTATS
    STATS->addto( "OsrmClient::getOsrmTime (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmTime (does the work) Cumulative time:",
                timer.duration() );
#endif
  return true;
}


bool OsrmClient::getOsrmTimes( std::deque<double> &times )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmTimes (does the work)" );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient:getOsrmTimes does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTimes (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmTimes invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTimes (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

  if ( not getTimes( jsondoc, times ) ) {
#ifdef DOSTATS
    STATS->addto( "OsrmClient::getOsrmTimes (errors) Cumulative time:",
                  timer.duration() );
#endif
    return false;
  }

#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmTimes (does the work) Cumulative time:",
                timer.duration() );
#endif
  return true;
}


bool OsrmClient::getOsrmPenalty( double &penalty )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmTurns" );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient:getOsrmTurns does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumultaive time:",
                  timer.duration() );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmTurns (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
    STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumultaive time:",
                  timer.duration() );
#endif
    return false;
  }

  if ( not getPenalty( jsondoc, penalty ) ) {
#ifdef DOSTATS
    STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumultaive time:",
                  timer.duration() );
#endif
    return false;
  }

#ifdef DOSTATS
  STATS->addto( "OsrmClient::getOsrmTurns (interface) Cumulative time:",
                timer.duration() );
#endif
  return true;
}


/*!
 * \brief Extract the geometry from the OSRM response if it was requested.
 * \param[out] geom A std::deque<Node> with each point in the path set as a \ref Node.
 * \return True if an error was encountered and err_msg will be set. False if ok.
 */
bool OsrmClient::getOsrmGeometry( std::deque<Node> &geom )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmGeometry (does the work) " );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmGeometry does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmGeometry invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  if ( not getGeom( jsondoc, geom ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::getOsrmGeometryText( std::string &geomText )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmGeometry (does the work) " );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmGeometry does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
}

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmGeometry invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  if ( not getGeomText( jsondoc, geomText ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::getOsrmHints( std::deque<std::string> &hints )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmHints (interface) " );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmHints (interface) does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmHints (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  if ( not getHints( jsondoc, hints ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::getOsrmStreetNames( std::deque<std::string> &names )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmStreetNames (interface) " );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmStreetNames (interface) does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmStreetNames (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  if ( not getNames( jsondoc, names ) ) {
    return false;
  }

  return true;
}


bool OsrmClient::getOsrmNamesOnRoute( std::deque<std::string> &names )
{
  if ( not connectionAvailable ) return false;

  if ( not use ) return false;

#ifdef DOSTATS
  Timer timer;
  STATS->inc( "OsrmClient::getOsrmNamesOnRoute (interface) " );
#endif

  if ( status != 1 or httpContent.size() == 0 ) {
    err_msg = "OsrmClient::getOsrmNamesOnRoute (interface) does not have a valid OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  rapidjson::Document jsondoc;
  jsondoc.Parse( httpContent.c_str() );

  if ( jsondoc.HasParseError() ) {
    err_msg = "OsrmClient:getOsrmNamesOnRoute (interface) invalid json document in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  if ( not getNamesOnRoute( jsondoc, names ) ) {
    return false;
  }

  return true;
}

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
#ifdef DOVRPLOG
    DLOG( INFO ) << "testing OsrmClient class";
#endif

  if (!use) return false;
#ifdef DOVRPLOG
    DLOG( INFO ) << "#1 OsrmClient set to Use";
#endif

  if (!connectionAvailable) return false;
#ifdef DOVRPLOG
    DLOG( INFO ) << "#2 OsrmClient Connection available";
#endif


  if ( getStatus() == -1 ) {
#ifdef DOVRPLOG
    DLOG( WARNING ) << getErrorMsg();
#endif
    return false;
  }

#ifdef DOVRPLOG
    DLOG( INFO ) << "#3 Status != -1";
#endif

  std::deque<std::string> hints;
  double penalty;
  bool oldPenalty = addPenalty;
  double time;
  std::string hint1;
  std::string hint2;

  // test 4
  if (getOsrmTime(x1, y1, x2, y2, time)) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "#4 test time:" << time;
#endif
  } else {
#ifdef DOVRPLOG
    DLOG( INFO ) << "#4 test time FAIL";
#endif
    return false;
  }


  // test 5
  if (getOsrmHints(hints)) {
    hint1 = hints[0];
    hint2 = hints[1];
#ifdef DOVRPLOG
    DLOG( INFO ) << "#5 get stored hints";
    DLOG( INFO ) << "#5 hint1" << hint1;
    DLOG( INFO ) << "#5 hint2" << hint2;
#endif
  } else {
#ifdef DOVRPLOG
    DLOG( INFO ) << "#5 get stored hints FAIL";
#endif
    return false;
  }

  // test 6
  hint1="";
  hint2="";
  if (getOsrmTime(x1, y1, x2, y2, hint1, hint2, time)) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "#6 get hint and time:" << time;
    DLOG( INFO ) << "#6 time:" << time;
    DLOG( INFO ) << "#6 hint1" << hint1;
    DLOG( INFO ) << "#6 hint2" << hint2;
#endif
  } else {
#ifdef DOVRPLOG
      DLOG( INFO ) << "#6 get hint and time FAIL";
#endif
      return false;
  }

//TODO make a test for testing penalties
#if 0
  // test 7
  if ( not getOsrmPenalty( penalty ) )
    return false;

  addPenalty = true;

  if ( getOsrmTime( -34.8917, -56.167694, -34.890816, -56.165529, time ) ) {
#ifdef DOVRPLOG
    DLOG( INFO ) << "test time: " << time;
#endif
  } else return false;

  getOsrmPenalty( penalty );

  if ( getOsrmHints( hints ) ) {
    std::string hint1 = hints[0];
    std::string hint2 = hints[1];

    if ( getOsrmTime( -34.8917, -56.167694, -34.890816, -56.165529, hint1, hint2,
                      time ) ) {
#ifdef DOVRPLOG
      DLOG( INFO ) << "test time: " << time;
#endif
    } else return false;
  } else return false;

  addPenalty = oldPenalty;
#endif

  // more tests
  clear();
  addViaPoint(x1, y1);
  addViaPoint(x2, y2);
  addViaPoint(x3, y3);
  addViaPoint(x1, y1);


  std::deque<double> times;
  // test 7 (with three points)
  if (!getOsrmViaroute()) {
#ifdef DOVRPLOG
    DLOG(INFO) << "#7 getOsrmViaroute Failed!" << std::endl;
#endif
    return false;
  }

  //test 8 (times array)
  if (getOsrmTimes(times)) {
#ifdef DOVRPLOG
      DLOG(INFO) << "#8 Times:" << std::endl;
      for (int i=0; i<times.size(); i++)
          DLOG(INFO) << "i: " << i << ", time: " << times[i] << std::endl;
#endif
  }
  else {
#ifdef DOVRPLOG
      DLOG(INFO) << "#8 getOsrmTimes Failed!";
#endif
      return false;
  }

  //test 8 (hints array)
  if (getOsrmHints(hints)) {
#ifdef DOVRPLOG
      DLOG(INFO) << "#9 Hints:" << std::endl;
      for (int i=0; i<hints.size(); i++)
          DLOG(INFO) << "i: " << i << ", hint: " << hints[i] << std::endl;
#endif
  }
  else {
#ifdef DOVRPLOG
      DLOG(INFO) << "#9 getOsrmHints Failed!" << std::endl;
#endif
      return false;
  }

  std::deque<std::string> names;
  if ( getOsrmStreetNames( names ) ) {
#ifdef DOVRPLOG
      DLOG(INFO) << "#10 StreetNames:" << std::endl;
      for (int i=0; i<names.size(); i++)
          DLOG(INFO) << "i: " << i << ", name: " << names[i] << std::endl;
#endif
  }
  else {
#ifdef DOVRPLOG
      DLOG(INFO) << "#10 getOsrmStreetNames Failed!" << std::endl;
#endif
      return false;
  }


  names.clear();
  if ( osrmi->getOsrmNamesOnRoute( names ) ) {
#ifdef DOVRPLOG
    DLOG(INFO) << "#11 NamesOnRoute:" << std::endl;
    for (int i=0; i<names.size(); i++)
        DLOG(INFO) << "i: " << i << ", name: " << names[i] << std::endl;
#endif
  } else {
#ifdef DOVRPLOG
    DLOG(INFO) << "#11 getOsrmNamesOnRoute Failed!" << std::endl;
#endif
    return false;
  }


  return true;
}

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

    if ( not connectionAvailable ) return false;

    if ( not use ) return false;

  #ifdef DOSTATS
    Timer timer;
    STATS->inc( "OsrmClient::getOsrmLocate (does the work) " );
  #endif

    // Backup current service name
    oldService = route_parameters.service;
    errorMsg = "";

    try {
      // Set service to use
      route_parameters.service = "locate";
      // Add point
      FixedPointCoordinate p(
          ilat * COORDINATE_PRECISION,
          ilon * COORDINATE_PRECISION
      );
      coordBackup = route_parameters.coordinates;
      route_parameters.coordinates.clear();
      route_parameters.coordinates.push_back(p);
      // std::cout << route_parameters.coordinates.size() << std::endl;
      // Ask OSRM
      routing_machine->RunQuery( route_parameters, jsonResult );
      // Restore points
      route_parameters.coordinates = coordBackup;
      // Restore service
      route_parameters.service = oldService;
    } catch ( std::exception &e ) {
      errorMsg = std::string( "OsrmClient:getOsrmLocate caught exception: " )
                + e.what();
      connectionAvailable = false;
  #ifdef DOSTATS
      STATS->inc( errorMsg );
      STATS->addto( "OsrmClient::getOsrmLocate (does the work) Cumulative time",
                    timer.duration() );
  #endif
      // Restore service
      route_parameters.service = oldService;
      return false;
    }

    //rContent = "";
    tmpSS << jsonResult;
    //rContent = tmpSS.str();
    jsonDoc.Parse( tmpSS.str().c_str() );

    // Everything ok. Set status.
    status = 1;
    // Get node coordinates from return json and set
    // http://127.0.0.1:5000/locate?loc=-34.8993,-56.1296
    // locate returns -> {"mapped_coordinate":[-34.899513,-56.129681],"status":0}
    if ( not jsonDoc.HasMember( "mapped_coordinate" ) ) {
      errorMsg = "OsrmClient:getOsrmLocate failed to find 'mapped_coordinate' key in OSRM response!";
  #ifdef DOSTATS
      STATS->inc( err_msg );
  #endif
      return false;
    }

    // extract the latitude and longitude
    olat = ( double ) jsonDoc["mapped_coordinate"][0].GetDouble(); // / COORDINATE_PRECISION;
    olon = ( double ) jsonDoc["mapped_coordinate"][1].GetDouble(); // / COORDINATE_PRECISION;
    return true;
}

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

    if ( not connectionAvailable ) return false;

    if ( not use ) return false;

  #ifdef DOSTATS
    Timer timer;
    STATS->inc( "OsrmClient::getOsrmLocate (does the work) " );
  #endif

    // Backup current service name
    oldService = route_parameters.service;
    errorMsg = "";

    try {
      // Set service to use
      route_parameters.service = "nearest";
      // Add point
      FixedPointCoordinate p(
          ilat * COORDINATE_PRECISION,
          ilon * COORDINATE_PRECISION
      );
      coordBackup = route_parameters.coordinates;
      route_parameters.coordinates.clear();
      route_parameters.coordinates.push_back(p);
      // std::cout << route_parameters.coordinates.size() << std::endl;
      // Ask OSRM
      routing_machine->RunQuery( route_parameters, jsonResult );
      // Restore points
      route_parameters.coordinates = coordBackup;
      // Restore service
      route_parameters.service = oldService;
    } catch ( std::exception &e ) {
      errorMsg = std::string( "OsrmClient:getOsrmNearest caught exception: " )
                + e.what();
      connectionAvailable = false;
  #ifdef DOSTATS
      STATS->inc( errorMsg );
      STATS->addto( "OsrmClient::getOsrmNearest (does the work) Cumulative time",
                    timer.duration() );
  #endif
      // Restore service
      route_parameters.service = oldService;
      return false;
    }

    //rContent = "";
    tmpSS << jsonResult;
    //rContent = tmpSS.str();
    jsonDoc.Parse( tmpSS.str().c_str() );

    // Everything ok. Set status.
    status = 1;
    // Get node coordinates from return json and set
    // http://127.0.0.1:5000/nearest?loc=-34.9137291,-56.1743363
    // nearest returns -> {"name":"Doctor Luis Piera","mapped_coordinate":[-34.913792,-56.174328],"status":0}
    if ( jsonDoc.HasMember( "status" ) ) {
        int rStatus = ( int ) jsonDoc["status"].GetInt();
        if (rStatus == 0) {
            // extract the latitude and longitude
            if ( jsonDoc.HasMember( "mapped_coordinate" ) ) {
                olat = ( double ) jsonDoc["mapped_coordinate"][0].GetDouble();
                olon = ( double ) jsonDoc["mapped_coordinate"][1].GetDouble();
            }
            if ( jsonDoc.HasMember( "one_way" ) ) {
                one_way = ( unsigned int ) jsonDoc["one_way"].GetInt();
            }
            if ( jsonDoc.HasMember( "forward_node_id" ) ) {
                forward_id = ( unsigned int ) jsonDoc["forward_node_id"].GetInt();
            }
            if ( jsonDoc.HasMember( "reverse_node_id" ) ) {
                reverse_id = ( unsigned int ) jsonDoc["reverse_node_id"].GetInt();
            }
            if ( jsonDoc.HasMember( "forward_weight" ) ) {
                forward_wt = ( unsigned int ) jsonDoc["forward_weight"].GetInt();
            }
            if ( jsonDoc.HasMember( "reverse_weight" ) ) {
                reverse_wt = ( unsigned int ) jsonDoc["reverse_weight"].GetInt();
            }
            if ( jsonDoc.HasMember( "name_id" ) ) {
                street_id = ( unsigned int ) jsonDoc["name_id"].GetInt();
            }
            return true;
        } else {
            errorMsg = "OsrmClient:getOsrmNearest failed. Bad status 'status' in OSRM response!";
        #ifdef DOSTATS
            STATS->inc( err_msg );
        #endif
          return false;
        }
    } else {
        errorMsg = "OsrmClient:getOsrmNearest failed to find 'status' key in OSRM response!";
    #ifdef DOSTATS
        STATS->inc( err_msg );
    #endif
        return false;
    }
}


// --------- private ----------------


/*!
 * \brief Parse the actual json document and extract the OSRM Travel time.
 * \param[in] jsondoc The json parse tree pointer.
 * \param[out] time The OSRM travel time as decimal minutes.
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getTime( rapidjson::Document &jsondoc, double &time )
{
  if ( not connectionAvailable ) return false;

  if ( not jsondoc.HasMember( "route_summary" ) ) {
    err_msg = "OsrmClient:getTime failed to find 'route_summary' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // find the 'total_time' in the 'route_summary'
  if ( not jsondoc["route_summary"].HasMember( "total_time" ) ) {
    err_msg = "OsrmClient:getTime failed to find 'total_time' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // extract the total_time and convert from seconds to minutes
  time = ( double ) jsondoc["route_summary"]["total_time"].GetDouble() / 60.0;

#ifdef OSRMCLIENTTRACE
  /*
  DLOG( INFO ) << "OsrmClient:getTime - json: " << httpContent << std::endl;
  DLOG( INFO ) << "OsrmClient:getTime - time: " << time << " min" << std::endl;
  */
#endif

  double penalty;

  if ( addPenalty and getPenalty( jsondoc, penalty ) ) time += penalty;

#ifdef OSRMCLIENTTRACE
  /*
  DLOG( INFO ) << "OsrmClient:getTime - time + penalty: " << time << " min" << std::endl;
  */
#endif

  return true;
}

/*!
 * \brief Parse the actual json document and extract via point times
 * \param[in] jsondoc The json parse tree pointer.
 * \param[out] time The OSRM travel time as decimal minutes.
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getTimes( rapidjson::Document &jsondoc, std::deque<double> &times )
{
  if ( not connectionAvailable ) return false;

  if ( not jsondoc.HasMember( "route_instructions" ) ) {
    err_msg = "OsrmClient:getTimes failed to find 'route_instructions' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  const rapidjson::Value& instructions = jsondoc["route_instructions"];

  // find the 'total_time' in the 'route_summary'
  if ( not instructions.IsArray() or instructions.Size() < 2 ) {
    err_msg = "OsrmClient:getTimes route_instructions is not an array of at least 2 in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // extract the total_time and convert from seconds to minutes
  times.clear();
  double time = 0.0;        // accumulate times
  times.push_back( time );  // push the start time is always 0.0
  for (rapidjson::SizeType i=0; i<instructions.Size(); i++) {
    const char * inst = instructions[i][0].GetString();
    if ( not strcmp(inst, "9") or not strcmp(inst, "15") )
      times.push_back( time );
    time += ( double ) instructions[i][4].GetDouble() / 60.0;
  }

#ifdef OSRMCLIENTTRACE
  /*
  DLOG( INFO ) << "OsrmClient:getTime - json: " << httpContent << std::endl;
  std::stringstream tmp2_ss;
  for (int i=0; i<times.size(); i++)
    tmp2_ss << times[i] << ", ";
  DLOG( INFO ) << "OsrmClient:getTime - times: " << tmp2_ss.str() << " min" << std::endl;
  */
#endif

  return true;
}


/*!
 * \brief Parse the actual json document and extract the geometry Nodes
 * \param[in] jsondoc The json parse tree pointer.
 * \param[out] geom A std::deque<Node> with each point in the path.
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getGeom( rapidjson::Document &jsondoc,
                          std::deque<Node> &geom )
{
  if ( not connectionAvailable ) return false;

  // clear the path
  geom.clear();

  // find the route 'geometry' key in the response
  const rapidjson::Value &jgeom = jsondoc["route_geometry"];

  if ( not jgeom.IsArray() ) {
    err_msg = "OsrmClient:getGeom failed to find 'geometry' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  for ( rapidjson::Value::ConstValueIterator itr = jgeom.Begin();
        itr != jgeom.End(); ++itr ) {

    // osrm returns [lat, lon], node expects [lon, lat]
    Node n( (*itr)[1].GetDouble(), (*itr)[0].GetDouble() );
    geom.push_back( n );
  }

  return true;
}


/*!
 * \brief Parse the actual json document and extract the geometry text
 * \param[in] jsondoc The json parse tree pointer.
 * \param[out] geomText A std::string which it the compressed geometry
 * \return True if an error and err_msg will be set. False otherwise.
 */
bool OsrmClient::getGeomText( rapidjson::Document &jsondoc,
                          std::string &geomText )
{
  if ( not connectionAvailable ) return false;

  if ( not jsondoc.HasMember("route_geometry") and
       not jsondoc["route_geometry"].IsString() ) {
    err_msg = "OsrmClient:getGeomText failed to find 'geometry' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // find the route 'geometry' key in the response
  geomText = jsondoc["route_geometry"].GetString();

  return true;
}


bool OsrmClient::getHints( rapidjson::Document &jsondoc,
                           std::deque<std::string> &hints )
{
  if ( not connectionAvailable ) return false;

  hints.clear();

  // find the route 'hint_data' key in the response
  const rapidjson::Value &jHintData = jsondoc["hint_data"];

  if ( not jHintData.IsObject() ) {
    err_msg = "OsrmClient:getHints (private)  failed to find 'hint_data' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  const rapidjson::Value &jLocations = jHintData["locations"];

  if ( not jLocations.IsArray() ) {
    err_msg = "OsrmClient:getHints (private)  failed to find 'locations' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }


  std::string hint;

  for ( rapidjson::Value::ConstValueIterator itr = jLocations.Begin();
        itr != jLocations.End(); ++itr) {

    hint = std::string( itr->GetString() );
    hints.push_back( hint );

  }

  return true;
}


bool OsrmClient::getNames( rapidjson::Document &jsondoc,
                           std::deque<std::string> &names )
{
  if ( not connectionAvailable ) return false;

  if ( not jsondoc.HasMember( "route_instructions" ) ) {
    err_msg = "OsrmClient:getNames failed to find 'route_instructions' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  const rapidjson::Value& instructions = jsondoc["route_instructions"];

  // find the 'total_time' in the 'route_summary'
  if ( not instructions.IsArray() or instructions.Size() < 2 ) {
    err_msg = "OsrmClient:getNames route_instructions is not an array of at least 2 in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // extract the total_time and convert from seconds to minutes
  names.clear();
  names.push_back( instructions[0][1].GetString() );
  for (rapidjson::SizeType i=0; i<instructions.Size(); i++) {
    const char * inst = instructions[i][0].GetString();
    if ( not strcmp(inst, "9") )
      names.push_back( instructions[i][1].GetString() );
    else if (not strcmp(inst, "15") and i-1 > 0)
      names.push_back( instructions[i-1][1].GetString() );
  }

  return true;
}


bool OsrmClient::getNamesOnRoute( rapidjson::Document &jsondoc,
                           std::deque<std::string> &names )
{
  if ( not connectionAvailable ) return false;

  if ( not jsondoc.HasMember( "route_instructions" ) ) {
    err_msg = "OsrmClient:getNames failed to find 'route_instructions' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  const rapidjson::Value& instructions = jsondoc["route_instructions"];

  // find the 'total_time' in the 'route_summary'
  if ( not instructions.IsArray() or instructions.Size() < 2 ) {
    err_msg = "OsrmClient:getNames route_instructions is not an array of at least 2 in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  // extract the total_time and convert from seconds to minutes
  names.clear();
  for (rapidjson::SizeType i=0; i<instructions.Size(); i++) {
    const char * name = instructions[i][1].GetString();
    if ( strlen(name) )
      names.push_back( name );
  }

  return true;
}


bool OsrmClient::getPenalty( rapidjson::Document &jsondoc,
                             double &penalty )   //in munutes
{
  if ( not connectionAvailable ) return false;

  int turn;

  std::string  trace;

  penalty = 0;

  // find the route 'hint_data' key in the response
  const rapidjson::Value &jInstructionsArray = jsondoc["route_instructions"];

  if ( not jInstructionsArray.IsArray() ) {
    err_msg = "OsrmClient:getTurns (private) failed to find 'route_instructions' key in OSRM response!";
#ifdef DOSTATS
    STATS->inc( err_msg );
#endif
    return false;
  }

  for (rapidjson::Value::ConstValueIterator itr = jInstructionsArray.Begin();
       itr != jInstructionsArray.End(); ++itr) {

    turn = strtol( (*itr)[0].GetString(), NULL, 10 );

    trace = std::string( (*itr)[0].GetString() );
#ifdef DOVRPLOG
    DLOG( INFO ) << "InstructionsData " << trace;
    DLOG( INFO ) << "Instruction " << turn;
#endif

    switch ( turn ) {
    case 2: penalty += 0.05; break; //slight right

    case 3: penalty += 0.10; break; //right

    case 4: penalty += 0.3 ; break; //sharp right

    case 5: penalty += 1;  break; //uturn

    case 8: penalty += 0.05; break; //slight left

    case 7: penalty += 0.10; break; //left

    case 6: penalty += 0.3 ; break; //sharp left
    }

  }

#ifdef DOVRPLOG
  DLOG( INFO ) << "Penalty " << penalty;
#endif
  return true;
}
