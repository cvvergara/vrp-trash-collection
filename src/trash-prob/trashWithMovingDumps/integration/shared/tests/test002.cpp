#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include "node.h"
#include "osrmclient.h"
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE OsrmClient test
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_SUITE( OsrmClientTest )

BOOST_AUTO_TEST_CASE( getOsrmLocate )
{
    double ilat, ilon;
    double olat, olon;
    // Sobre Piera
    // http://127.0.0.1:5000/locate?loc=-34.9137291,-56.1743363
    // {"mapped_coordinate":[-34.913815,-56.174625],"status":0}
    double rolat = -34.913815;
    double rolon = -56.174625;

    ilat = -34.913729100;
    ilon = -56.174336300;
    olat = -1.0;
    olon = -1.0;

    osrmi->useOsrm(true);
    // BOOST_TEST_MESSAGE( "Testing getOsrmLocate (vrptools)" );
    osrmi->getOsrmLocate(ilat,ilon,olat,olon);
    BOOST_REQUIRE_CLOSE( olat, rolat, 0.00001 );
    BOOST_REQUIRE_CLOSE( olon, rolon, 0.00001 );

    // Sobre frugoni
    // http://127.0.0.1:5000/locate?loc=-34.9136522,-56.174577
    // {"mapped_coordinate":[-34.913815,-56.174625],"status":0}
    ilat = -34.9136522;
    ilon = -56.174336300;
    olat = -1.0;
    olon = -1.0;
    osrmi->getOsrmLocate(ilat,ilon,olat,olon);
    BOOST_REQUIRE_CLOSE( olat, rolat, 0.0001 );
    BOOST_REQUIRE_CLOSE( olon, rolon, 0.0001 );
}

BOOST_AUTO_TEST_CASE( getOsrmNearest )
{
    double ilat, ilon;
    double olat, olon;
    unsigned int forw_id, reve_id, forw_wt, reve_wt, street_id;
    unsigned int one_way;
    double rolat, rolon;


    // http://127.0.0.1:5000/nearest?loc=-34.9137291,-56.1743363
    // {"name":"Doctor Luis Piera","mapped_coordinate":[-34.913792,-56.174328],"status":0}
    ilat = -34.9137291;
    ilon = -56.1743363;
    one_way = 1000;
    olat = -1.0;
    olon = -1.0;
    rolat = -34.913792;
    rolon = -56.174328;
    //roname = "Doctor Luis Piera";

    osrmi->useOsrm(true);
    osrmi->getOsrmNearest(ilon, ilat, olon, olat, one_way, forw_id, reve_id, forw_wt, reve_wt, street_id);
    std::cout << olat << "|" << olon << "|" << one_way << "|" << forw_id << "|" << reve_id << "|" << street_id << std::endl;
    BOOST_REQUIRE_CLOSE( olat, rolat, 0.0001 );
    BOOST_REQUIRE_CLOSE( olon, rolon, 0.0001 );
    BOOST_REQUIRE_EQUAL( one_way, 0);

    // http://127.0.0.1:5000/nearest?loc=-34.9136522,-56.174577
    // {"name":"Doctor Emilio Frugoni","mapped_coordinate":[-34.913654,-56.174641],"status":0}
    ilat = -34.9136522;
    ilon = -56.174577;
    one_way = 1000;
    olat = -1.0;
    olon = -1.0;
    rolat = -34.913654;
    rolon = -56.174641;
    //roname = "Doctor Emilio Frugoni";
    osrmi->useOsrm(true);
    osrmi->getOsrmNearest(ilon, ilat, olon, olat, one_way, forw_id, reve_id, forw_wt, reve_wt, street_id);
    std::cout << olat << "|" << olon << "|" << one_way << "|" << forw_id << "|" << reve_id << "|" << street_id << std::endl;
    BOOST_REQUIRE_CLOSE( olat, rolat, 0.0001 );
    BOOST_REQUIRE_CLOSE( olon, rolon, 0.0001 );
    BOOST_REQUIRE_EQUAL( one_way, 1 );
}

BOOST_AUTO_TEST_CASE( isRightToSegment )
{
    Node lineBegin;
    Node lineEnd;
    Node point;
    bool res;

    lineBegin = Node(1,1);
    lineEnd = Node(10,10);
    point = Node (2,1);

    // Esta a la derecha
    res = point.isRightToSegment(lineBegin,lineEnd);
    BOOST_REQUIRE_EQUAL(res, true);
    // Esta a la izquierda
    res = point.isRightToSegment(lineEnd,lineBegin);
    BOOST_REQUIRE_EQUAL(res, false);

    // Esta a la derecha
    point.set_x(100);
    point.set_y(2);
    res = point.isRightToSegment(lineBegin,lineEnd);
    BOOST_REQUIRE_EQUAL(res, true);

    // Esta a la izquierda
    point.set_x(5);
    point.set_y(20);
    res = point.isRightToSegment(lineBegin,lineEnd);
    BOOST_REQUIRE_EQUAL(res, false);
}

BOOST_AUTO_TEST_SUITE_END()
