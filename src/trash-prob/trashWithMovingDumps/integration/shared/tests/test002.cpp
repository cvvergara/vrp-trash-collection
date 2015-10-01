#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include "osrmclient.h"
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE OsrmClient test
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_CASE( getOsrmLocate )
{
    double ilat, ilon;
    double olat, olon;
    // http://127.0.0.1:5000/locate?loc=-34.9137291,-56.1743363
    // {"mapped_coordinate":[-34.913815,-56.174625],"status":0}
    double rolat = -34.913815;
    double rolon = -56.174625;

    ilat = -34.913729100;
    ilon = -56.174336300;
    olat = -1.0;
    olon = -1.0;

    osrmi->useOsrm(true);
    osrmi->getOsrmLocate(ilat,ilon,olat,olon);
    BOOST_REQUIRE_CLOSE( olat, rolat, 0.00001 );
    BOOST_REQUIRE_CLOSE( olon, rolon, 0.00001 );

    // my_class test_object( "qwerty" );
    // BOOST_CHECK( test_object.is_valid() );
}

BOOST_AUTO_TEST_CASE( getOsrmNearest )
{
    double ilat, ilon;
    double olat, olon;
    std::string oname;
    // http://127.0.0.1:5000/nearest?loc=-34.9137291,-56.1743363
    // {"name":"Doctor Luis Piera","mapped_coordinate":[-34.913792,-56.174328],"status":0}
    double rolat = -34.913792;
    double rolon = -56.174328;
    std::string roname = "Doctor Luis Piera";

    ilat = -34.9137291;
    ilon = -56.1743363;
    olat = -1.0;
    olon = -1.0;

    osrmi->useOsrm(true);
    osrmi->getOsrmNearest(ilat, ilon, olat, olon, oname);
    BOOST_REQUIRE_CLOSE( olat, rolat, 0.00001 );
    BOOST_REQUIRE_CLOSE( olon, rolon, 0.00001 );
    BOOST_REQUIRE_EQUAL(oname, roname);

    // my_class test_object( "qwerty" );
    // BOOST_CHECK( test_object.is_valid() );
}
