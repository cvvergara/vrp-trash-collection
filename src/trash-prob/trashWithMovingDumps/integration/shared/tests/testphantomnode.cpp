#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include "phantomnode.h"
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PhantomNode test
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

BOOST_AUTO_TEST_SUITE( PhantomNodeTest )

BOOST_AUTO_TEST_CASE( init )
{

    double ilat, ilon;
    ilat = -34.123456;
    ilon = -56.654321;

    Point p = Point(ilon,ilat);
    BOOST_REQUIRE_CLOSE( p.x(), ilon, 0.00001 );
    BOOST_REQUIRE_CLOSE( p.y(), ilat, 0.00001 );

    Point q(p);
    BOOST_REQUIRE_CLOSE( p.x(), q.x(), 0.00001 );
    BOOST_REQUIRE_CLOSE( p.y(), q.y(), 0.00001 );

    Point r = p;
    BOOST_REQUIRE_CLOSE( p.x(), r.x(), 0.00001 );
    BOOST_REQUIRE_CLOSE( p.y(), r.y(), 0.00001 );
}

BOOST_AUTO_TEST_SUITE_END()
