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

    UID phantomNodeId,fwNodeId,rvNodeId, fwWeight, rvWeight, nameId;
    double ilat, ilon;
    ilat = -34.123456;
    ilon = -56.654321;
    phantomNodeId = 1;
    fwNodeId = 2;
    rvNodeId = 3;
    fwWeight = 4;
    rvWeight = 5;
    nameId = 6;
    std::cout << "Creating PhantomNode with parameters:"<< std::endl;
    std::cout << "phantomNodeId,lon,lat,fwNodeId,rvNodeId,fwWeight, rvWeight, nameId" << std::endl;
    std::cout << phantomNodeId << ilon << ilat << fwNodeId << rvNodeId <<  fwWeight <<  rvWeight <<  nameId << std::endl;
    PhantomNode pn(phantomNodeId,ilon,ilat,fwNodeId,rvNodeId, fwWeight, rvWeight, nameId);
    std::cout << "Created PhantomNode with parameters:"<< std::endl;
    std::cout << "phantomNodeId,lon,lat,fwNodeId,rvNodeId,fwWeight, rvWeight, nameId" << std::endl;
    std::cout << pn.mPhantomNodeId << pn.mPoint.x() << pn.mPoint.y() << pn.mForwNodeId << pn.mReveNodeId <<  pn.forwWeight() <<  pn.reveWeight() <<  pn.mNameId << std::endl;

    BOOST_REQUIRE_EQUAL(phantomNodeId,pn.mPhantomNodeId);
    BOOST_REQUIRE_EQUAL(ilon,pn.mPoint.x());
    BOOST_REQUIRE_EQUAL(ilat,pn.mPoint.y());
    BOOST_REQUIRE_EQUAL(fwNodeId,pn.mForwNodeId);
    BOOST_REQUIRE_EQUAL(rvNodeId,pn.mReveNodeId );
    BOOST_REQUIRE_EQUAL(fwWeight,pn.forwWeight());
    BOOST_REQUIRE_EQUAL(rvWeight,pn.reveWeight());
    BOOST_REQUIRE_EQUAL(nameId,pn.mNameId);
}

BOOST_AUTO_TEST_SUITE_END()
