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
    std::cout << phantomNodeId << "," << ilon << "," << ilat << "," << fwNodeId << "," << rvNodeId << "," <<  fwWeight << "," <<  rvWeight << "," <<  nameId << "," << std::endl;
    PhantomNode pn(phantomNodeId,ilon,ilat,fwNodeId,rvNodeId, fwWeight, rvWeight, nameId);
    std::cout << "Created PhantomNode with parameters:"<< std::endl;
    std::cout << "phantomNodeId,lon,lat,fwNodeId,rvNodeId,fwWeight, rvWeight, nameId" << std::endl;
    std::cout << pn.id() << pn.point().x() << pn.point().y() << pn.forwNodeId() << pn.reveNodeId() <<  pn.forwWeight() <<  pn.reveWeight() <<  pn.nameId()<< std::endl;

    BOOST_REQUIRE_EQUAL(phantomNodeId,pn.id());
    BOOST_REQUIRE_EQUAL(ilon,pn.point().x());
    BOOST_REQUIRE_EQUAL(ilat,pn.point().y());
    BOOST_REQUIRE_EQUAL(fwNodeId,pn.forwNodeId());
    BOOST_REQUIRE_EQUAL(rvNodeId,pn.reveNodeId());
    BOOST_REQUIRE_EQUAL(fwWeight,pn.forwWeight());
    BOOST_REQUIRE_EQUAL(rvWeight,pn.reveWeight());
    BOOST_REQUIRE_EQUAL(nameId,pn.nameId());

    std::cout << "Creating other PhantomNode with created PhantomNode as parameter:"<< std::endl;
    PhantomNode pn2(pn);
    std::cout << "Created PhantomNode with parameters:"<< std::endl;
    std::cout << "phantomNodeId,lon,lat,fwNodeId,rvNodeId,fwWeight, rvWeight, nameId" << std::endl;
    std::cout << pn2.id() << pn2.point().x() << pn2.point().y() << pn2.forwNodeId() << pn2.reveNodeId() <<  pn2.forwWeight() <<  pn2.reveWeight() <<  pn2.nameId()<< std::endl;

    BOOST_REQUIRE_EQUAL(pn2.id(),pn.id());
    BOOST_REQUIRE_EQUAL(pn2.point().x(),pn.point().x());
    BOOST_REQUIRE_EQUAL(pn2.point().y(),pn.point().y());
    BOOST_REQUIRE_EQUAL(pn2.forwNodeId(),pn.forwNodeId());
    BOOST_REQUIRE_EQUAL(pn2.reveNodeId(),pn.reveNodeId());
    BOOST_REQUIRE_EQUAL(pn2.forwWeight(),pn.forwWeight());
    BOOST_REQUIRE_EQUAL(pn2.reveWeight(),pn.reveWeight());
    BOOST_REQUIRE_EQUAL(pn2.nameId(),pn.nameId());

    std::cout << "Testing equal constructor:"<< std::endl;
    PhantomNode pn3(pn);
    std::cout << "Created PhantomNode with parameters:"<< std::endl;
    std::cout << "phantomNodeId,lon,lat,fwNodeId,rvNodeId,fwWeight, rvWeight, nameId" << std::endl;
    std::cout << pn3.id() << pn3.point().x() << pn3.point().y() << pn3.forwNodeId() << pn3.reveNodeId() <<  pn3.forwWeight() <<  pn3.reveWeight() <<  pn3.nameId()<< std::endl;

    BOOST_REQUIRE_EQUAL(pn3.id(),pn.id());
    BOOST_REQUIRE_EQUAL(pn3.point().x(),pn.point().x());
    BOOST_REQUIRE_EQUAL(pn3.point().y(),pn.point().y());
    BOOST_REQUIRE_EQUAL(pn3.forwNodeId(),pn.forwNodeId());
    BOOST_REQUIRE_EQUAL(pn3.reveNodeId(),pn.reveNodeId());
    BOOST_REQUIRE_EQUAL(pn3.forwWeight(),pn.forwWeight());
    BOOST_REQUIRE_EQUAL(pn3.reveWeight(),pn.reveWeight());
    BOOST_REQUIRE_EQUAL(pn3.nameId(),pn.nameId());

    std::cout << "Testing inSameStreet function:"<< std::endl;
    BOOST_REQUIRE_EQUAL(pn.inSameStreet(pn2),true);


}

BOOST_AUTO_TEST_SUITE_END()
