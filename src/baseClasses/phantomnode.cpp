#include "phantomnode.h"

PhantomNode::PhantomNode(UID phantomNodeId, double x, double y, UID fwNodeId, UID rvNodeId, UID fwWeight, UID rvWeight, UID nameId) {
    mPhantomNodeId = phantomNodeId;
    mPoint = Point(x, y);
    mForwNodeId = fwNodeId;
    mReveNodeId = rvNodeId;
    mForwWeight = fwWeight;
    mReveWeight = rvWeight;
    mNameId = nameId;
}

PhantomNode::PhantomNode(const PhantomNode &other) {
    mPhantomNodeId = other.mPhantomNodeId;
    mPoint = other.mPoint;
    mForwNodeId = other.mForwNodeId;
    mReveNodeId = other.mReveNodeId;
    mForwWeight = other.mForwWeight;
    mReveWeight = other.mReveWeight;
    mNameId = other.mNameId;
}

bool PhantomNode::inSameStreet(const PhantomNode &other)
{
    return (
        mNameId == other.mNameId &&
            (
                (mForwNodeId == other.mForwNodeId && mReveNodeId == other.mReveNodeId) ||
                (mForwNodeId == other.mReveNodeId && mReveNodeId == other.mForwNodeId)
            )
    );
}

PhantomNode& PhantomNode::operator= (const PhantomNode &other)
{
    //PhantomNode tmp(other);
    //*this = tmp;
    mPhantomNodeId = other.id();
    mPoint = other.point();
    mForwNodeId = other.forwNodeId();
    mReveNodeId = other.reveNodeId();
    mForwWeight = other.forwWeight();
    mReveWeight = other.reveWeight();
    mNameId = other.nameId();
    return *this;
    /*
    mPhantomNodeId = other.mPhantomNodeId;
    mX = other.mX;
    mY = other.mY;
    mForwNodeId = other.mForwNodeId;
    mReveNodeId = other.mReveNodeId;
    mForwWeight = other.mForwWeight;
    mReveWeight = other.mReveWeight;
    mNameId = other.mNameId;
    */
}
