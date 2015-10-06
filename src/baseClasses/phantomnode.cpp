#include "phantomnode.h"

PhantomNode::PhantomNode(UID phantomNodeId, double x, double y, UID fwNodeId, UID rvNodeId, UID fwWeight, UID rvWeight, UID nameId) {
    mPhantomNodeId = phantomNodeId;
    mX = x;
    mY = y;
    mForwNodeId = fwNodeId;
    mReveNodeId = rvNodeId;
    mForwWeight = fwWeight;
    mReveWeight = rvWeight;
    mNameId = nameId;
}

PhantomNode::PhantomNode(const PhantomNode &other) {
    mPhantomNodeId = other.mPhantomNodeId;
    mX = other.mX;
    mY = other.mY;
    mForwNodeId = other.mForwNodeId;
    mReveNodeId = other.mReveNodeId;
    mForwWeight = other.mForwWeight;
    mReveWeight = other.mReveWeight;
    mNameId = other.mNameId;
}

PhantomNode& PhantomNode::operator= (const PhantomNode &other)
{
    PhantomNode tmp(other);
    *this = tmp;
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
