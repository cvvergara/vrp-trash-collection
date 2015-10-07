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
<<<<<<< HEAD
    this->mPhantomNodeId = other.mPhantomNodeId;
    this->mPoint = other.mPoint;
    this->mForwNodeId = other.mForwNodeId;
    this->mReveNodeId = other.mReveNodeId;
    this->mForwWeight = other.mForwWeight;
    this->mReveWeight = other.mReveWeight;
    this->mNameId = other.mNameId;
=======
    //PhantomNode tmp(other);
    //*this = tmp;
    mPhantomNodeId = other.id();
    mPoint = other.point();
    mForwNodeId = other.forwNodeId();
    mReveNodeId = other.reveNodeId();
    mForwWeight = other.forwWeight();
    mReveWeight = other.reveWeight();
    mNameId = other.nameId();
>>>>>>> 79ce6645d32029ab58b1c56c9b026e34068e4252
    return *this;
}
