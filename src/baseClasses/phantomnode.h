#ifndef PHANTOMNODE_H
#define PHANTOMNODE_H

#include <basictypes.h>

class Point
{
public:
  Point() {}
  Point(double x, double y) {
      mX = x;
      mY = y;
  }
  double x () { return mX; }
  double y () { return mY; }

private:
  double mX;
  double mY;
};


/*! \class PhantomNode
 * \brief This class implement information of Phantom Nodes
 *
 * OSRM phantom nodes are virtual nodes. Nearest point in an edge from input point
 *
 */
class PhantomNode
{
public:
    PhantomNode() {}
    PhantomNode( UID phantomNodeId ) { mPhantomNodeId = phantomNodeId; }
    PhantomNode( UID phantomNodeId, double x, double y, UID fwNodeId, UID rvNodeId, UID fwWeight, UID rvWeight, UID nameId );
    PhantomNode( const PhantomNode &other );

    void setId(UID phantomNodeId) { mPhantomNodeId = phantomNodeId; }
    UID id () { return mPhantomNodeId; }

    void setX (double x) { mX = x; }
    double x () { return mX; }

    void setY (double y) { mY = y; }
    double y () { return mY; }

    void setForwNodeId(UID fwNodeId) { mForwNodeId = fwNodeId; }
    UID forwNodeId () { return mForwNodeId; }

    void setReveNodeId(UID rvNodeId) { mReveNodeId = rvNodeId; }
    UID reveNodeId () { return mReveNodeId; }

    void setForwWeight(UID fwWeight) { mForwWeight = fwWeight; }
    UID forwWeight () { return mForwWeight; }

    void setReveWeight(UID rvWeight) { mReveWeight = rvWeight; }
    UID reveWeight () { return mReveWeight; }

    void setNameId(UID nameId) { mNameId = nameId; }
    UID nameId () { return mNameId; }

    void setBeforePNode (Point p) { mBeforePNode = p; }
    Point beforePNode () { return mBeforePNode; }

    void setAfterPNode (Point p) { mAfterPNode = p; }
    Point afterPNode () { return mAfterPNode; }

    PhantomNode& operator= ( const PhantomNode &other );

private:
    UID mPhantomNodeId;     ///< internal node number
    double mX;              ///< Longitude
    double mY;              ///< Latitude
    UID mForwNodeId;        ///< OSRM forward node id
    UID mReveNodeId;        ///< OSRM reverse node id
    UID mForwWeight;        ///< OSRM forward weight
    UID mReveWeight;        ///< OSRM reverse weight
    UID mNameId;            ///< OSRM name id
    Point mBeforePNode;     ///< Longitude and latitude of point before phantom node
    Point mAfterPNode;      ///< Longitude and latitude of point after phantom node
};

#endif // PHANTOMNODE_H
