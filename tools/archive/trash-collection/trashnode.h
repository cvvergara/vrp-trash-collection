#ifndef TRASHNODE_H
#define TRASHNODE_H

#include "tweval.h"

class Trashnode : public Tweval {
  protected:
    int ntype;              // node type (0=depot, 1=dump, 2=pickup)
    double depotdist;       // distance to nearest depot
    long int depotnid;      // nid of the closet depot
    double depotdist2;      // distance to nearest depot
    long int depotnid2;     // nid of the closet depot
    double dumpdist;        // distance to nearest dump
    long int dumpnid;       // nid of closet dump


  public:
    // accessors
    double getdepotdist() const {return depotdist;};
    long int getdepotnid() const {return depotnid;};
    double getdepotdist2() const {return depotdist2;};
    long int getdepotnid2() const {return depotnid2;};
    double getdumpdist() const {return dumpdist;};
    long int getdumpnid() const {return dumpnid;};
    void dump() const;

    // state
    bool isdepot() const {return ntype==0;};
    bool isdump() const {return ntype==1;};
    bool ispickup() const {return ntype==2;};
    bool isvalid() const;

    // mutators
    void setvalues(int _nid, double _x, double _y, int _demand,
                   int _tw_open, int _tw_close, int _service, int _ntype);
    void setntype(int _ntype) { ntype = _ntype; };
    void setdepotdist(int _nid, double _dist, int _nid2, double _dist2);
    void setdumpdist(int _nid, double _dist);

    // structors
/*    Trashnode(int _nid, double _x, double _y, int _demand,
              int _tw_open, int _tw_close, int _service, int _ntype) 
              : Tweval(_nid, _x, _y, _demand, _tw_open, _tw_close, _service) {
        // in Trashnode
        ntype = _ntype;
        depotdist = 0.0;
        depotnid = -1;
        depotdist2 = 0.0;
        depotnid2 = -1;
        dumpdist = 0.0;
        dumpnid = -1;
    };
*/
    Trashnode() : Tweval() {
        ntype = -1;
        depotdist = 0.0;
        depotnid = -1;
        depotdist2 = 0.0;
        depotnid2 = -1;
        dumpdist = 0.0;
        dumpnid = -1;
    };

    Trashnode(std::string line);

    ~Trashnode() {};

};

#endif
