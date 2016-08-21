/*VRP*********************************************************************
 *
 * vehicle routing problems
 *      A collection of C++ classes for developing VRP solutions
 *      and specific solutions developed using these classes.
 *
 * Copyright 2014 Stephen Woodbridge <woodbri@imaptools.com>
 * Copyright 2014 Vicky Vergara <vicky_vergara@hotmail.com>
 *
 * This is free software; you can redistribute and/or modify it under
 * the terms of the MIT License. Please file LICENSE for details.
 *
 ********************************************************************VRP*/
#include <iostream>
#include <deque>


#include "order.h"
#include "orders.h"
#include "twpath.h"
#include "twc.h"
#include "plot.h"
#include "vehicle.h"

#include <sstream>


//antes del from no lo voy a poner
int   Vehicle::getPosLowLimit(int nid ,int from ,const TWC<Dpnode> &twc )const {
       int RL;
       for (RL=size(); RL>from  and twc.isCompatibleIJ( nid , path[RL-1].getnid() );RL--) {};
std::cout<<"RL"<<RL<<"\t";
       return RL;
}

int   Vehicle::getPosHighLimit(int nid ,int from, int to, const TWC<Dpnode> &twc) const {
//despues del to no lo voy a poner
       int LR;
       for (LR=from ; LR<to and twc.isCompatibleIJ( path[LR].getnid(), nid ) ; LR++) {};
std::cout<<"LR"<<LR<<"\n";
       return LR;
}
       

double Vehicle::testInsertLIFO(const Order &order, const Orders &orders, int &pickPos, int &delPos,const TWC<Dpnode> &twc) {
std::cout<<"***************LIFO********************\n";
       //lastDelivery 
       //lastPickup
       //check if its D's -> O
       int oid=order.getoid();
       int route_oid;
       bool flag=true;
       double bestCost=std::numeric_limits<double>::max();
       int pLR,pRL,dLR,dRL;
       int pickId=order.getpid();
       int delId=order.getdid();
       int i,j;

       for (i=lastPickup+1;i<=lastDelivery  ;i++) {
std::cout<<"pick en"<<i<<"\n";
std::cout<<"from "<<getnid(i-1)<<" to "<<pickId<<" Compat="<<( twc.isCompatibleIJ(getnid(i-1),pickId) ?"YES\n":"no\n");
            if (not twc.isCompatibleIJ(getnid(i-1),pickId) ) break;
            path.insert(order.getPickup(),i);
            path.evaluate(i-1,maxcapacity);

            for (j=i+1 ; j<=lastDelivery+1  ;j++) {
                std::cout<<"\t\tdeliver en "<<j<<"\n";
                std::cout<<"from "<<delId<<" to "<<getnid(j)<<" Compat="<<( twc.isCompatibleIJ(delId,getnid(j)) ?"YES\n":"no\n");
                if (not twc.isCompatibleIJ(delId,getnid(j))) break;
     
                path.insert(order.getDelivery(),j);
                path.evaluate(j-1,maxcapacity);
                evalLast();
                if (feasable()) {
                     bestCost = getcost();
                     pickPos=i;
                     delPos=j;
tau();
std::cout<<"FEASABLE pick en"<<pickPos<<"\t";
std::cout<<"deliver en "<<delPos<<"\n";
                     e_erase(pickPos,delPos);                      // are trying go back to normal
                     return bestCost;
                }
                path.erase(j);
            }
            path.erase(i);
       }
       path.evaluate(lastPickup,maxcapacity);
       evalLast();
       delPos=pickPos=-1;
std::cout<<"***************LIFO********************\n";
       return bestCost;

       std::cout<<"pick en"<<i-1<<"\n";
       std::cout<<"\t\tdeliver en "<<j<<"\n";

      

std::cout<<"lastPick "<<lastPickup<<"\t lastDelivery "<<lastDelivery<<"\n";

       if (lastPickup==lastDelivery-1) {
std::cout<<"test Insert LIFO over a last node\n";
           pickPos=2;delPos=3;
           bestCost=tryInsertPOS(order,pickPos,delPos);
           return bestCost;
       }

std::cout<<"test Insert LIFO looking for best position---\n";


       pRL=getPosLowLimit(pickId,lastPickup,twc);
       pLR=getPosHighLimit(pickId,lastPickup,lastDelivery,twc);

       if (pRL>pLR) {
std::cout<<"pRL"<<pRL<<"\t";
std::cout<<"pLR"<<pLR<<"\n";
            pickPos=-1; delPos=-1;
            return bestCost;
       }

       dRL=getPosLowLimit(delId,lastPickup,twc);
       dLR=getPosHighLimit(delId,lastPickup,lastDelivery,twc);

       if (dRL>dLR) {
std::cout<<"dRL"<<dRL<<"\t";
std::cout<<"dLR"<<dLR<<"\n";
            pickPos=-1; delPos=-1;
            return bestCost;
       }
       for (i=pRL+1; i<=pLR;i++) {
           insert(order.getPickup(),i);
           path.evaluate(i-1,maxcapacity);
std::cout<<"test Insert FIFO looking for best position---"<<i<<"\n";
tau();std::cout<<"\n";
           if (dRL<=lastDelivery) j=lastDelivery+1;
           else j=dRL;
std::cout<<"Inner Loop---"<<j<<"\n";
tau();std::cout<<"\n";
           while (j<=dLR+2) {
                insert(order.getDelivery(),j);
std::cout<<"Inner Loop---"<<j<<"\n";
tau();std::cout<<"\n";
                path.evaluate(j-1,maxcapacity);
                evalLast();
                if (feasable()) {
                     cost = getcost();
                     pickPos=i;
                     delPos=j;
                     e_erase(pickPos,delPos);                      // are trying go back to normal
                     return cost;
                }
                path.erase(j);
                j++;
            }
            path.erase(i);
        }
       return cost;
}



double Vehicle::testInsertFIFO(const Order &order, const Orders &orders, int &pickPos, int &delPos,const TWC<Dpnode> &twc) {
std::cout<<"***************FIFO********************\n";
       //lastDelivery 
       //lastPickup
       //check if its D's -> O
       int oid=order.getoid();
       int route_oid;
       bool flag=true;
       double bestCost=std::numeric_limits<double>::max();
       int pLR,pRL,dLR,dRL;
       int pickId=order.getpid();
       int delId=order.getdid();
       int i,j;

       for (i=lastPickup+1;i<=lastDelivery  ;i++) {
std::cout<<"pick en"<<i<<"\n";
std::cout<<"from "<<getnid(i-1)<<" to "<<pickId<<" Compat="<<( twc.isCompatibleIJ(getnid(i-1),pickId) ?"YES\n":"no\n");
            if (not twc.isCompatibleIJ(getnid(i-1),pickId) ) break;
            path.insert(order.getPickup(),i);
            path.evaluate(i-1,maxcapacity);

            for (j=lastDelivery+1 ; j<size()  ;j++) {
std::cout<<"\t\tdeliver en "<<j+1<<"\n";
std::cout<<"from "<<delId<<" to "<<getnid(j)<<" Compat="<<( twc.isCompatibleIJ(getnid(j),delId) ?"YES\n":"no\n");
                if (not twc.isCompatibleIJ(getnid(j-1),delId)) break;
                path.insert(order.getDelivery(),j+1);
                path.evaluate(j,maxcapacity);
                evalLast();
dump();
                if (feasable()) {
                     bestCost = getcost();
                     pickPos=i;
                     delPos=j+1;
                     e_erase(pickPos,delPos);                      // are trying go back to normal
                     return bestCost;
                }
                path.erase(j);
            }
            path.erase(i);
       }
       path.evaluate(lastPickup,maxcapacity);
       evalLast();
       delPos=pickPos=-1;
std::cout<<"***************FIFO********************\n";
       return bestCost;
       std::cout<<"pick en"<<i-1<<"\n";
       std::cout<<"\t\tdeliver en "<<j<<"\n";

std::cout<<"lastPick "<<lastPickup<<"\t lastDelivery "<<lastDelivery<<"\n";

       if (lastPickup==lastDelivery-1) {
std::cout<<"test Insert FIFO on last entries\n";
           pickPos=2;delPos=4;
           bestCost=tryInsertPOS(order,pickPos,delPos);
           return bestCost;
       }

std::cout<<"test Insert FIFO looking for best position---\n";

       pRL=getPosLowLimit(pickId,lastPickup,twc);
       pLR=getPosHighLimit(pickId,lastPickup,lastDelivery,twc);

       dRL=getPosLowLimit(delId,lastDelivery,twc);
       dLR=getPosHighLimit(delId,lastDelivery,size(),twc);

std::cout<<"pRL"<<pRL<<"\t";
std::cout<<"pLR"<<pLR<<"\n";
std::cout<<"dRL"<<dRL<<"\t";
std::cout<<"dLR"<<dLR<<"\n";
       for (i=pRL+1; i<=pLR;i++) {
           insert(order.getPickup(),i);
           path.evaluate(i-1,maxcapacity);
std::cout<<"test Insert FIFO looking for best position---"<<i<<"\n";
tau();std::cout<<"\n";
           if (dRL<=lastDelivery) j=lastDelivery+1;
           else j=dRL;
std::cout<<"Inner Loop---"<<j<<"\n";
tau();std::cout<<"\n";
           while (j<=dLR+2) {
                insert(order.getDelivery(),j);
std::cout<<"Inner Loop---"<<j<<"\n";
tau();std::cout<<"\n";
                path.evaluate(j-1,maxcapacity);
                if (feasable()) {
                     cost = getcost();
                     pickPos=i;
                     delPos=j;
                     e_erase(pickPos,delPos);                      // are trying go back to normal
                     return cost;
                }
                path.erase(j);
                j++;
            }
            path.erase(i);
        }
       return cost;
}





double Vehicle::testInsertPUSH(const Order &order, const Orders &orders, int &pickPos, int &delPos,const TWC<Dpnode> &twc) {
std::cout<<"***************PUSH********************\n";
       //lastDelivery 
       //lastPickup
       //check if its D's -> O
       int oid=order.getoid();
       int route_oid;
       double bestCost=std::numeric_limits<double>::max();
       int pLR,pRL,dLR,dRL;
       int pickId=order.getpid();
       int delId=order.getdid();
       int i,j;

       for (i=lastDelivery+1;i<=size()  ;i++) {
std::cout<<"pick en"<<i<<"\n";
std::cout<<"from "<<getnid(i-1)<<" to "<<pickId<<" Compat="<<( twc.isCompatibleIJ(getnid(i-1),pickId) ?"YES\n":"no\n");
            if (not twc.isCompatibleIJ(getnid(i-1),pickId) ) break;
            path.insert(order.getPickup(),i);
            path.evaluate(i-1,maxcapacity);

            for (j=i+1 ; j<=size()  ;j++) {
std::cout<<"\t\tdeliver en "<<j<<"\n";
std::cout<<"from "<<delId<<" to "<<getnid(j-1)<<" Compat="<<( twc.isCompatibleIJ(getnid(j-1),delId) ?"YES\n":"no\n");
                if (not twc.isCompatibleIJ(getnid(j-1),delId)) break;
                path.insert(order.getDelivery(),j);
                path.evaluate(j-1,maxcapacity);
                evalLast();
dump();
                if (feasable()) {
                     bestCost = getcost();
                     pickPos=i;
                     delPos=j;
                     e_erase(pickPos,delPos);                      // are trying go back to normal
                     return bestCost;
                }
                path.erase(j);
            }
            path.erase(i);
       }
       path.evaluate(lastPickup,maxcapacity);
       evalLast();
       delPos=pickPos=-1;
std::cout<<"***************PUSH********************\n";
       return bestCost;
}

double Vehicle::tryInsertPOS(const Order &order, int pickPos, int delPos) {
       double cost=std::numeric_limits<double>::max();  //suposse unfeasable
       if (pickPos>=delPos) return cost;                //pick after delivery is unfeasable
       path.insert(*order.pickup,pickPos);
       path.insert(*order.delivery,delPos);
       path.evaluate(pickPos,maxcapacity);
       evalLast();
tau();std::cout<<"\n";
       if (feasable()) cost = getcost();   // came out feasable()
       e_erase(pickPos,delPos);                      // are trying go back to normal
       return cost;
}

double Vehicle::insertPOS(const Order &order, int pickPos, int delPos) {
       double cost=std::numeric_limits<double>::max();  //suposse unfeasable
       if (pickPos>=delPos) return cost;      //Throw (we are not trying) pick after delivery is impossible
       insert(order.getPickup(),pickPos);
       insert(order.getDelivery(),delPos);
       path.evaluate(pickPos,maxcapacity);
       evalLast();
       if (feasable()) cost = getcost();   // came out feasable()
       else {};   // throw came out unfeasable
       lastPickup=pickPos;
       lastDelivery=delPos;
       return getcost();
}


double Vehicle::e_erase(int pickPos, int delPos) {
       path.erase(delPos);
       path.erase(pickPos);
       path.evaluate(pickPos,maxcapacity);
       evalLast();
       return getcost();
}

bool Vehicle::e_feasable4(const Dpnode& n1, const Dpnode& n2,const Dpnode& n3,const Dpnode& n4) {
       path.resize(1);
       path.insert(n1,1);
       path.insert(n2,2);
       path.insert(n3,3);
       path.insert(n4,4);
       path.evaluate(1,maxcapacity);
       evalLast();
tau();std::cout<<(feasable()?"YES":"no")<<"\n";
       return feasable();
}



// *******
bool Vehicle::insertOrderAfterLastPickup(const Order &order,const TWC<Dpnode> &twc){
    int i;
    for (i=size()-1; i>=0 and not ispickup(i); i--) {};
    int lastPickPos=i+1;

    int deliverPosLR=0;
    int pickPosLR=0;
    int deliverPosRL=size()-1;
    int pickPosRL=size()-1;
    int Px= order.getpid();
    int Dx= order.getdid();
    for  (int i=0; i<size(); i++) {
        pickPosLR=i;
        if ( twc.isCompatibleIJ(getnid(i),Px) ) continue;
        else {break; };
    };

    for (int i=size()-1; i>=0;i--) {
        pickPosRL=i;
        if ( twc.isCompatibleIJ(Px,getnid(i)) ) continue;
        else {break; };
    };

    if (pickPosRL>pickPosLR) std::cout<<" SOMETHING WENT WRONG 1 \n";
    else std::cout<<"\nCan place the picup from pick Pos RL="<<++pickPosRL<<"\t up to pick Pos LR="<<pickPosLR<<"\n";

    for  (int i=0; i<size();i++) {
        deliverPosLR=i;
        if ( twc.isCompatibleIJ(getnid(i),Dx) ) continue;
        else {break;};
    };
    for (int i=size()-1; i>=0;i--) {
        deliverPosRL=i;
        if ( twc.isCompatibleIJ(Dx,getnid(i)) ) continue;
        else {break; };
    };
    // teoricamente Rl<LR
    if (deliverPosLR<deliverPosRL) std::cout<<" SOMETHING WENT WRONG 2";
std::cout<<"\n can place the deliver from Pos RL="<<++deliverPosRL<<"\t up to deliver Pos LR="<<deliverPosLR<<"\n";
    double bestCost= std::numeric_limits<double>::max();
    int bestI,bestJ;
    for (int i=pickPosRL; i<=pickPosLR+1;i++) {
std::cout<<"CYCLE"<<i<<"\n";
        insert(order.getPickup(),i);
        insert(order.getDelivery(),i+1);
        for (int j=i+1; j<=deliverPosLR+2;j++) {
std::cout<<"        CYCLE"<<j<<"\n";
tau(); std::cout<<"\n";
            if (getcost()<bestCost) {
                bestCost=getcost();
                bestI=i; bestJ=j;
            }
            swap(j,j+1);
        };
tau(); std::cout<<"\n";
        e_erase(size()-1);
        e_erase(i);
tau(); std::cout<<"\n";
    }
           
tau(); std::cout<<"\n";
    insert(order.getPickup(),bestI);
tau(); std::cout<<"\n";
    insert(order.getDelivery(),bestJ);
dump(); std::cout<<"<---- salgo con\n"<<(feasable()? "FEASABLE":"NOT FEASABLE");
    return feasable();
}



int Vehicle::findBetterDeliveryForward(const int ppos,const int dpos,double &bestcost) {
           int bestJ=-1;
           int deliveryPos=dpos;
           if ( not (ppos<dpos and sameorder(ppos,dpos) and ispickup(ppos) and isdelivery(dpos) ))  return -1; //thoerically we never get to this point if called from funtion bellow
           for (int j=ppos+1; j<path.size(); j++) {
                  move(deliveryPos,j); deliveryPos=j;   
                  if (getcost()<bestcost and feasable()){
                             bestcost=getcost();
                             bestJ=j;
                  }
           }
           return  bestJ;
}


double Vehicle::costBetterPickupBackward(int &bppos, int &bdpos) {
           double bestcost=getcost();
           int ppos=bppos;
	   int dpos=bdpos;
	   int bestI=-1;
           int bestJ=-1;
           int j;
           if ( not (ppos<dpos and sameorder(ppos,dpos) and ispickup(ppos) and isdelivery(dpos)) ) return bestcost;
           for (int i=1;i<dpos;i++) { //ensuring pickup comes before delivery
              j=-1;
              move (ppos, i); ppos=i;               
              j=findBetterDeliveryForward(ppos,dpos,bestcost);
              if (j>0) { //a better cost was found
                    bestI=i;
                    bestJ=j;
             }
           }
           if ( bestI=bestJ=-1 ) {
                 bppos=ppos;
                 bdpos=dpos;
                 return bestcost;                    //no better cost was found
           } else {
                 bppos=bestI;
                 bdpos=bestJ;
                 return bestcost;
           }
}
          


double Vehicle::findBestCostBackForw(const int oid,int &bppos,int &bdpos){
    int ppos=getppos(oid);  //actual positions and costs as best
    int dpos=getdpos(oid);
    double actualcost=getcost();
    double bestcost=actualcost;
    bppos=ppos; bdpos=dpos; 
    bestcost=costBetterPickupBackward(bppos, bdpos);
    return bestcost;
}




/***************************** DUMP PRINTS PLOTS   ********/

   void Vehicle::dump() const {
     for (int i=0;i<path.size();i++){
         std::cout<<"\npath stop #:"<<i<<"\n";
          path[i].dumpeval();
     }
     std::cout<<"\nBack to depot:"<<"\n";
     backToDepot.dumpeval();
     std::cout <<"TOTAL COST="<<cost <<"\n";
   }


   void Vehicle::smalldump() const {
      backToDepot.dumpeval();
      std::cout << "TOTAL COST="<<cost << ", TAU= ";
      tau(); std::cout<<"\n";
   }

   void Vehicle::tau() const {
      for (int i=0; i< path.size(); i++)
         std::cout<<getnid(i)<<" , ";
      std::cout<<" (";
      for (int i=0; i< path.size(); i++)
         std::cout<<getid(i)<<" , ";
      std::cout<<" )";
   }



int Vehicle::findForwardImprovment(const int i,double &bestcost) {
           bool improved=false;
           int bestJ=-1;
           if (isdepot(i)) return -1;
           for (int j=i+1; j<path.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
                  move(i,j);
                  if (getcost()<bestcost){
                             bestcost=getcost();
                             bestJ=j;
                  }
                  move(j,i);
           }
           return  bestJ;
}


bool Vehicle::findImprovment(int i) {
           double oldcost= getcost();
           bool oldfeasable= feasable();
           bool improved=false;
           if (isdepot(i)) return false; //should never arrive here if order is inserted
           for (int j=i+1; j<path.size() and !(ispickup(i) and isdelivery(j) and sameorder(i,j)); j++) {
               swapstops(i,j);
std::cout<<"\n testing  and is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
               if (feasable() and not oldfeasable)    return true;
               if (getcost()<oldcost and feasable())  return true;
                   else  swap(i,j);
           }
           return false;
}


void Vehicle::hillClimbOpt() {
           int i=1;
           double bestCost=getcost();
std::cout<<"\nentering hill, initialy is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
           while (i<path.size()-1) {
              if (!findImprovment(i)) {
                 i++;
              }
              else { 
std::cout<<"\nimprovement found is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
                 i=0;
              }
           }
std::cout<<"\n\texiting hill, final is "<<(feasable()? "FEASABLE":"unfeasable")<<"---\t";
tau();
}



void Vehicle::findBetterForward(int &bestI, int &bestJ) {
           double bestcost=getcost();
           int j=-1;
           for (int i=1;i<path.size()-1;i++) { //not even bothering with the depot
              j=-1;
              j=findForwardImprovment(i,bestcost); 
              if (j>0) { //a better cost was found
                    bestI=i;
                    bestJ=j;
             }
           }
}


/* to forget about evaluation everything done in Vehicle has to be evaluated */
    
    
    
    
/********* MOVEMENT OF PATHSTOPS WITHIN PATH  *****/

/****** removal of nodes from the path  ********/

/***  direct evaluation **/
/****** Indirect evaluation *****/    

    void Vehicle::removeOrder(const Order &order){
         removeOrder(order.getoid());
    }
    
    void Vehicle::removeOrder(const int oid){
         removePickup(oid);
         removeDelivery(oid);
    }

    int Vehicle::pos(int nid) const{return path.pos(nid);}
             
    bool Vehicle::in(int nid) const{
         return ( (path.pos(nid)!=-1) ? true:false );
    }
              

/* O(n) */
    void Vehicle::removePickup(int oid){
          for (int at=0;at<path.size();at++) {
               if (ispickup(at) and getoid(at)==oid ){
                   e_erase(at); break; 
               }
         }
   }

/* O(n) */
    void Vehicle::removeDelivery(int oid){
           for (int at=0;at<path.size();at++) {
               if (isdelivery(at) and getoid(at)==oid ){
                   e_erase(at); break; //only 1 delivery per order
               }
           }
    }






bool Vehicle::isEmptyTruck() const {return path.size()==1;}





/****** Direct evaluation *****/    
    void Vehicle::e_push_back(const Dpnode &pathstop) {
          path.e_push_back(pathstop,maxcapacity);
          evalLast();
    }

   void Vehicle::e_erase(int at){
          if (!path.empty()) {
              path.e_remove(at,maxcapacity);
              evalLast();  
          }
    }

    void Vehicle::e_insert(const Dpnode &pathstop,int at) {
         path.e_insert(pathstop,at,maxcapacity);
         evalLast();
    }

    void Vehicle::e_move(int fromi,int toj) {
          if (fromi==toj) return; //nothing to move
          path.e_move(fromi,toj,maxcapacity);
          evalLast();
    }

    void Vehicle::e_clean() {
          path.resize(1);
          evalLast();
    }

       
    void Vehicle::e_swap(int i,int j){	
          if (i==j) return; //nothing to swap
          path.e_swap(i,j,maxcapacity);
          evalLast();
    }

/****** Indirect evaluation *****/    
    void  Vehicle::insertPickup   (const Order &o, const int at)  { e_insert(*o.pickup,at); }
    void  Vehicle::insertDelivery (const Order &o, const int at) { e_insert(*o.delivery,at); }

    
    bool  Vehicle::pushPickup(const Order &o)  { 
          push_back(*o.pickup); 
          evalLast();
          if (not feasable()) {
              e_erase(size()-1);
              return false;
          }
          return true;
    }

    void Vehicle::pushDelivery(const Order &o) { e_push_back(*o.delivery); }

    
    bool Vehicle::pushOrder( const Order &o) {
          push_back(*o.pickup); 
          push_back(*o.delivery); 
          evaluate(size()-2);
          if (not feasable()) {
              e_erase(size()-1);
              e_erase(size()-1);
              return false;
          }
          return true;
    }

    
/****** moves between pathstops  ********/

    void Vehicle::swapstops(int i,int j){
          if(i>j)  std::cout<<"This is a restrictive swap, requierment: i<j\n";  
          if ( ispickup(i) and isdelivery(j) and sameorder(i,j) ) {
               std::cout<<"This is a restrictive swap, requierment: cant swap from the same order\n";
               return;
          }
          e_swap(i,j);
     }

     

/***********************   EVALUATION **************************/


   void Vehicle::evaluate(int from) {
     assert(from < size());
     path.evaluate(from, maxcapacity);
     evalLast();
   };

   void Vehicle::evalLast() {
      Dpnode last=path[path.size()-1];
      backToDepot.evaluate(last,maxcapacity);
      cost= w1*backToDepot.gettotDist()+ w2*backToDepot.getcvTot() + w3*backToDepot.gettwvTot();
   }





/**************************************PLOT************************************/
void Vehicle::plot(std::string file,std::string title,int carnumber){
//std::cout<<"USING VEHICLE PLOT\n";
    Twpath<Dpnode> trace=path;
    trace.push_back(backToDepot);
    
    /** cpp11  the following next 3 lines become std::string carnum=std::to_string(carnumber */
    std::stringstream convert; 
    convert << carnumber;
    std::string carnum = convert.str();
    std::string extra=file+"vehicle"+carnum ;

    Plot<Dpnode> graph( trace ); 
    graph.setFile( file+extra+".png" );
    graph.setTitle( title+extra );
    graph.drawInit();
    for (int i=0; i<size(); i++){
        if (ispickup(i))  { 
             graph.drawPoint(path[i], 0x0000ff, 9, true);
        } else if (isdelivery(i)) {
             graph.drawPoint(path[i], 0x00ff00, 5, true);
        } else  {
             graph.drawPoint(path[i], 0xff0000, 7, true);
        }
    }
    plot(graph,carnumber);
    graph.save();
}

void Vehicle::plot(Plot<Dpnode> graph, int carnumber){
//std::cout<<"USING VEHICLE PLOT  1\n";
    Twpath<Dpnode> trace=path;
    trace.push_back(backToDepot);
    graph.drawPath(trace,graph.makeColor(carnumber*10), 1, true);
}



std::deque<int> Vehicle::getpath() const {
    std::deque<int> p=path.getpath();
    p.push_back(backToDepot.getnid());
    return p;
}

