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
#include <algorithm>
#include <cstdlib>

#include "trashstats.h"
#include "optsol.h"
#include "tabuopt.h"


/**
    This Tabu search algorithm was adapted from the paper:

    "Tabu Search Techniques for the Hetrogeneous Vehicle Routing
    Problem with Time Windows and Carrier-Dependent Cots" by
    Sara Ceschia, Luca Di Gaspero, and Andrea Schaerf

    We use a sequential solving strategy for combining our three neighborhood
    move functions. This as know as a "token-ring" search. Given an initial
    state and a set of algorithms, it makes circularly a run at each algorithm,
    always starting from the best solution found by the previous one. The
    overall process stops either when a full round of the algorithms does not
    find an improvement or the time (aka: interation count) granted has elapsed.
    
    Each single algorithm stops when it does not improve the current best
    solution for a given number of iterations (ie: stagnation).

*/
void TabuOpt::search() {
#ifndef TESTED
std::cout<<"Entering TabuOpt::search() \n";
#endif

    std::deque<Move> aspirationalTabu;
    std::deque<Move> nonTabu;
    std::deque<Move> tabu;
    currentIteration = 0;
    maxIteration = 1;

    Timer start;
    bool improvedBest;
    int lastImproved = 0;
    bestSolution.optimizeTruckNumber();
    std::cout << "TABUSEARCH: Removal of truck time: " << start.duration() << std::endl;
    start.restart();
    TabuList.clear();

    do {
        std::cout << "TABUSEARCH: Starting iteration: " << currentIteration << std::endl;

        // this is a token ring search
        improvedBest  = doNeighborhoodMoves(Ins,     limitIns*5, aspirationalTabu, nonTabu,tabu);
        //improvedBest |= doNeighborhoodMoves(InterSw, limitInterSw*5, aspirationalTabu, nonTabu, tabu);
        //improvedBest |= doNeighborhoodMoves(IntraSw, limitIntraSw*100, aspirationalTabu, nonTabu,tabu);

        if (improvedBest) lastImproved = 0;
        else ++lastImproved;

        std::cout << "TABUSEARCH: Finished iteration: " << currentIteration
            << ", improvedBest: " << improvedBest
            << ", run time: " << start.duration()
            << std::endl;

        STATS->set("0 Iteration", currentIteration);
        STATS->set("0 Best Cost After", bestSolution.getCost());
        dumpStats();
        std::cout << "--------------------------------------------\n";
    }
    //while (improvedBest and ++currentIteration < maxIteration);
    while (lastImproved < 1 and ++currentIteration < maxIteration);

    std::cout << "TABUSEARCH: Total time: " << start.duration() << std::endl;
}





void TabuOpt::getNeighborhood( neighborMovesName whichNeighborhood, std::deque<Move> &neighborhood,double factor) const {
        neighborhood.clear();
#ifdef TESTED
std::cout<<"Entering TabuOpt::getNeighborhod() \n";
#endif
        Timer getNeighborhoodTimer;
        switch (whichNeighborhood) {
            case Ins:
                ++currentIterationIns;
                currentSolution.v_getInsNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("Ins", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
            case IntraSw:
                ++currentIterationIntraSw;
                currentSolution.v_getIntraSwNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("IntraSw", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
            case InterSw:
                ++currentIterationInterSw;
                currentSolution.v_getInterSwNeighborhood(neighborhood, factor);
                generateNeighborhoodStats("InterSw", getNeighborhoodTimer.duration(), neighborhood.size());
                break;
        }
#ifdef TESTED
std::cout<<"Exiting TabuOpt::getNeighborhod() \n";
#endif
}

bool TabuOpt::applyAmove(const Move &move) {

        currentSolution.v_applyMove(move); 
        makeTabu(move);
        computeCosts(currentSolution);
        if (bestSolutionCost <currentSolution.getCost()) {
        	bestSolution = currentSolution;
        	bestSolutionCost = bestSolution.getCost();
        	STATS->set("best Updated Last At", currentIteration);
        	STATS->inc("best Updated Cnt");
	}
        return true;
}



bool TabuOpt::applyAspirationalNotTabu(const Move  &move) {
	applyAmove(move); 			//has only one move
        STATS->inc("cnt Aspirational Not Tabu");
#ifndef TESTED
std::cout<<"\nAspirational non Tabu aplied ";move.Dump();
#endif
	return true;
}

bool TabuOpt::applyAspirationalTabu(std::deque<Move> &moves) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyAspirationalTabu \n";
#endif
	assert (moves.size());
	if (not moves.size()) return false;
        std::sort(moves.begin(), moves.end(), Move::bySavings); 
	applyAmove(moves[0]);
        STATS->inc("cnt Aspirational Tabu");
#ifndef TESTED
std::cout<<"\nAspirational  Tabu aplied ";moves[0].Dump();
#endif
/*
        STATS->set("best Updated Last At", currentIteration);
        STATS->inc("best Updated Cnt");

        currentSolution.v_applyMove(aspirationalTabu[0]);  //allways the best even if negative
        makeTabu(aspirationalTabu[0]);
        bestSolution = currentSolution;
        computeCosts(bestSolution);
        bestSolutionCost = bestSolution.getCost();
	addToStats(aspirationalTabu[0]);
*/
	moves.clear();
        return true;
}


bool TabuOpt::applyNonTabu (std::deque<Move> &notTabu) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyNonTabu() \n";
#endif
        assert (  notTabu.size() ) ;  //cant apply, there are non saved
        std::sort(notTabu.begin(), notTabu.end(), Move::bySavings); //the list is short so not so yikes (1 per local neighborhood)
std::cout << "\tapplyNonTabu: Not Tabu: "; notTabu[0].dump();
std::cout << "\n";

        currentSolution.v_applyMove(notTabu[0]);  //allways the best even if negative
        makeTabu(notTabu[0]);
	addToStats(notTabu[0]);
	return true;
}

bool  TabuOpt::applyTabu (std::deque<Move> &tabu) {
        assert ( tabu.size() );   //cant apply, there are non saved
	return applyTabu(tabu,0);
}

bool TabuOpt::applyTabu (std::deque<Move> &tabu, int strategy) {
#ifndef TESTED
std::cout<<"Entering TabuOpt::applyTabu #of possible moves:"<<tabu.size()<<"\n";
#endif
        assert( tabu.size() );  //cant apply, there are non saved
	
        if (strategy==0) {  //pick Best
            currentSolution.v_applyMove(tabu[0]);
            makeTabu(tabu[0]);
std::cout << "\tapplyTabu:  best: "; tabu[0].dump();
	    addToStats(tabu[0]);
	} else {
          int pickWorse = rand()% ( tabu.size()-1 );
std::cout << "\tapplyTabu: pickworse"<<pickWorse<<"\n"; tabu[pickWorse].dump();
            currentSolution.v_applyMove(tabu[pickWorse]);
            makeTabu(tabu[pickWorse]);
	    addToStats(tabu[pickWorse]);
	}
	tabu.clear();
#ifndef TESTED
std::cout<<"Exiting TabuOpt::applyTabu #of possible moves:"<<tabu.size()<<"\n";
#endif
	return true;
}

bool TabuOpt::reachedMaxCycles(int number, neighborMovesName whichNeighborhood) {
	bool limit;
        switch (whichNeighborhood) {
               case Ins: { limit = number>limitIns; }
               case IntraSw: { limit = number>limitIntraSw; }
               case InterSw: { limit = number>limitInterSw; }
        };
	return limit;
}

/*
    Algorithm for processing neighborhood moves [article]

    For the requested individual move neighborhood:
        doInsMoves, doIntraSwMoves, doInterSwMoves

    do {
        Generate the neighborhood of moves and order from best to worst.
        Working through the neighborhood (best to worst)
        Filter out inFeasible moves then
        if the move is aspirational we apply the move
        otherwise if the move is not Tabu apply the move
        even if it makes the current solution worse.
        If all moves are tabu, then apply the best one
    } until there are not valid moves or stagnation 
    return an indicator that we improved the best move or not.
*/

bool TabuOpt::doNeighborhoodMoves(neighborMovesName whichNeighborhood, int maxMoves, 
    std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu, std::deque<Move> &tabu) { 

#ifndef TESTED
std::cout<<"Entering TabuOpt::doNeighobrhoodMoves\n";
#endif
    bool improvedBest = false;
    int Cnt = 0;
    int CntNonAspirational =0;
    int CntNoNeighborhood =0;
    double factor = 0.5;
    bool limit;

    // we always start from the best solution of the last run
    //currentSolution = bestSolution;

    STATS->set("factor", factor);
    std::deque<Move> neighborhood;

    do {
        std::string solBefore = currentSolution.solutionAsText();

	getNeighborhood(whichNeighborhood,neighborhood,factor); 
	if (not neighborhood.size()) { 
		CntNoNeighborhood++; 
std::cout<<" No Moves are found  "<< CntNoNeighborhood <<"\n";
		if ( reachedMaxCycles(CntNoNeighborhood,whichNeighborhood) ) {
std::cout<<" Reached end of cycle -No moves found- "<<Cnt<<" out of "<< maxMoves<<"\n";
		   return improvedBest; //we cycled and no neighborhood moves were found
                } else continue; 
        };

	CntNoNeighborhood=0; 

        std::string solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);

	if ( classifyMoves(neighborhood, aspirationalTabu, notTabu, tabu) ) {
		assert (not neighborhood.size());
		assert (not aspirationalTabu.size());
		assert (not notTabu.size());
		assert (not tabu.size());
		continue;
        } else {
		if (aspirationalTabu.size() ) 	assert ( not notTabu.size() and not tabu.size() );
		if (notTabu.size() ) assert ( not aspirationalTabu.size() and not tabu.size() );
		if (tabu.size() ) assert ( not aspirationalTabu.size() and not notTabu.size() );
	}

#ifndef LOG
	dumpMoves("neighborhood",neighborhood);
	dumpMoves("aspirationalTabu",aspirationalTabu);
	dumpMoves("notTabu",notTabu);
	std::cout<<" Tabu size:  "<< tabu.size() <<"\n";
#endif

	if ( reachedMaxCycles(aspirationalTabu.size(),whichNeighborhood) ) {
		assert (not neighborhood.size());
		assert (aspirationalTabu.size());
		assert (not notTabu.size());
		assert (not tabu.size());

                Cnt++;
                applyAspirationalTabu( aspirationalTabu );
                //CntNonAspirational=0;
                //neighborhood.clear();
                //aspirationalTabu.clear();
                //tabu.clear();
                //notTabu.clear();

		assert (not aspirationalTabu.size());
                continue;
        }


	if ( reachedMaxCycles(notTabu.size(),whichNeighborhood) ) {
		assert (not neighborhood.size());
		assert (not aspirationalTabu.size());
		assert (notTabu.size());
		assert (not tabu.size());
                Cnt++;
		applyNonTabu( notTabu );
		//CntNonAspirational=0;
		neighborhood.clear();
                //aspirationalTabu.clear();
		//tabu.clear();	
		//notTabu.clear();	
		continue;
	} 
	//CntNonAspirational++;
	if (notTabu.size()) continue;
assert(true==false);
	
        solAfter  = currentSolution.solutionAsText();
        assert (solBefore==solAfter);
	if ( tabu.size()>0 and reachedMaxCycles(CntNonAspirational ,whichNeighborhood) ) {
	//if ( limit ) {  // we cycled thru all the local neighborhoods and no non aspirational nor non Tabu move was found 
                Cnt++;
		applyTabu( tabu,0 ); //random

		//CntNonAspirational=0;
		neighborhood.clear();
                aspirationalTabu.clear();
		tabu.clear();
		notTabu.clear();	
	        break;
       }  
    }	
    while ( Cnt < maxMoves );

#ifndef TESTED
std::cout<<" Moves made "<<Cnt<<" out of "<< maxMoves<<"\n";
std::cout<<"Exiting TabuOpt::doNeighobrhoodMoves\n";
#endif
    return improvedBest;
}

/**
  Classify the moves into:
	aspirational Not tabu	(if found the buckets bellow are cleared)
	aspirational tabu	(adds to the bucket the best move)
	not Tabu		(adds to the bucket the best move) 
	Tabu			(adds to the bucket all the moves)

  if during the classification a truck is removed:
	move is applied
	all buckets are cleared


	returns true: any kind of move was made
*/
bool TabuOpt::classifyMoves (std::deque<Move> &neighborhood,  std::deque<Move> &aspirationalTabu, std::deque<Move> &notTabu,std::deque<Move> &tabu) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::classifyMoves \n";
#endif
        Timer start;
//        bool allTabu = true;
	bool found = false;
	bool foundAspTabu = false;
	bool removedTruck = false;  
	double newCost;
	computeCosts(currentSolution);
        double actualCost= currentSolution.getCost();
        OptSol current=currentSolution;
        std::sort(neighborhood.begin(), neighborhood.end(), Move::bySavings); 

        for (std::deque<Move>::iterator it=neighborhood.begin();
                it!=neighborhood.end(); ++it) {
	    current=currentSolution;
            current.v_applyMove(*it);
	    removedTruck = computeCosts(current);
	    newCost = current.getCost();
	    if (removedTruck) {
		currentSolution=current;
                bestSolution = current;
                computeCosts(bestSolution);
                bestSolutionCost = newCost;
                makeTabu(*it);
		STATS->set("best Updated Last At", currentIteration);
                STATS->inc("best Updated Cnt");
		//clear up all buckets
                neighborhood.clear(); 
                aspirationalTabu.clear(); notTabu.clear(); tabu.clear();
		return true;
            }

#ifndef LOG
	    std::cout<<"isTabu??: "<<(isTabu(*it)?"YES\n":"NO\n");it->Dump();
	    if (not ( (std::abs( (actualCost - newCost)  -  it->getsavings())) <0.5) ) {
		it->Dump();
		std::cout<<"something is wrong with the savings****** "<<it->getsavings()<< " ***** "<<actualCost - newCost<<"\n";
            } 
#endif
            // if the move is aspirational and not tabu then we apply it
            if (newCost  < bestSolutionCost and not isTabu(*it) ) {  
		applyAspirationalNotTabu(*it);
		//clear up all buckets
                neighborhood.clear(); 
                aspirationalTabu.clear(); notTabu.clear(); tabu.clear();
	        return true;
		
            }
	   // if the move is aspirational, but tabu, we save it
	   if (newCost  < bestSolutionCost and not foundAspTabu) { //save only the best aspirational Tabu
		aspirationalTabu.push_back(*it);
		foundAspTabu = true;	
		//clear up remaining buckets 
                notTabu.clear(); tabu.clear();
	   } else if (not aspirationalTabu.size()  and not isTabu(*it) and not found ) { //save only the best nonTabu move when there is no aspirational tabu move
		notTabu.push_back(*it); 
		found = true;
		tabu.clear();  //no need to keep the tabus
		if (it->getsavings()<0) {neighborhood.clear(); return false; }  //we can stop now, no possible aspirational move can exist in this neighborhood
	    } else if (  not aspirationalTabu.size() and not notTabu.size() ) {  //all other buckets are empty so we save everything
		tabu.push_back(*it); 
            }	
        };
	neighborhood.clear();
	return false;         //we didnt make a move 
};
	
bool TabuOpt::computeCosts(OptSol &s) {
#ifdef TESTED
std::cout<<"Entering TabuOpt::computeCosts \n";
#endif
        int removedTruck = s.v_computeCosts();
        if (removedTruck==-1) return false;
	removeTruckFromTabuList(removedTruck) ;
	return true;
}







    bool TabuOpt::dumpMoves(std::string str, std::deque<Move> moves) const {
	std::cout<<"Bucket: "<< str<<"\n";
	for (int i=0;i<moves.size();i++)
		moves[i].dump();
    };

