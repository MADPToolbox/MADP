/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _BGCG_SOLVERNONSERIALDYNAMICPROGRAMMING_H_
#define _BGCG_SOLVERNONSERIALDYNAMICPROGRAMMING_H_ 1

/* the include directives */
#include <iostream>
#include <set>
#include "Globals.h"
#include "BGCG_Solver.h"
#include "Scope.h"

class LocalBGValueFunctionInterface;


/**BGCG_SolverNonserialDynamicProgramming implements non-serial dynamic programming for
 * collaborative graphical Bayesian games.  */
class BGCG_SolverNonserialDynamicProgramming :
    public BGCG_Solver
{
private:    
    
    /// The set of neighbors for each agent.
    std::vector< std::set<Index> > _m_neighbors;
    
    /// Maintains the local BGvalue functions in which each agent participates.
    std::vector< std::set<LocalBGValueFunctionInterface*> > _m_agentFunctions;

    //the vector that maintaint the f function (Vlassis07)
    //or V(jpolBG_G) functions sec. 4.3.
    std::set<LocalBGValueFunctionInterface*> _m_local_Fs;

    std::vector< std::vector<Index> > _m_bestResponses;
    Scopes _m_bestResponseScopes;

    /// Returns an order in which to eliminate agents.
    std::vector<Index> ComputeHeuristicAgentOrder() const;

    void CreateInitialLocalBGFunctionsFromBGCG();
    
    void InitializeNeighborsAndAgentFuncs();
    
    void EliminateAgent(Index agToElim);

    LIndex GetJpolIndexForBestResponses(const std::vector<Index> &agentOrdering)
        const;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGCG_SolverNonserialDynamicProgramming(
        const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg);

    //operators:
    
    //data manipulation (set) functions:
    double Solve();

    // this is an exact solver
    bool IsExactSolver() const { return(true); }

    void SetCBGupperBound(double ub){}
    void SetCBGlowerBound(double lb){}
        
    //get (data) functions:
};


#endif /* !_BGCG_SOLVERNONSERIALDYNAMICPROGRAMMING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
