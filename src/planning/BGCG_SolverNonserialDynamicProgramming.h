/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
