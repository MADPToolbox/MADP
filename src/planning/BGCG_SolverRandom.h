/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGCG_SOLVERRANDOM_H_
#define _BGCG_SOLVERRANDOM_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_Solver.h"

/** \brief BGCG_SolverRandom is a class that represents a BGCG solver that
 * returns a random joint BG policy.
 * */
class BGCG_SolverRandom : public BGCG_Solver
{
private:
    
protected:

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGCG_SolverRandom(
        const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,
        size_t nrSolutions = 1);
    
    virtual double Solve();

    bool IsExactSolver() const { return(false); }

    void SetCBGupperBound(double ub){}
    void SetCBGlowerBound(double lb){}

};


#endif /* !_BGCG_SOLVERRANDOM_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
