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
