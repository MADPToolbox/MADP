/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGIP_SOLVERRANDOM_H_
#define _BGIP_SOLVERRANDOM_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"



/** \brief BGIP_SolverRandom creates random solutions to Bayesian
 * games for testing purposes
 */
class BGIP_SolverRandom : public BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector>
{
private:    

    int _m_verbose;

protected:
    
public:
    // Constructor, destructor and copy assignment.

    /**Constructor. Directly Associates a problem with the planner
     * Information regarding the problem is used to construct a joint policy
     * of the proper shape.*/
    BGIP_SolverRandom(boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bg,
                      int verbose=0,
                      size_t nrSolutions=1);

    double Solve();

    bool IsExactSolver() const { return(false); }

    void SetCBGupperBound(double upperbound){}// _m_CBGupperbound=upperbound; }
    void SetCBGlowerBound(double lb){}
        
};


#endif /* !_BGIP_SOLVERRANDOM_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
