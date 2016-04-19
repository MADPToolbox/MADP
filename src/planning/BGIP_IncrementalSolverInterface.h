/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGIP_INCREMENTALSOLVERINTERFACE_H_
#define _BGIP_INCREMENTALSOLVERINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BayesianGameIdenticalPayoffSolver.h"

/**\brief BGIP_IncrementalSolverInterface is an interface for
 * BGIP_Solvers that can incrementally return multiple solutions.
 */
class BGIP_IncrementalSolverInterface : public BayesianGameIdenticalPayoffSolver
{
private:    
protected:

    /// The number of solutions we already *returned* (not computed)
    size_t _m_nrSolutionsReturned;
    
    ///The specific implementation that should be implemented by the derived class:
    virtual bool GetNextJointPolicyAndValueSpecific(boost::shared_ptr<JointPolicyDiscretePure> &jpol, double &value) = 0;
public:

    BGIP_IncrementalSolverInterface(
        const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
        size_t nrDesiredSolutions=INT_MAX) 
        :
        BayesianGameIdenticalPayoffSolver(bg,nrDesiredSolutions),
        _m_nrSolutionsReturned(0)
        {}


    virtual ~BGIP_IncrementalSolverInterface(){};

    /// Gets the next solution from the Incremental CBG solver.
    /** If a next solution does not exist, for instance because of
     * upper and lower bounds on the solution value, the function will
     * return false instead of true.
     */
    bool GetNextJointPolicyAndValue(boost::shared_ptr<JointPolicyDiscretePure> &jpol, double &value)
    {
        _m_nrSolutionsReturned++;
        return GetNextJointPolicyAndValueSpecific( jpol, value );
    }
 
    bool AllSolutionsHaveBeenReturned() const
    {
        if(this->GetNrDesiredSolutions() != INT_MAX)
            return (this->GetNrDesiredSolutions() <= _m_nrSolutionsReturned);

        if(_m_nrSolutionsReturned >=
            BayesianGameIdenticalPayoffSolver::GetBGIPI()->GetNrJointPolicies()) 
            return(true);
        else
            return(false);
    };

};


#endif /* !_BGIP_INCREMENTALSOLVERINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
