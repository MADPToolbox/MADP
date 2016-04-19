/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGIP_SOLVERCREATORINTERFACE_T_H_
#define _BGIP_SOLVERCREATORINTERFACE_T_H_ 1

/* the include directives */
#include "Globals.h"

class BayesianGameIdenticalPayoffInterface;
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "BGIP_SolverCreatorInterface.h"

/** \brief BGIP_SolverCreatorInterface_T is an interface for classes that create
 * BGIP solvers.
 *
 * The template argument JP represents the joint policy class the
 * solver should return.
 */
template<class JP>
class BGIP_SolverCreatorInterface_T 
    : public BGIP_SolverCreatorInterface
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    
    virtual ~BGIP_SolverCreatorInterface_T(){};

    //operators:

    /// Returns a pointer to a new BGIP solver object.
    virtual BayesianGameIdenticalPayoffSolver_T<JP>* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const = 0;

    /// Returns a description of the solver creator.
    virtual std::string SoftPrint() const = 0;

    /// Returns a brief description of the solver creator.
    virtual std::string SoftPrintBrief() const = 0;

    /**\brief Methods should indicated whether they compute exact
    * (optimal) solutions or not. */
    virtual bool IsExactSolver() const = 0;

};


#endif /* !_BGIP_SOLVERCREATORINTERFACE_T_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
