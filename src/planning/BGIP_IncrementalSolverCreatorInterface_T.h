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
#ifndef _BGIP_INCREMENTALSOLVERCREATORINTERFACE_T_H_
#define _BGIP_INCREMENTALSOLVERCREATORINTERFACE_T_H_ 1

/* the include directives */
#include "Globals.h"

class BayesianGameIdenticalPayoffInterface;
//#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "BGIP_IncrementalSolverInterface_T.h"
#include "BGIP_SolverCreatorInterface_T.h"

/** \brief BGIP_IncrementalSolverCreatorInterface_T is an interface for classes that create
 * BGIP solvers.
 *
 * The template argument JP represents the joint policy class the
 * solver should return.
 */
template<class JP>
class BGIP_IncrementalSolverCreatorInterface_T 
    : public BGIP_SolverCreatorInterface
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    
    virtual ~BGIP_IncrementalSolverCreatorInterface_T(){};

    //operators:

    /// Returns a pointer to a new BGIP solver object.
    virtual  BGIP_IncrementalSolverInterface_T<JP>* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const = 0;

    /// Returns a description of the solver creator.
    virtual std::string SoftPrint() const = 0;

    /// Returns a brief description of the solver creator.
    virtual std::string SoftPrintBrief() const = 0;

    /**\brief Methods should indicated whether they compute exact
    * (optimal) solutions or not. */
    virtual bool IsExactSolver() const = 0;

};


#endif /* !_BGIP_INCREMENTALSOLVERCREATORINTERFACE_T_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
