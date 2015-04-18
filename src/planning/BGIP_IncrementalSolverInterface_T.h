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
#ifndef _BGIP_INCREMENTALSOLVERINTERFACE_T_H_
#define _BGIP_INCREMENTALSOLVERINTERFACE_T_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "BGIP_IncrementalSolverInterface.h"

/**\brief BGIP_IncrementalSolverInterface_T is an interface for
 * BGIP_Solvers that can incrementally return multiple solutions.
 */
template<class JP>
class BGIP_IncrementalSolverInterface_T : 
    public BGIP_IncrementalSolverInterface
    //,public BayesianGameIdenticalPayoffSolver_T<JP> //<- this leads to cyclic inheritence, 
{    
public:
    BGIP_IncrementalSolverInterface_T(
        const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
        size_t nrDesiredSolutions=INT_MAX
        ) 
    :
        BGIP_IncrementalSolverInterface(bg,nrDesiredSolutions)
        //BayesianGameIdenticalPayoffSolver_T<JP>(bg,nrDesiredSolutions),
        //_m_nrSolutionsReturned(0)
        {}

    
    //virtual boost::shared_ptr<JointPolicyDiscretePure> GetNewJpol() const
    //{
        //return this->BayesianGameIdenticalPayoffSolver_T<JP>:: GetNewJpol();

    //}
    
    /// this gives a implementation of GetNewJpol (specified in BayesianGameIdenticalPayoffSolver)
    virtual boost::shared_ptr<JointPolicyDiscretePure> GetNewJpol() const
    //boost::shared_ptr<JP> GetNewJpol() const
    {
        boost::shared_ptr<JP> jpol = boost::shared_ptr<JP>(new JP(this->GetBGIPI()));
        // check whether it is a
        // JointPolicyPureVectorForClusteredBG, because then we
        // need to store more things
        boost::shared_ptr<JointPolicyPureVectorForClusteredBG> JPPVfCBG=
            boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(jpol);
        if(JPPVfCBG)
        {
            boost::shared_ptr<const JointPolicyDiscretePure> pastJpol =
                JPPVfCBG->GetBG()->GetPastJointPolicyPVFCBG();
            JPPVfCBG->SetPrevJPPVfCBG(
                boost::dynamic_pointer_cast<const JointPolicyPureVectorForClusteredBG>(pastJpol));
            JPPVfCBG->SetDepth(JPPVfCBG->GetBG()->GetStage());
        }
        return(jpol);
    }

    
};


#endif /* !_BGIP_INCREMENTALSOLVERINTERFACE_T_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
