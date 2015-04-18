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
#ifndef _BGIP_SOLVERCREATOR_CE_H_
#define _BGIP_SOLVERCREATOR_CE_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_SolverCreatorInterface_T.h"

//We have to include this (otherwise compiler doesn't know that 
//BGIP_SolverCE is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGIP_SolverCE.h"

/** \brief BGIP_SolverCreator_CE creates BGIP Solvers with Cross
 * Entropy. */
//template<class JP>
class BGIP_SolverCreator_CE : public BGIP_SolverCreatorInterface_T<JointPolicyPureVector>
{
private:
    size_t _m_nrRestarts;
    size_t _m_nrIterations;
    size_t _m_nrSampledJointPolicies;
    size_t _m_nrJointPoliciesForUpdate;
    bool _m_use_gamma;
    double _m_alpha;
    double _m_deadlineInSeconds;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverCreator_CE(
        size_t nrCERestarts = 10,
        size_t nrIterations = 30,
        size_t nrSamples = 40,
        size_t nrSamplesForUpdate =15,
        bool use_hard_threshold = true, //(gamma in CE papers)
        double CEalpha = 0.3, //the learning rate
        double deadlineInSeconds = 0
        ):
        _m_nrRestarts(nrCERestarts),
        _m_nrIterations(nrIterations),
        _m_nrSampledJointPolicies(nrSamples),
        _m_nrJointPoliciesForUpdate(nrSamplesForUpdate),
        _m_use_gamma(use_hard_threshold),
        _m_alpha(CEalpha),
        _m_deadlineInSeconds(deadlineInSeconds)
        {}
        virtual ~BGIP_SolverCreator_CE(){};
/*        /// Copy constructor.
        BGIP_SolverCreator_CE(const BGIP_SolverCreator_CE& a);
        /// Destructor.
        /// Copy assignment operator
        BGIP_SolverCreator_CE& operator= (const BGIP_SolverCreator_CE& o);
*/
    //operators:
    BGIP_SolverCE* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            BGIP_SolverCE* bgsolver=
                new BGIP_SolverCE(
                    bg,
                    _m_nrRestarts,
                    _m_nrIterations,
                    _m_nrSampledJointPolicies,
                    _m_nrJointPoliciesForUpdate,
                    _m_use_gamma,
                    _m_alpha
                    );
            bgsolver->SetDeadline(_m_deadlineInSeconds);
            return(bgsolver);
        }

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGIP_SolverCreator_CE object with ";
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGIPSC_CE"
               << "_CEr" << _m_nrRestarts 
               << "_i" << _m_nrIterations
               << "_s" << _m_nrSampledJointPolicies
               << "_sfu" << _m_nrJointPoliciesForUpdate
               << "_a" << _m_alpha 
               << "_ht" << _m_use_gamma;
            return(ss.str());
        }

    bool IsExactSolver() const { return(false); }

};


#endif /* !_BGIP_SOLVERCREATOR_CE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
