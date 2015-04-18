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
#ifndef _BGIP_SOLVERCREATOR_BNB_H_
#define _BGIP_SOLVERCREATOR_BNB_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_IncrementalSolverCreatorInterface_T.h"

//We have to include this (otherwise compiler doesn't know that 
//BGIP_SolverBruteForceSearch is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGIP_SolverBranchAndBound.h"
//class BGIP_SolverBruteForceSearch;

/** \brief BGIP_SolverCreator_BnB creates BGIP Solvers with
 * Branch-and-Bound search.  */
template<class JP>
class BGIP_SolverCreator_BnB : public BGIP_IncrementalSolverCreatorInterface_T<JP>
{
private:    
    int _m_verbose;
    size_t _m_nrSolutions;
    bool _m_keepAll;
    BGIP_BnB::BnB_JointTypeOrdering _m_jtOrdering;
    bool _m_reComputeH;
    double _m_deadlineInSeconds;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverCreator_BnB(int verbose=0,
                           size_t nrSolutions=1,
                           bool keepAll=false,
                           BGIP_BnB::BnB_JointTypeOrdering ordering=BGIP_BnB::IdentityMapping,
                           bool reComputeHeur=false,
                           double deadlineInSeconds=0)
        :
        _m_verbose(verbose),
        _m_nrSolutions(nrSolutions),
        _m_keepAll(keepAll),
        _m_jtOrdering(ordering),
        _m_reComputeH(reComputeHeur),
        _m_deadlineInSeconds(deadlineInSeconds)
        {}
    //operators:
    BGIP_SolverBranchAndBound<JP>* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            BGIP_SolverBranchAndBound<JP>* bgsolver=
                new BGIP_SolverBranchAndBound<JP>(
                    bg,
                    _m_verbose,
                    _m_nrSolutions,
                    _m_keepAll,
                    _m_jtOrdering,
                    _m_reComputeH
                    );
            bgsolver->SetDeadline(_m_deadlineInSeconds);
            return( bgsolver );
        };

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGIP_SolverCreator_BnB object with "<<
                ", _m_verbose="<<_m_verbose <<
                ", _m_nrSolutions="<<_m_nrSolutions<<
                std::endl;
            return (ss.str());
        }
    
    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGIPSC_BnB_nrSol" << _m_nrSolutions
               << "_jtOrder" << BGIP_BnB::SoftPrint(_m_jtOrdering)
               << "_CCIheur" << _m_reComputeH;
            return(ss.str());
        }


    bool IsExactSolver() const { return(true); }

};


#endif /* !_BGIP_SOLVERCREATOR_BNB_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
