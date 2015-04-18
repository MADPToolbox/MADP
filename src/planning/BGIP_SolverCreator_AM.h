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
#ifndef _BGIP_SOLVERCREATOR_AM_H_
#define _BGIP_SOLVERCREATOR_AM_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_IncrementalSolverCreatorInterface_T.h"

//We have to include this (otherwise compiler doesn't know that 
//BGIP_SolverAlternatingMaximization is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGIP_SolverAlternatingMaximization.h"
//class BGIP_SolverAlternatingMaximization;

/** \brief BGIP_SolverCreator_AM creates BGIP Solvers with Alternating
 * Maximization. */
template<class JP>
class BGIP_SolverCreator_AM : public BGIP_IncrementalSolverCreatorInterface_T<JP>
{
private:    
    size_t _m_nrRestarts;
    int _m_verbose;
    size_t _m_nrSolutions;
    double _m_deadlineInSeconds;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverCreator_AM(size_t nrRestarts=10, 
                          int verbose=0,
                          size_t nrSolutions=1,
                          double deadlineInSeconds=0)
        :
        _m_nrRestarts(nrRestarts),
        _m_verbose(verbose),
        _m_nrSolutions(nrSolutions),
        _m_deadlineInSeconds(deadlineInSeconds)
        {}
        virtual ~BGIP_SolverCreator_AM(){};
/*        /// Copy constructor.
        BGIP_SolverCreator_AM(const BGIP_SolverCreator_AM& a);
        /// Destructor.
        /// Copy assignment operator
        BGIP_SolverCreator_AM& operator= (const BGIP_SolverCreator_AM& o);
*/
    //operators:
    BGIP_SolverAlternatingMaximization<JP>* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            if(_m_verbose >= 2)
            {
                std::cout << "BGIP_SolverCreator_AM:: creating a new BGIP_SolverAlternatingMaximization with nrRestarts=" << _m_nrRestarts<< ", verbose=" << _m_verbose << ", nrSols="<<_m_nrSolutions << std::endl;
            }
            BGIP_SolverAlternatingMaximization<JP>* bgsolver=
                new BGIP_SolverAlternatingMaximization<JP>(
                    bg,
                    _m_nrRestarts,
                    _m_verbose,
                    _m_nrSolutions);
            bgsolver->SetDeadline(_m_deadlineInSeconds);
            return(bgsolver);
        }

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGIP_SolverCreator_AM object with "<<
                ", _m_verbose="<<_m_verbose <<
                ", _m_nrSolutions="<<_m_nrSolutions<<
                ", _m_nrRestarts="<<_m_nrRestarts;
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGIPSC_AltMax_nrSol" << _m_nrSolutions
               << "_nrRestarts" << _m_nrRestarts;
            return(ss.str());
        }


    bool IsExactSolver() const { return(false); }

};


#endif /* !_BGIP_SOLVERCREATOR_AM_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
