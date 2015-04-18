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
#ifndef _BGCG_SOLVERCREATOR_MP_H_
#define _BGCG_SOLVERCREATOR_MP_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_SolverCreator.h"

//We have to include this (otherwise compiler doesn't know that 
//BGCG_SolverMaxPlus is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGCG_SolverMaxPlus.h"

/** \brief BGCG_SolverCreator_MP creates BGCG Solvers with
 * Max Plus. */
class BGCG_SolverCreator_MP : public BGCG_SolverCreator
{
private:    
    size_t _m_maxiter;
    std::string _m_updateType;
    size_t _m_verbose;
    double _m_damping; 
    size_t _m_nrSolutions;
    size_t _m_nrRestarts;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGCG_SolverCreator_MP( size_t maxiter =25,
                           const std::string & updateT = std::string("PARALL"),
                           size_t verbose=0,
                           double damping=0.0,
                           size_t nrSolutions=1,
                           size_t nrRestarts=10 
        )
        :
        _m_maxiter(maxiter),
        _m_updateType(updateT),
        _m_verbose(verbose),
        _m_damping(damping),
        _m_nrSolutions(nrSolutions),
        _m_nrRestarts(nrRestarts)        
        {}
/*        /// Copy constructor.
          BGCG_SolverCreator_MP(const BGCG_SolverCreator_MP& a);
        /// Destructor.
        ~BGCG_SolverCreator_MP();
        /// Copy assignment operator
        BGCG_SolverCreator_MP& operator= (const BGCG_SolverCreator_MP& o);
*/
    //operators:
    //BGCG_SolverMaxPlus* operator()
    BGCG_SolverMaxPlus* Create_BGCG_Solver(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bg) const
        {
            return( 
                new BGCG_SolverMaxPlus(
                    bg,
                    _m_maxiter,
                    _m_updateType,
                    _m_verbose,
                    _m_damping,
                    _m_nrSolutions,
                    _m_nrRestarts
                    )
                );
        };

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGCG_SolverCreator_MP object with _m_maxiter="<<_m_maxiter <<
                ", _m_updateType=" << _m_updateType <<
                ", _m_verbose="<<_m_verbose <<", _m_damping="<< _m_damping <<
                ", _m_nrSolutions="<<_m_nrSolutions<<", _m_nrRestarts="<<
                _m_nrRestarts;
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGCGSC_MaxPlus_nrSol" << _m_nrSolutions
               << "_maxIter" << _m_maxiter
               << "_updateType" << _m_updateType
               << "_damping" <<  _m_damping
               << "_nrRestarts" << _m_nrRestarts;
            return (ss.str());
        }

    bool IsExactSolver() const { return(false); }

};


#endif /* !_BGCG_SOLVERCREATOR_MP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
