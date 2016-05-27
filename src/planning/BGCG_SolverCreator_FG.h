/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
#ifndef _BGCG_SOLVERCREATOR_FG_H_
#define _BGCG_SOLVERCREATOR_FG_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_SolverCreator.h"

//We have to include this (otherwise compiler doesn't know that 
//BGCG_SolverFG is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGCG_SolverFG.h"

/** \brief BGCG_SolverCreator_FG creates BGCG Solvers with
 * Max Plus. */
class BGCG_SolverCreator_FG : public BGCG_SolverCreator
{
private:    
    BG_FactorGraphCreator::BGFactorGraph_t _m_FGt;
    FG_Solver::FG_Solver_t _m_FGSt;
    int _m_verbosity;
    size_t _m_nrSolutions;
    double _m_deadlineInSeconds;
    
    //the below are max-plus specific, should be put in a 
    //parameter structure or something...
    size_t _m_maxiter;
    std::string _m_updateType;
    double _m_damping; 
    size_t _m_nrRestarts;

    ///Do we want to exploit sparseness of joint type space?
    bool _m_exploitSparse;
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGCG_SolverCreator_FG( 
                            BG_FactorGraphCreator::BGFactorGraph_t FGt
                                =BG_FactorGraphCreator::AgentTypeIndependence,
                            FG_Solver::FG_Solver_t FGSt
                                =FG_Solver::FGSt_MaxPlus,
                            size_t nrSolutions=1,
                            int verbose=2,
                            double deadlineInSeconds=0,
                            
                            //the below are max-plus specific, should be put in a 
                            //parameter structure or something...
                            size_t maxiter =25,
                            const std::string & updateT = std::string("PARALL"),
                            double damping=0.0,
                            size_t nrRestarts=10,

                            //this is getting messy, but for now I'm hacking it in - FRANS
                            bool exploitSparseness=false
        )
        :
        _m_FGt(FGt),
        _m_FGSt(FGSt),
        _m_verbosity(verbose),
        _m_nrSolutions(nrSolutions),
        _m_deadlineInSeconds(deadlineInSeconds),

        _m_maxiter(maxiter),
        _m_updateType(updateT),
        _m_damping(damping),
        _m_nrRestarts(nrRestarts),
        _m_exploitSparse(exploitSparseness)
        {}
/*        /// Copy constructor.
          BGCG_SolverCreator_FG(const BGCG_SolverCreator_FG& a);
        /// Destructor.
        ~BGCG_SolverCreator_FG();
        /// Copy assignment operator
        BGCG_SolverCreator_FG& operator= (const BGCG_SolverCreator_FG& o);
*/
    //operators:
    //BGCG_SolverFG* operator()
    BGCG_SolverFG* Create_BGCG_Solver
        (const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bg) const
        {
            //cout << "BGCG_SolverCreator_FG() - _m_verbosity="<<_m_verbosity<<std::endl;
            //this initializes the BGCG_SolverFG and creates the factor graph:
            BGCG_SolverFG* sfg = new BGCG_SolverFG(
                        bg,  
                        _m_FGt,     
                        _m_FGSt,       
                        _m_nrSolutions,
                        _m_verbosity,
                        _m_deadlineInSeconds,
                        _m_exploitSparse
                    );
            //the method-specific parameters need to be set:
            //max-plus options
            sfg->SetMaxIter(_m_maxiter);
            sfg->SetUpdateType(_m_updateType);
            sfg->SetDamping(_m_damping);
            sfg->SetNrRestarts(_m_nrRestarts);
            return(sfg);
        };

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGCG_SolverCreator_FG object for " 
               << BG_FactorGraphCreator::SoftPrint(_m_FGt)
               << " factor graphs and a "
               << FG_Solver::SoftPrint(_m_FGSt)
               << " solver with _m_maxiter="<<_m_maxiter
               << ", _m_updateType=" << _m_updateType
               << ", _m_verbosity="<<_m_verbosity <<", _m_damping="
               << _m_damping << ", _m_nrSolutions="<<_m_nrSolutions
               <<", _m_nrRestarts="<< _m_nrRestarts;
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGCGSC_FG_" << BG_FactorGraphCreator::SoftPrint(_m_FGt)
               << "_" << FG_Solver::SoftPrint(_m_FGSt) 
               << "_nrSol" << _m_nrSolutions;
            if(_m_FGSt==FG_Solver::FGSt_MaxPlus)
            {
               ss << "_maxIter" << _m_maxiter
                  << "_updateType" << _m_updateType
                  << "_damping" <<  _m_damping
                  << "_nrRestarts" << _m_nrRestarts;
            }
            return (ss.str());
        }

    bool IsExactSolver() const { return(false); }

};


#endif /* !_BGCG_SOLVERCREATOR_FG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
