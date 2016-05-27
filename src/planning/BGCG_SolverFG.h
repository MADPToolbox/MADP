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
#ifndef _BGCG_SOLVERFG_H_
#define _BGCG_SOLVERFG_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_Solver.h"
#include "BG_FactorGraphCreator.h"
#include "FG_Solver.h"

/**\brief BGCG_SolverFG solves collaborative graphical Bayesian games
 * Bayesian game by converting to a Factor Graph and then applying
 * a optimization method to this FG.
 *
 **/

namespace{
    typedef std::pair< double, std::vector<size_t> > valConf;
}
namespace libDAI{
    class MaxPlus;
    
}

class BGCG_SolverFG :
    public BGCG_Solver
{
    public:
        typedef libDAI::MADP_util::valConf valConf;
    private:

        BG_FactorGraphCreator::BGFactorGraph_t _m_FGt;
        FG_Solver::FG_Solver_t _m_FGSt;

        /// the verbosity level
        int _m_verbosity;

        /// the factor graph creator
        BG_FactorGraphCreator* _m_fgc;

        double _m_deadlineInSeconds;

        // max-plus parameters - these are not initialized (to a sane value) by 
        // constructor.
        std::string _m_updateType;
        double _m_damping;
        size_t _m_nrRestarts;
        size_t _m_maxIter; 

        double ProcessFoundConfigurations_ATI(std::list< valConf >& bestConfs);
        double ProcessFoundConfigurations_AI(std::list< valConf >& bestConfs);
    
    protected:
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        BGCG_SolverFG(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,  
            BG_FactorGraphCreator::BGFactorGraph_t FGt
                =BG_FactorGraphCreator::AgentTypeIndependence, 
            FG_Solver::FG_Solver_t FGSt
                =FG_Solver::FGSt_MaxPlus,
            size_t nrSolutions=1,
            int verbosity = 2,
            double deadlineInSeconds = 0,
            bool exploitSparseness=false
        );
        /// Copy constructor.
        BGCG_SolverFG(const BGCG_SolverFG& a);
        /// Destructor.
        ~BGCG_SolverFG();
        /// Copy assignment operator
        BGCG_SolverFG& operator= (const BGCG_SolverFG& o);

        ///max-plus parameter setting
        void SetUpdateType(std::string s) {_m_updateType = s;}
        void SetDamping(double d){_m_damping = d;}
        void SetNrRestarts(size_t n){_m_nrRestarts = n;}
        void SetMaxIter(size_t m){_m_maxIter = m;}
        ///Go! (solve the BayesianGameCollaborativeGraphical)
        double Solve();
        
        //TODO this should depend on the solver type (but all that is not defined yet...)
        bool IsExactSolver() const { return(false); }
        

        void SetCBGupperBound(double ub){}
        void SetCBGlowerBound(double lb){}
        //get (data) functions:
};


#endif /* !_BGCG_SOLVERFG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
