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
#ifndef _BGCG_SOLVERMAXPLUS_H_
#define _BGCG_SOLVERMAXPLUS_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_Solver.h"
#include "MaxPlusSolverForBGs.h"

/** \brief BGCG_SolverMaxPlus is a class  that performs max plus for BGIPs with agents independence.
 *
 * This can also be used to solve BGIPs without agent independence by using the BGCGWrapperForBGIP 
 * wrapper. Alternatively, one may want to use BGIP_SolverMaxPlus
 * 
 * */
class BGCG_SolverMaxPlus :
    public BGCG_Solver,
    public MaxPlusSolverForBGs
{
    private:    
    
    protected:
        static void Construct_JointType_Factors_CGBG(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgip,
            const vector< vector<Index> >& var_indices,
            const vector< libDAI::Var >& vars,
            vector<libDAI::Factor>& facs,
            int  verbosity);

    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        BGCG_SolverMaxPlus(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,                
            size_t maxiter = 1000,
            std::string updateType=std::string("PARALL"),
            int  verbosity = 2,
            double damping = 0.0,
            size_t nrSolutions = 1,
            size_t nrRestarts = 1);
/*        
        /// Copy constructor.
        BGCG_SolverMaxPlus(const BGCG_SolverMaxPlus& a);
        /// Destructor.
        ~BGCG_SolverMaxPlus();
        /// Copy assignment operator
        BGCG_SolverMaxPlus& operator= (const BGCG_SolverMaxPlus& o);
*/
        virtual double Solve();

        bool IsExactSolver() const { return(false); }

        void SetCBGupperBound(double ub){}
        void SetCBGlowerBound(double lb){}

};


#endif /* !_BGCG_SOLVERMAXPLUS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
