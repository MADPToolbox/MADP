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
#ifndef _BGIP_SOLVERMAXPLUS_H_
#define _BGIP_SOLVERMAXPLUS_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "MaxPlusSolverForBGs.h"
#include "var.h"
#include "maxplus.h"

/** 
 * BGIP_SolverMaxPlus is a class that performs max plus for BGIPs (without agents independence)
 *
 * Note: if there is agent independence, you want to use BGCG_SolverMaxPlus instead!
 *
 *
 */
template<class JP>
class BGIP_SolverMaxPlus : 
    public BayesianGameIdenticalPayoffSolver_T<JP>,
    public MaxPlusSolverForBGs
{
private:        


protected:
    
public:
    // Constructor, destructor and copy assignment.
    // (default) Constructor
    //BGIP_SolverMaxPlus();
    /**Constructor. Directly Associates a problem with the planner
     * Information regarding the problem is used to construct a joint policy
     * of the proper shape.*/
    BGIP_SolverMaxPlus(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
            size_t maxiter = 1000,
            std::string updateType=std::string("PARALL"),
            size_t verbosity = 2,
            double damping = 0.0,
            size_t nrSolutions = 1,
            size_t nrRestarts = 1
        ) :
        BayesianGameIdenticalPayoffSolver_T<JP>(bg),
        MaxPlusSolverForBGs(maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts)
        {}

    ///Solve the BayesianGameIdenticalPayoffInterface
    /**This method returns the expected reward and stores the found policy
     * in _m_solution of this object.
     */
    double Solve()
        {
            boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bgip = 
                BayesianGameIdenticalPayoffSolver_T<JP>::GetBGIPI();

            //an assignment of variables (v3, v8, v14) corresponds to an assignment
            //of joint actions (a1, a2, a3) for a particular joint type.
            //I.e., each (the action taken for eah) (agent,type)-pair is a variable 
            //of the factor graph.
            //let's create these variables
    
            // var_indices[agI][tI] stores the (agent,type)-pair index atI
            std::vector< std::vector<Index> > var_indices;    
            // vars stores the variables. vars[atI]
            std::vector< libDAI::Var > vars;
            bool debug = (_m_verbosity > 7);
            Construct_AgentTypePair_Variables(bgip, var_indices, vars, debug);
            
            //each joint type corresponds to a factor... 
            //let's create them
            std::vector<libDAI::Factor> facs;
            Construct_JointType_Factors(bgip, var_indices, vars, facs, debug);

            //and finally you construct the FactorGraph from that:
            libDAI::FactorGraph fg(facs);

            //size_t  maxiter = 1000;
            //size_t  verb = 2;
            libDAI::Properties props;
            props.Set("maxiter",_m_maxiter);
            //libDAI expects verbosity to be a size_t and will actually crash!
            size_t verb = static_cast<size_t>(_m_verbosity);
            props.Set("verbose", verb);
            props.Set("updates",_m_updateType);
            props.Set("damping",_m_damping);
            double  tol = 1e-4;
            props.Set("tol",tol);
            libDAI::MaxPlus mp (fg, props);
            mp.init();
            if(BayesianGameIdenticalPayoffSolver_T<JP>::GetWriteAnyTimeResults())
                mp.SetAnyTimeResults(true,
                                     BayesianGameIdenticalPayoffSolver_T<JP>::GetResultsOFStream(),
                                     BayesianGameIdenticalPayoffSolver_T<JP>::GetTimingsOFStream());
            double value = mp.run();
            
            //Create the BG policy as computed by MaxPlus...
            
            const std::vector<size_t> & config = mp.GetBestConfiguration();
            // construct the JP with the bgip now
            //JP jpolBG( BayesianGameIdenticalPayoffSolver_T<JP>::_m_solution.GetJointPolicyPureVector() );
            boost::shared_ptr<JP> temp = 
                boost::dynamic_pointer_cast<JP>( BayesianGameIdenticalPayoffSolver_T<JP>::GetNewJpol() );
            JP jpolBG( *temp);
//            delete temp;
            for(Index agI = 0; agI < bgip->GetNrAgents(); agI++)
                for(Index tI = 0; tI < bgip->GetNrTypes(agI); tI++)
                {
                    Index varIndex = var_indices[agI][tI];
                    Index bestAction = config[varIndex];
                    jpolBG.SetAction(agI, tI, bestAction);
                }
            //store the solution
            BayesianGameIdenticalPayoffSolver_T<JP>::AddSolution(jpolBG, value);
            
            return(value);
        }

    bool IsExactSolver() const { return(false); }
        
    void SetCBGupperBound(double upperbound){}// _m_CBGupperbound=upperbound; }
    void SetCBGlowerBound(double lb){}
};


#endif /* !_BGIP_SOLVERMAXPLUS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
