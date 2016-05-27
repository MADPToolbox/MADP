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

#include "BGIP_SolverRandom.h"
#include "TimeTools.h"
#include <float.h>

using namespace std;

//Default constructor
BGIP_SolverRandom::BGIP_SolverRandom(
    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bg,
    int verbose,
    size_t nrSolutions) :
    BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector>(bg, nrSolutions),
    _m_verbose(verbose)
{
}

double BGIP_SolverRandom::Solve()
{    
    for(Index i=0;i!=GetNrDesiredSolutions();++i)
    {
        JPPV_sharedPtr jpol=
            JPPV_sharedPtr (new JointPolicyPureVector(GetBGIPI()));
        jpol->RandomInitialization();

        double v = 0.0;
        
        for(Index jt = 0; jt < GetBGIPI()->GetNrJointTypes(); jt++)
        {
            double P_jt = GetBGIPI()->GetProbability(jt);
            Index ja = jpol->GetJointActionIndex(jt);  
            v += P_jt * GetBGIPI()->GetUtility(jt, ja);
        }
    
        AddSolution( *jpol, v);
    }

    return(BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector>::GetPayoff());
}
