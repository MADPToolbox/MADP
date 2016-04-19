/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
