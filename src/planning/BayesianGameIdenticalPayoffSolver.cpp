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

#include "BayesianGameIdenticalPayoffSolver.h"
#include "BayesianGameIdenticalPayoff.h"
#include "JointPolicy.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameWithClusterInfo.h"
#include "EDeadline.h"

using namespace std;
BayesianGameIdenticalPayoffSolver::
BayesianGameIdenticalPayoffSolver(
        const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
        size_t nrDesiredSolutions)
    :
    _m_referredBG(bg),
    _m_solution(bg, nrDesiredSolutions),
    _m_writeAnyTimeResults(false),
    _m_deadlineInSeconds(0)
{
}

void 
BayesianGameIdenticalPayoffSolver::
SetAnyTimeResults(bool turn_on, std::ofstream* results, 
    std::ofstream* timings)
{
    _m_writeAnyTimeResults = turn_on;
    if(turn_on)
    {
        _m_results_f = results;
        _m_timings_f = timings;         
    }
    else
    {
        _m_results_f = NULL;
        _m_timings_f = NULL;
    }
}
    
double 
BayesianGameIdenticalPayoffSolver::
Evaluate(const JointPolicyPureVector & jpolBG) const
{
    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bgip=GetBGIPI();
    const BayesianGameIdenticalPayoffInterface *bgipRawPtr=bgip.get();
    double v = 0.0;
    for(Index jt = 0; jt < bgipRawPtr->GetNrJointTypes(); jt++)
    {
        Index jaI = jpolBG.GetJointActionIndex(jt);  
        double p = bgipRawPtr->GetProbability(jt);
        double u = bgipRawPtr->GetUtility(jt, jaI);
        v += p * u;    
    }
    return v;
}

double 
BayesianGameIdenticalPayoffSolver::
Evaluate(const JointPolicyPureVectorForClusteredBG & jpolBG) const
{
    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bgip=GetBGIPI();
    const BayesianGameIdenticalPayoffInterface *bgipRawPtr=bgip.get();
    double v = 0.0;
    for(Index jt = 0; jt < bgipRawPtr->GetNrJointTypes(); jt++)
    {
        Index jaI = jpolBG.GetJointActionIndex(jt);  
        double p = bgipRawPtr->GetProbability(jt);
        double u = bgipRawPtr->GetUtility(jt, jaI);
        v += p * u;    
    }
    return v;
}

//boost::shared_ptr<JointPolicyPureVector> 
boost::shared_ptr<JointPolicyDiscretePure> 
BayesianGameIdenticalPayoffSolver::
GetNewJpol() const
{
    boost::shared_ptr<JointPolicyPureVector> jpol = boost::shared_ptr<JointPolicyPureVector>(new JointPolicyPureVector( _m_referredBG ) );
    boost::shared_ptr<JointPolicyDiscretePure> jpol2 = boost::static_pointer_cast<JointPolicyDiscretePure>(jpol);
    return(jpol2);
}

void 
BayesianGameIdenticalPayoffSolver::CheckDeadline(const std::string &errorMessage) const
{
    // make sure we don't use too much time
    if(_m_deadlineInSeconds>0)
    {
        tms ts_now;
        times(&ts_now);
        clock_t utime = ts_now.tms_utime -
            _m_timeAtStartOfSolving.tms_utime;
        clock_t stime = ts_now.tms_stime -
            _m_timeAtStartOfSolving.tms_stime;
        double timeSpentInS=static_cast<double>(utime+stime) / sysconf(_SC_CLK_TCK);
        if(timeSpentInS>_m_deadlineInSeconds)
            throw(EDeadline(errorMessage));
    }
}
