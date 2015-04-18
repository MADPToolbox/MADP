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

#include "BruteForceSearchPlanner.h"
#include <float.h>

using namespace std;

#define DEBUG_BFS 0

//Default constructor
BruteForceSearchPlanner::BruteForceSearchPlanner(size_t horizon, 
        DecPOMDPDiscreteInterface* p) :
    PlanningUnitDecPOMDPDiscrete(horizon, p),
    _m_foundPolicy()
{
}

BruteForceSearchPlanner::BruteForceSearchPlanner(
    const PlanningUnitMADPDiscreteParameters &params,
    size_t horizon, DecPOMDPDiscreteInterface* p) :
    PlanningUnitDecPOMDPDiscrete(params, horizon, p),
    _m_foundPolicy()
{
}

void BruteForceSearchPlanner::Plan()
{    
    bool round = false;
    JointPolicyPureVector* jpol = new JointPolicyPureVector(this);
    JointPolicyPureVector* best = new JointPolicyPureVector(this);
    double v_best = -DBL_MAX;
    double v = 0.0;
    
    int i = 0;
    if(DEBUG_BFS)
        cout << "Starting Bruteforce search - v_best is init to "
             << v_best << endl;
    LIndex nrJPols = GetNrJointPolicies();
    
    while(!round)
    {
        if(DEBUG_BFS){    cout << "Jpol#"<< i << " - ";}
        PrintProgress("Jpol #",i,nrJPols,1000);
        i++;

        ValueFunctionDecPOMDPDiscrete vf(this, jpol);
        v = vf.CalculateV(true);//set caching to true!
        if(DEBUG_BFS)    cout << "Expected value = "<< v;
        if(v > v_best)
        {
            if(DEBUG_BFS) cout << " -> new best policy!!!";
            v_best = v;
            best = jpol;
        }
        if(DEBUG_BFS) cout << endl << "Incrementing joint policy..." <<endl; 
        round = ++(*jpol);
    }

    _m_foundPolicy=JPPV_sharedPtr(best);
    _m_expectedRewardFoundPolicy=v_best;
}
