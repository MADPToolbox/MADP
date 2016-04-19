/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
