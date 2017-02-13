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

#include "JESPExhaustivePlanner.h"
#include <float.h>

using namespace std;

#define DEBUG_EXHBR 0
#define DEBUG_EXJESP 0

JESPExhaustivePlanner::JESPExhaustivePlanner(
    size_t horizon,
    DecPOMDPDiscreteInterface* p,
    const PlanningUnitMADPDiscreteParameters * params
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon, p, params),
    _m_foundPolicy()
    //,_m_exhBRBestPol(*this)
{
}

void JESPExhaustivePlanner::Plan()
{
    if(DEBUG_EXJESP){ 
        cout << "\n---------------------------------"<<endl;
        cout << "Exhaustive JESP  - Plan() started"<<endl;
        cout << "---------------------------------"<<endl;
    }
    double v_best = -DBL_MAX;
    JointPolicyPureVector *jpol = new JointPolicyPureVector(this);
    JointPolicyPureVector *best = new JointPolicyPureVector(this);
    jpol->RandomInitialization();
    
    if(DEBUG_EXJESP) {cout << "joint policy randomly initialized to:";
        jpol->Print();}
    
    int stop = 0;
    size_t nr_non_improving_agents = 0;
    while(nr_non_improving_agents < GetDPOMDPD()->GetNrAgents()
            && stop++ < 1000) 
    {
        int agentI = GetNextAgentIndex();
        double v = ExhaustiveBestResponse(jpol, agentI);
        if(v > v_best + 1e-9)
        {  
            (*best) = (*jpol);
            if(DEBUG_EXHBR)
                {cout << "Plan: new best policy:"<<endl; best->Print();}
            v_best = v;
            nr_non_improving_agents = 0;
        }        
        else
            nr_non_improving_agents++;
    }
    _m_foundPolicy = JPPV_sharedPtr(best);
    _m_expectedRewardFoundPolicy=v_best;
    

    if(DEBUG_EXJESP){ 
        cout << "Exhaustive JESP  - resulting policy:"<<endl;
        cout << "------------------------------------"<<endl;
        best->Print();
    }
}

double JESPExhaustivePlanner::ExhaustiveBestResponse(JointPolicyPureVector* 
    jpol, int agentI)
{
    if(DEBUG_EXHBR)
        cout << "JESPExhaustivePlanner::ExhaustiveBestResponse called "
             << "for agent " << agentI << endl;

    bool round = false;
    JointPolicyPureVector best(this);
    double v_best = -DBL_MAX;
    double v = 0.0;
    jpol->ZeroInitialization(agentI);
    
    while(!round)
    {
        ValueFunctionDecPOMDPDiscrete vf(this, jpol);
        v = vf.CalculateV<true>();
        if(v > v_best)
        {
            best = (*jpol);
            if(DEBUG_EXHBR)
                {cout << "ExhaustiveBestResponse: new best policy:"<<endl; 
                best.Print();}
            v_best = v;
        }
        round = jpol->Increment(agentI);
    }
    //_m_exhBRBestPol = best;
    *jpol = best;
    if(DEBUG_EXJESP){   cout << "Best response V="<<v_best<<endl;}
    if(DEBUG_EXHBR){ cout << "policy="; jpol->Print();}
    return(v_best);
}
