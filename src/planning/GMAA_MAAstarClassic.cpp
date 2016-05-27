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

#include <vector>
#include <float.h>
#include "GMAA_MAAstarClassic.h"
#include "JPPVValuePair.h"
#include "BayesianGameForDecPOMDPStage.h"


using namespace std;

GMAA_MAAstarClassic::GMAA_MAAstarClassic(
    const PlanningUnitMADPDiscreteParameters &params,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p,
    int verboseness
    ) :
//    PlanningUnitDecPOMDPDiscrete(params, horizon, p), //virtual base must be called directly
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(params, horizon, p, verboseness)
{
}

GMAA_MAAstarClassic::GMAA_MAAstarClassic(
    size_t horizon, 
    DecPOMDPDiscreteInterface* p) :
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(horizon, p)
{
}

void GMAA_MAAstarClassic::ResetPlanner()
{
    // nothing to do here
}

bool GMAA_MAAstarClassic::ConstructAndValuateNextPolicies(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi,
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
        bool &cleanUpPPI)
{
    cleanUpPPI=true;
    //return(ConstructAndValuateNextPoliciesExactBG( ppi, poolOfNextPolicies));
    //moved this function here since MAA* is the only GMAA variant that does it
    //in this way...
    PJPPV_sharedPtr jpolPrevTs = 
        boost::dynamic_pointer_cast<PartialJointPolicyPureVector >(ppi->GetJPol());//jpol^ts-1
    size_t depth = jpolPrevTs->GetDepth(); // = depth = ts(jpolPrevTs) + 1
    size_t ts = depth; //jpol = jpol^ts-1, we construct BG for ts == depth
    bool is_last_ts = (ts ==  GetHorizon() - 1);

    // Construct the bayesian game for this timestep - 
    BayesianGameForDecPOMDPStage *bg_ts= new BayesianGameForDecPOMDPStage(
            this,
            _m_qHeuristic,
            jpolPrevTs
            );

    double discountToThePowerT = pow( GetDiscount(), (double)(ts) );
    vector<Index> firstOHtsI(GetNrAgents());
    for(Index agI=0; agI < GetNrAgents(); agI++)
        firstOHtsI.at(agI) = CastLIndexToIndex(GetFirstObservationHistoryIndex(agI, ts));
    const vector<size_t>& nrOHts = bg_ts->GetNrTypes(); 
    LIndex nrJPols = bg_ts->GetNrJointPolicies();
    size_t nrJT = bg_ts->GetNrJointTypes();

    _m_bgCounter++;
    if(_m_bgBaseFilename!="")
    {
        stringstream ss;
        ss << _m_bgBaseFilename << _m_bgCounter;
        BayesianGameIdenticalPayoff::Save(*bg_ts,ss.str());
    }

#if DEBUG_GMAA3
    if(_m_verboseness >= 3) {
        cout << "Constructed BG:"<<endl;
        bg_ts->Print();
    }
#endif


    //the policy for the Bayesian game 
    JointPolicyPureVector jpolBG = JointPolicyPureVector(bg_ts);

    if(_m_verboseness >= 3) 
        cout << "starting on solution of BG for t="<<ts<<" with nrJPols="
            <<nrJPols<<endl;

    stringstream ss;
    ss << "GMAA::NextExact_ts" << ts;
    StartTimer(ss.str());

    //some variables used when this is the last time-step
    double maxLBv = -DBL_MAX;
    JointPolicyPureVector bestLBjpolBG = JointPolicyPureVector(bg_ts);
    double newPastReward = 0.0;
    //we cache the immediate rewards in the BG...
    bg_ts->ComputeAllImmediateRewards();

    /* Do the full enumeration of all joint policies for the BG...*/
    bool carry_over = false;
    LIndex i = 0;
    while(!carry_over)
    {
        if(_m_verboseness >= 0)
            PrintProgress("BG joint policy", i++, nrJPols, 10000);
        //eval the expected future payoff of this jpolBG
        double f = 0.0;        
        double r = 0.0; //immediate reward (exact)
        for(Index jt = 0; jt < nrJT ; jt++)
        {
            Index jaI = jpolBG.GetJointActionIndex(jt);
            double jt_prob = bg_ts->GetProbability(jt);
            double jt_r = bg_ts->GetImmediateReward(jt, jaI);
            r += jt_prob * jt_r;
            double jt_util = bg_ts->GetUtility(jt, jaI);
            f += jt_prob * jt_util;
        }
        //add the expected reward for 0...ts-1
        //NOTE the expected reward has been computed as
        //f = R + gamma * F
        //by the heuristic. That means that f itself is not yet
        //discounted!!:
        double discounted_F = discountToThePowerT * f;
        double v = jpolPrevTs->GetPastReward() + discounted_F;
        // the new past reward:
        double discounted_r = discountToThePowerT * r;

#if DEBUG_GMAA4
        cout <<"v = pastReward_prevTs + g^t * f = "
                << v <<" = "
                << jpolPrevTs->GetPastReward() <<" + "
                << discounted_F << "   "
                << "(g^t * f ="<<discountToThePowerT << " * " << f << ")"
                << endl;
#endif

        newPastReward = jpolPrevTs->GetPastReward() + discounted_r;
        if(!is_last_ts) 
        {   
            //not the last time step...construct and return all 
            //partial DecPOMDP policies,
            
            // construct a policy for the DecPOMDP: a copy of jpolPrevTs
            // extended to this time step (ts) by jpolBG
            PJPDP_sharedPtr jpolTs = 
                ConstructExtendedJointPolicy(*jpolPrevTs,
                    jpolBG, nrOHts, firstOHtsI);
            jpolTs->SetPastReward(newPastReward);

            // wrap the policy and put it in the pool of next policies.
            // (these next policies are candidates to be added to the 
            // main policy pool)
            // Warning: jpolTs may have been deleted afterwards
            poolOfNextPolicies->Insert( NewPPI(jpolTs, v) );

            // conserve memory 
            // is this necessary?  - this is performed by 
            //      'SelectPoliciesToProcessFurther' 
            // seems like duplicate work...
            //if(poolOfNextPolicies->Size()>(_m_nrPoliciesToProcess*10))
                //Prune(poolOfNextPolicies,_m_nrPoliciesToProcess);
        }
        else
        {   
            //if using a heuristic that does not specify the exact immediate
            //reward, we still want to return the *EXACT* payoff here...
            v = newPastReward;
            //this is the last time step: 
            //  -values are lowerbounds, so if we find a policy with total 
            //      reward equal to the upperbound of the parent (ppi->
            //      GetValue() ) then we can stop evaluating other policies.
            //  -we will only need to return 1 policy and value
            //   (so we do not put them in a pool)
            if(v > maxLBv)
            {
                bestLBjpolBG = jpolBG;
                maxLBv = v;
            }
            if(v >= ppi->GetValue() - 1e-8)
            {
                if(_m_verboseness >= 0) 
                    cout << "GMAA_MAAstarClassic::ConstructAndValuateNextPolicies"<<
                        " Last time step, found tight bound: "<<
                        "found value v="<<v<<", parent (upperbound) value="<<
                        ppi->GetValue() << endl;
                break;
            }
        }
        carry_over = ++jpolBG; //next policy for the BG...
        _m_nrJPolBGsEvaluated++;
    }
    //empty the imm reward cache
    bg_ts->ClearAllImmediateRewards();

    //if this is the last time step, we have not constructed the Dec-POMDP
    //policy which will be returned yet, so we do this now:
    if(is_last_ts) 
    {
        PJPDP_sharedPtr jpolTs = ConstructExtendedJointPolicy(
                *jpolPrevTs,
                bestLBjpolBG, 
                nrOHts, 
                firstOHtsI
                );
        jpolTs->SetPastReward(newPastReward);
        // Warning: jpolTs may have been deleted afterwards
        poolOfNextPolicies->Insert( NewPPI(jpolTs, maxLBv ) );
    }

    if(_m_verboseness >= 2) 
        cout <<"Solved BG for t="<<ts<<". Max. expected value (including "
             <<"true expected reward up to this ts:"<<jpolPrevTs->GetPastReward()<<") is:"
             <<poolOfNextPolicies->Select()->GetValue() <<endl;

    delete(bg_ts);
    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);

}

void GMAA_MAAstarClassic::SelectPoliciesToProcessFurther(
    const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies, bool are_LBs, double bestLB)
{
    //for MAA*, all policies are processed further when they aren't full
    //policies (i.e. lower bounds)
    //so, unless these are full policies...

    if(are_LBs)
    {
        while(!poolOfNextPolicies->Empty())
            poolOfNextPolicies->Pop();
    }
    //we can return immediately...
    return;
}

PartialPolicyPoolItemInterface_sharedPtr
GMAA_MAAstarClassic::NewPPI(const PJPDP_sharedPtr &jp, double v) const
{
    //NOTE: this conversion to an index does save a lot of space, so
    //if we can enable this again would be better.
    //PartialPolicyPoolItemInterface* ppi=new JPPVIndexValPair(jp,v);
    //delete jp;
    PartialPolicyPoolItemInterface_sharedPtr ppi=
        PartialPolicyPoolItemInterface_sharedPtr(new PartialJPDPValuePair(jp,v));
    return (ppi);
}
