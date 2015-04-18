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

#include <float.h>

#include "GMAA_MAA_ELSI.h"
#include "BayesianGameCollaborativeGraphical.h"
#include "BayesianGameIdenticalPayoff.h"

#include "BGCG_SolverNonserialDynamicProgramming.h"
#include "JointBeliefInterface.h"
#include "PartialJPPVIndexValuePair.h"
#include "PartialPolicyPoolInterface.h"
#include "PartialPolicyPoolItemInterface.h"
#include "PartialJointPolicyPureVector.h"

#include "JointObservationHistoryTree.h"
#include "JointActionObservationHistoryTree.h"

using namespace std;

#define DEBUG_GMAA_EM 0

GMAA_MAA_ELSI::GMAA_MAA_ELSI(
    const PlanningUnitMADPDiscreteParameters &params,
    size_t horizon, 
    FactoredDecPOMDPDiscreteInterface* p) :
    GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete(params, horizon, p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "GMAA_MAA_ELSI::GMAA_MAA_ELSI(PlanningUnitMADPDiscreteParameters params, size_t horizon, FactoredDecPOMDPDiscreteInterface* p)  called"<<endl;
}

GMAA_MAA_ELSI::GMAA_MAA_ELSI(
    size_t horizon, 
    FactoredDecPOMDPDiscreteInterface* p) :
    GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete(horizon, p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "GMAA_MAA_ELSI::GMAA_MAA_ELSI(size_t horizon, FactoredDecPOMDPDiscreteInterface* p)  called"<<endl;
}

void GMAA_MAA_ELSI::ResetPlanner()
{
}

bool GMAA_MAA_ELSI::ConstructAndValuateNextPolicies(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi,
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
        bool &cleanUpPPI)
{
    cleanUpPPI=true;
    return(CAVNP_quick_n_dirty2(ppi, poolOfNextPolicies));
}


bool GMAA_MAA_ELSI::CAVNP_quick_n_dirty2(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi,
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies)
{
    PJPDP_sharedPtr jpolPrevTs = ppi->GetJPol();//jpol^ts-1
    size_t depth = jpolPrevTs->GetDepth(); // = depth = ts(jpolPrevTs) + 1
    size_t ts = depth; //jpol = jpol^ts-1, we construct BG for ts == depth
    bool is_last_ts = (ts ==  GetHorizon() - 1);
    if(!is_last_ts)
        return(ConstructAndValuateNextPoliciesExactBG(ppi, poolOfNextPolicies));
    //if last stage -
    cout << "\n\n---------------------------------------------"<<endl;
    cout << "GMAA_MAA_ELSI::ConstructAndValuateNextPolicies called for last"<<
        " stage. This is where the exploiting of interaction should happen\n";

    stringstream ss;
    ss << "GMAA::Last_stage_NDP_ts" << ts;    
    StartTimer(ss.str());

    //prerequisites - we need 
    //nrOHts, nrJOHts, firstOHtsI
    //
    const size_t nrA = GetNrAgents();
    const vector<size_t> nrActions = GetNrActions();
    vector<size_t> nrOHts;
    for(Index agentI = 0; agentI < GetNrAgents(); agentI++)
        nrOHts.push_back( GetNrObservationHistories(agentI, ts) );
    size_t nrJOHts = GetNrJointObservationHistories(ts);
        //stores the indices of the first OH for time step ts for each agent
    vector<Index> firstOHtsI;
    Fill_FirstOHtsI(ts, firstOHtsI);
    
    //initialize the BG:
    boost::shared_ptr<BayesianGameCollaborativeGraphical> cgbg=
        boost::shared_ptr<BayesianGameCollaborativeGraphical>(
            new BayesianGameCollaborativeGraphical(nrA, nrActions, nrOHts));

//CONSTRUCT CGBG
    cout << endl;
    cout << "-------------------------------------------\n";
    cout << "--CONSTRUCTING GRAPHICAL BAYESIAN GAME-----\n";
    cout << "-------------------------------------------\n";

/*--------------
 * 1) add scopes
 * -------------
 */
    cout << "\n1) adding scopes"<<endl;
    //for each LRF
    size_t nrLQFs = GetNrHeuristicLQFs(ts);
    if(nrLQFs <= 0)
    {
        throw E(" GetNrHeuristicLQFs(); <= 0");
    }
    for(size_t e=0; e < nrLQFs; e++)
    {
        Scope agSc = GetAgentScopeForLQF(e, ts);
        cgbg->AddLRF(agSc);
    }

/*-------------
 * 2) compute the joint distribution P(s,aoh), given the past policy (and b^0)
 * ------------
 *      -we maintain this as P(s,jaoh) = P(s|jaoh) * P(jaoh)
 *      -and only do this for the consistent jaoh
 *      -en passant, we compute the expected past reward (for stages 0...t-1)
 */

    cout << "2) computing joint distribution P(s,aoh), given the past policy (and b^0)"<<endl;
    // a mapping from joint types (=joh) to jaoh (for the past pol jpolPrevTs)
    vector<Index> jt2jaohI;
    // P(s,jaohI)   = P(s,jtI) = P(s|jtI)*P(jtI)
    //              = jb_given_jtI[jtI]->at(s) * PjtI[jtI]
    vector<double> PjtI;
    vector<JointBeliefInterface*> jb_given_jtI;

    //the expected reward for time-steps 0...ts-1
    //this will be calculated as Sum_jaoh P(jaoh) R_{0...ts-1}(jaoh)
    double ExpR_0_prevTS = 0.0;
    for(Index jtI = 0; jtI < nrJOHts; jtI++)
    {
        const vector<Index> indTypes = cgbg->JointToIndividualTypeIndices(jtI);
        //array for the joint observations at ts=1,...,ts
        Index joI_arr[ts];
        Fill_joI_Array(ts, indTypes, firstOHtsI, joI_arr);
        const JointObservationHistoryTree* joht = Get_joht(ts, joI_arr);
        //see what joint action-observation history corresponds to jpolPrevTs
        Index jaI_arr[ts];//the taken joint actions at t=0,...,ts-1
        Fill_jaI_Array(ts, joht, jpolPrevTs, jaI_arr);
        //now we know the taken actions and the observation history, so we 
        //can reconstruct the joint action-observation history and its 
        //probability.
        Index jaohI = 0;
        double PjaohI = 0.0;
        //the cumulative reward over 0...ts-1 GIVEN that this JAOH occurs.
        double ExpR_0_prevTS_thisJAOH = 0.0;
        //get the joah Index and corresponding reward and prob.
        JointBeliefInterface* jb=
            ProbRewardForjoahI(ts, jtI, jaI_arr,joI_arr, jaohI, PjaohI, 
              ExpR_0_prevTS_thisJAOH );
        //cache the computed stuff:
        jt2jaohI.push_back(jaohI);
        PjtI.push_back(PjaohI);
        jb_given_jtI.push_back(jb);
        //Store the expected past reward for this jtI.
        ExpR_0_prevTS += ExpR_0_prevTS_thisJAOH * PjaohI;

        //now we have found the jaohI corresponding to johI (jtI) and 
        //previous policy jpolPrevTs, so we can get the Q-value and prob. for 
        //the BG.
        cgbg->DistributeProbability(jtI, PjaohI);
    }
/*--------------
 * 3) compute the LRFQs
 * -------------
 */
    //let use write jtGI and jaGI for the restricted joint type and action
    //(i.e., restricted to the scope of the concerning LRF)

    cout << "3) compute the LRFQs "<<endl;
    
    vector < vector < vector<double > > > LRFQ;
    vector < vector < vector<double > > > LRFQWeighted;
    //initialize
    //  LRFQ[e][jtGI][jaGI], and  
    //  LRFQWeighted[e][jtI][jaGI]
    for(size_t e=0; e < nrLQFs; e++)
    {
        size_t nrJT = cgbg->GetNrJointTypes();
        size_t nrJTG = cgbg->GetNrJointTypesForLRF(e);
        size_t nrJAG = cgbg->GetNrJointActionsForLRF(e);
        LRFQ.push_back( vector < vector<double > >(nrJTG) );
        LRFQWeighted.push_back( vector < vector<double > >(nrJT) );
        for(Index jtI=0; jtI < nrJT; jtI++)
            LRFQWeighted.at(e).at(jtI) = vector<double>(nrJAG, 0.0);
        for(Index jtGI=0; jtGI < nrJTG; jtGI++)
            LRFQ.at(e).at(jtGI) = vector<double>(nrJAG, 0.0);
    }



    for(Index jtI = 0; jtI < nrJOHts; jtI++)
    {
        Index jaohI = jt2jaohI.at(jtI);
        vector<Index> joah_vec = 
            JointToIndividualActionObservationHistoryIndices(jaohI);


        //get the vector with probs P(jtI_agSC{e}) consistent with jtI.
        vector<double> restrictedJType_probs = 
            cgbg->GetRestrictedJointTypeProbabilities(jtI);
        for(size_t e=0; e < nrLQFs; e++)
        {
            //compute LRFQWeighted[e][jtI][jaGI] for all jaGI
            // sum x_e
            //   sum x_rest
            //     P(x_e,oah) += P(s,oah)
            //   LRFQWeighted += P(x_e,oah) * Q(x_e, oah_e,ja_e)
            // LRFQWeighted /= P(oah_e)
            //
            /* which is given by the following latex code:
             * \QEd\ed(\oaHistT\ts,\jaG{\agl})=\frac{\sum_{\sfacGT{\sfl}{\ts}}\left[\sum_{\sfacGT{\bar{\sfl}}{\ts}}\Pr(\sfacGT{\sfl}{\ts},\sfacGT{\bar{\sfl}}{\ts},\oaHistGT{\bar{\agl}}{\ts},\oaHistGT{\agl}{\ts}|b^{0},\jpol^{*})\right]\QEd\ed(\sfacGT{\sfr}{\ts},\oaHistGT{\agl}{\ts},\jaG{\agl})}{\Pr(\oaHistGT{\agl}{\ts}|b^{0},\jpol^{*})}
             */

            //for now, we use last stage. This should be dependent on the stage
            //in general...
            Scope agSC_e =  GetAgentScopeForLQF(e, ts);
            vector<Index> jaoh_e(agSC_e.size());
            IndexTools::RestrictIndividualIndicesToScope(
                joah_vec, agSC_e, jaoh_e);
            Scope sfSC_e =  GetStateFactorScopeForLQF(e, ts);

            size_t nrRestrictedJA = cgbg->GetNrJointActionsForLRF(e);
            //for us it is easier to loop over joint states.
            for(Index sI=0; sI < GetNrStates(); sI++)
            {
                vector<Index> sfVecFul = StateIndexToFactorValueIndices(sI);
                vector<Index> sfacVector(sfSC_e.size());
                IndexTools::RestrictIndividualIndicesToScope(sfVecFul, sfSC_e, sfacVector);

                //sfSC_sI = IndividualToJointStateIndex(e, sfacVector);
                //P(s,oahist)
                //double P_s_joah = PjtI[jtI]*jb_given_jtI[jtI]->Get(sI);
                ////P(s,oahist)
                //double P_s_joah = PjtI[jtI]*jb_given_jtI[jtI][sI];

                double P1 = PjtI.at(jtI);
                double P2 = jb_given_jtI.at(jtI)->Get(sI);
                double P_s_joah = P1 * P2;
                if (P_s_joah > 0)
                    for(Index jaGI=0; jaGI < nrRestrictedJA; jaGI++)
                    {
#if 0
                        cout << "adding e " << e << " " << jtI << " " << jaGI
                             << " " << P_s_joah << " "
                             << GetHeuristicLRFQ(e, sfacVector, jaoh_e, jaGI )
                             << endl;
#endif
                        LRFQWeighted.at(e).at(jtI).at(jaGI) += P_s_joah
                            * GetHeuristicLocalQValue(e, ts, sfacVector, jaoh_e, jaGI );
                    }
                
                
            }

            for(Index jaGI=0; jaGI < nrRestrictedJA; jaGI++)
            {
                LRFQWeighted.at(e).at(jtI).at(jaGI) = 
                    LRFQWeighted.at(e).at(jtI).at(jaGI) /
                    restrictedJType_probs.at(e);
                Index jtGI = cgbg->JointToGroupTypeIndex(e,jtI);
                LRFQ.at(e).at(jtGI).at(jaGI) +=
                    LRFQWeighted.at(e).at(jtI).at(jaGI);
            }
            //process next LRF

        }//end for e in LRF

        //process next joint type jtI
    }
    //now we have computed the LRF values
    
    cout << "4) Writing the computed LRFQs values to the CGBG "<<endl;
    //and now we can write them to the CGBG
    for(size_t e=0; e < nrLQFs; e++)
    {
        cout << "writing utilities to CGBG for LRF "<<e<<endl;
        for(Index jtGI = 0; jtGI < cgbg->GetNrJointTypesForLRF(e); jtGI++)
            for(Index jaGI = 0; jaGI < cgbg->GetNrJointActionsForLRF(e); jaGI++)
            {
#if 0
                cout << "e " << e << " " << jtGI << " " << jaGI << " "
                     << LRFQ.at(e).at(jtGI).at(jaGI) << endl;
#endif
                cgbg->SetUtility(e, jtGI, jaGI, LRFQ.at(e).at(jtGI).at(jaGI) );
            }
    }
//END OF CONSTRUCT CGBG

#if 0
    cout << "Printing graphical Bayesian game:" << endl;
    cgbg->Print();
#endif

    cout << endl;
    cout << "----------------------------------------------\n";
    cout << "--FINISHED CONSTRUCTING CGBG, STARTING SOLVE--\n";
    cout << "----------------------------------------------\n";

    
    if(_m_verboseness >= 2 )
        cout <<"BG for time-step" <<ts<<" constructed, nrJOHts="
             << nrJOHts << ", nrJPols=" << cgbg->GetNrJointPolicies() << endl;
    if(_m_verboseness >= 3) 
        cout << "previously obtained expected reward="<<ExpR_0_prevTS<<endl;

    if(cgbg->GetNrJointPolicies()==0)
        throw(E("GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete:ConstructBayesianGame nrJPols==0, possible overflow?"));

    StopTimer(ss.str());
    _m_bgCounter++;
    if(_m_bgBaseFilename!="")
    {
        stringstream ss;
        ss << _m_bgBaseFilename << _m_bgCounter;
//        BayesianGameIdenticalPayoff::Save(*cgbg,ss.str());
    }
//    return(cgbg);


    //solve the CGBG
    BGCG_SolverNonserialDynamicProgramming bg_solver(cgbg);
    double maxLBv = bg_solver.Solve()+ExpR_0_prevTS;
            //if using a heuristic that does not specify the exact immediate
            //reward, we still want to return the *EXACT* payoff here...
    cerr << "warning, GMAA_MAA_ELSI might give in-exact result when used with heuristics that do not faitfully represent the immediate reward"<<endl;

    JointPolicyPureVector bestBGjpol = bg_solver.GetJointPolicyPureVector();

    //first construct a Dec-POMDP policy out of bestBGjpol
    PJPDP_sharedPtr jpolTs = ConstructExtendedJointPolicy(*jpolPrevTs,
            bestBGjpol, nrOHts, firstOHtsI);


    if(_m_verboseness >= 1) 
        cout << "\n>>Solved CGBG for t="<<ts<<". Max. expected value "
            <<"(including true expected reward up to this ts:"<<ExpR_0_prevTS
            << ") is:" << maxLBv <<endl
            << "The resulting joint policy is:"<<jpolTs->SoftPrint()<<endl;

    poolOfNextPolicies->Insert( NewPPI(jpolTs, maxLBv ) );
    // NOTE: do NOT use jpolTs after this point!!!
    // (it might be deleted, right?!)

    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);
}


void GMAA_MAA_ELSI::SelectPoliciesToProcessFurther(
    const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
    bool are_LBs, double bestLB)
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




bool GMAA_MAA_ELSI::
ConstructAndValuateNextPoliciesExactBG(
    const PartialPolicyPoolItemInterface_sharedPtr &ppi,
    const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies)
{
    PJPDP_sharedPtr jpolPrevTs = ppi->GetJPol();//jpol^ts-1
    size_t depth = jpolPrevTs->GetDepth(); // = depth = ts(jpolPrevTs) + 1
    size_t ts = depth; //jpol = jpol^ts-1, we construct BG for ts == depth
    bool is_last_ts = (ts ==  GetHorizon() - 1);

    vector<size_t> nrOHts; //the number of types for the BG we're constructing
    size_t nrJOHts;      
    vector<Index> firstOHtsI;

    // Construct the bayesian game for this timestep - 
    // This also returns ExpR_0_prevTS (the expected reward for time-steps
    // 0...ts-1 (given jpolPrevTs) ),  nrOHts, nrJOHts and firstOHtsI.
    double ExpR_0_prevTS = 0.0;
    boost::shared_ptr<BayesianGameIdenticalPayoff> bg_ts=
        boost::shared_ptr<BayesianGameIdenticalPayoff>(
            ConstructBayesianGame(jpolPrevTs,
                                  nrOHts, nrJOHts, firstOHtsI, ExpR_0_prevTS));

    //the policy for the Bayesian game 
    JointPolicyPureVector jpolBG = JointPolicyPureVector(  bg_ts);

    // Do the full enumeration of all joint policies for the BG...
    bool carry_over = false;
    LIndex nrJPols = bg_ts->GetNrJointPolicies();
    LIndex i = 0;

    if(_m_verboseness >= 2) 
        cout << "starting on solution of BG for t="<<ts<<" with nrJPols="
            <<nrJPols<<endl;

    stringstream ss;
    ss << "GMAA::NextExact_ts" << ts;
    StartTimer(ss.str());

    //some variables used when this is the last time-step
    double maxLBv = -DBL_MAX;
    JointPolicyPureVector bestLBjpolBG = JointPolicyPureVector(bg_ts);
    while(!carry_over)
    {
        PrintProgress("BG joint policy", i++, nrJPols, 10000);
        //eval the expected future payoff of this jpolBG
        double v = 0.0;        
        for(Index jt = 0; jt < nrJOHts; jt++)
        {
            Index jaI = jpolBG.GetJointActionIndex(jt);
            double jt_prob = bg_ts->GetProbability(jt);
            double jt_util = bg_ts->GetUtility(jt, jaI);
            v += jt_prob * jt_util;
        }
        //add the expected reward for 0...ts-1
        v += ExpR_0_prevTS;

        if(!is_last_ts) 
        {   
            //not the last time step...construct and return all 
            //partial DecPOMDP policies,
            
            // construct a policy for the DecPOMDP: a copy of jpolPrevTs
            // extended to this time step (ts) by jpolBG
            //PartialJointPolicyPureVector* jpolTs = 
            PJPDP_sharedPtr jpolTs = 
                ConstructExtendedJointPolicy(*jpolPrevTs,
                    jpolBG, nrOHts, firstOHtsI);

            // wrap the policy and put it in the pool of next policies.
            // (these next policies are candidates to be added to the 
            // main policy pool)
            // Warning: jpolTs may have been deleted afterward!
            // (if the policy is stored as in index for instance)
            poolOfNextPolicies->Insert( NewPPI(jpolTs, v) );

            // conserve memory 
            // is this necessary?  - this is performed by 
            //      'SelectPoliciesToProcessFurther' 
            // seems like duplicate work...
            //if(poolOfNextPolicies->Size()>(_m_nrPoliciesToProcess*10))
                //Prune(poolOfNextPolicies,_m_nrPoliciesToProcess);
        }
        else
        {   //this is the last time step: 
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
                    cout << "Last time step, found tight bound" << endl;
                break;
            }
        }
        carry_over = ++jpolBG; //next policy for the BG...
        _m_nrJPolBGsEvaluated++;
    }

    //if this is the last time step, we have not constructed the Dec-POMDP
    //policy which will be returned yet, so we do this now:
    if(is_last_ts) 
    {
        PJPDP_sharedPtr jpolTs = 
            ConstructExtendedJointPolicy(*jpolPrevTs,
                bestLBjpolBG, nrOHts, firstOHtsI);
        poolOfNextPolicies->Insert( NewPPI(jpolTs, maxLBv ) );
    }

    if(_m_verboseness >= 1) 
        cout << "Solved BG for t="<<ts<<". Max. expected value (including "
             << "true expected reward up to this ts:"<<ExpR_0_prevTS<<") is:"
             << poolOfNextPolicies->Select()->GetValue() <<endl;

    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);
}



// ExpR_0_prevTS is returned by this function.
BayesianGameIdenticalPayoff *
GMAA_MAA_ELSI::ConstructBayesianGame  (
        //input
        const boost::shared_ptr<const PartialJointPolicyDiscretePure> &jpolPrevTs, 
        //output arguments:
        vector<size_t>& nrOHts,
        size_t& nrJOHts,
        vector<Index>& firstOHtsI,
        double &ExpR_0_prevTS)// const //Should be fixed: can't make const because of timers...
{
    size_t depth = jpolPrevTs->GetDepth(); // = depth = ts(jpolPrevTs) + 1

    stringstream ss;
    ss << "GMAA::ConstructBG_ts" << depth;
    StartTimer(ss.str());

    if(_m_verboseness >= 1) 
        cout << ">>>ConstructBayesianGame(from ConstructAndValuateNextPolicies)"
             << " ts=" << depth << endl;
    if(_m_verboseness >= 2) 
        cout <<" - previous policy: " << jpolPrevTs->SoftPrintBrief() << endl;

    Index ts = depth; //jpol = jpol^ts-1, we construct BG for ts == depth
    //vector<size_t> nrOHts; //the number of types for the BG we're constructing

    for(Index agentI = 0; agentI < GetNrAgents(); agentI++)
        nrOHts.push_back( GetNrObservationHistories(agentI, ts) );
    //size_t 
    nrJOHts = GetNrJointObservationHistories(ts);
    
    //stores the indices of the first OH for time step ts for each agent
    //vector<Index> firstOHtsI;
    Fill_FirstOHtsI(ts, firstOHtsI);
    
    //the expected reward for time-steps 0...ts-1
    //this will be calculated as Sum_jaoh P(jaoh) R_{0...ts-1}(jaoh)
    ExpR_0_prevTS = 0.0;

    //initialize the BG:
    BayesianGameIdenticalPayoff *bg_ts=
        new BayesianGameIdenticalPayoff(GetNrAgents(), GetNrActions(), nrOHts);

    //for each joint obs. history (type of the BG), we determine the actions
    //that jpolPrevTs would have specified (i.e. we determine JAOH, the act-obs.
    //history). This is then used to compute:
    //  -the probability of this joint obs. history (given jpolPrevTs)
    //  -the expected reward over 0...ts-1 GIVEN that this JAOH occurs
    for(Index jtI = 0; jtI < nrJOHts; jtI++)
    {
        if(_m_verboseness >= 2) 
            PrintProgress("jtI",jtI,nrJOHts, 10);
        
        //we loop over Joint Type indices - these correspond to 
        //joint observation history indices, but non-trivially, so let's first
        //compute the joint observation history 
        const vector<Index> indTypes = bg_ts->JointToIndividualTypeIndices(jtI);

        //array for the joint observations at ts=1,...,ts
        Index joI_arr[ts];
        Fill_joI_Array(ts, indTypes, firstOHtsI, joI_arr);
        const JointObservationHistoryTree* joht = Get_joht(ts, joI_arr);

        //see what joint action-observation history corresponds to 
        // previous policy jpolPrevTs

        //first get all actions taken
        Index jaI_arr[ts];//the taken joint actions at t=0,...,ts-1
        Fill_jaI_Array(ts, joht, jpolPrevTs, jaI_arr);
        //now we know the taken actions and the observation history, so we 
        //can reconstruct the joint action-observation history and its 
        //probability.
        Index jaohI = 0;
        double PjaohI = 0.0;
        //the cumulative reward over 0...ts-1 GIVEN that this JAOH occurs.
        double ExpR_0_prevTS_thisJAOH = 0.0;
        //get the joah Index and corresponding reward and prob.
        ProbRewardForjoahI(ts, jtI, jaI_arr,joI_arr, jaohI, PjaohI, 
              ExpR_0_prevTS_thisJAOH );
        ExpR_0_prevTS += ExpR_0_prevTS_thisJAOH * PjaohI;

        //now we have found the jaohI corresponding to johI (jtI) and 
        //previous policy jpolPrevTs, so we can get the Q-value and prob. for 
        //the BG.
        bg_ts->SetProbability(jtI, PjaohI);
        for(Index jaI=0; jaI < GetNrJointActions(); jaI++)
        {
            if(PjaohI>0) // asking for a heuristic Q for a history
                         // that cannot have occurred might lead to
                         // problems (QMDP cannot compute a belief for
                         // instance, so just put 0
            {
                double ut =  GetHeuristicQ(jaohI, jaI);
                bg_ts->SetUtility(jtI, jaI, ut );
            }
            else
                bg_ts->SetUtility(jtI, jaI, 0);
        }
    
    }//end for jtI
    //now the Bayesian game is constructed completely.
    if(_m_verboseness >= 2) 
        cout <<"BG for time-step" <<ts<<" constructed, nrJOHts="
             << nrJOHts << ", nrJPols=" << bg_ts->GetNrJointPolicies() << endl;
    if(_m_verboseness >= 3) 
        cout << "previously obtained expected reward="<<ExpR_0_prevTS<<endl;

    if(bg_ts->GetNrJointPolicies()==0)
        throw(E("GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete:ConstructBayesianGame nrJPols==0, possible overflow?"));

    StopTimer(ss.str());
    _m_bgCounter++;
    if(_m_bgBaseFilename!="")
    {
        stringstream ss;
        ss << _m_bgBaseFilename << _m_bgCounter;
        BayesianGameIdenticalPayoff::Save(*bg_ts,ss.str());
    }
    return(bg_ts);
}



void  GMAA_MAA_ELSI::Fill_FirstOHtsI(Index ts, 
        vector<Index>& firstOHtsI)
{
    //because the OHs are constructed breath-first, we know the OHs for agent i
    //for this time step are numbered:
    //firstOHtsGI[i]...firstOHtsGI[i]+nrOH[i]-1
    //
    //(read first-OH-for-time-step-ts its Global Index)
    //
    //i.e. ohGI = ohI + firstOHtsGI 

    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        Index fI = CastLIndexToIndex(GetFirstObservationHistoryIndex(agI, ts));
        firstOHtsI.push_back(fI); 
    }
}




void GMAA_MAA_ELSI::Fill_joI_Array(const Index ts, 
        const vector<Index>& indTypes, 
        const vector<Index>& firstOHtsI, 
        Index* joI_arr)
{           
    //convert indiv type indices to ind. observation history indices:
    vector<Index> indOHI = vector<Index>(indTypes);
    vector<const ObservationHistory*> indOH;
    for(Index agentI=0; agentI < GetNrAgents(); agentI++)
    {
        indOHI[agentI] += firstOHtsI[agentI];
        const ObservationHistoryTree* oht = 
            GetObservationHistoryTree(agentI, indOHI[agentI]);
        const ObservationHistory* oh = oht->GetObservationHistory();
        indOH.push_back(oh);
    }
    //convert the vector of  ind. observation history indices to a array of
    //joint observations
    for(int tI=ts-1; tI >= 0; tI--)
    {
        vector<Index> indivObservationsTI;
        for(Index agentI=0; agentI < GetNrAgents(); agentI++)
        {
            indivObservationsTI.push_back( indOH[agentI]->
                    GetLastObservationIndex() );
            indOH[agentI] =  indOH[agentI]->GetPredecessor();
        }
        joI_arr[tI] = IndividualToJointObservationIndices(
                indivObservationsTI);
    }
}
void GMAA_MAA_ELSI::Fill_jaI_Array(
        Index ts, 
        const JointObservationHistoryTree* joht,
        const boost::shared_ptr<const PartialJointPolicyDiscretePure> &jpolPrevTs,
        Index* jaI_arr)
{
    if(! (ts >= 1) ) // to guarantee that t2 > 0
    {   //ts == 0, we're constructing the first time-step BG...
        ; 
    }
    else // ts > 1
    {
        Index t2 = ts;// t2 > 0
        do
        {
            t2--; //in first iter. t2 = ts-1 > 0 (because ts > 1, see above)
            joht = joht->GetPredecessor();
            Index johI = CastLIndexToIndex(joht->GetIndex());
            jaI_arr[t2] = jpolPrevTs->GetJointActionIndex(johI);
        }
        while(t2 > 0);
    }
    //t2 == 0
}
const JointObservationHistoryTree* GMAA_MAA_ELSI:: 
Get_joht(const Index ts, const Index* joI_arr)
{
    JointObservationHistoryTree* joht = 
        GetJointObservationHistoryTree(0);
    Index tI = 0;
    while(tI < ts)
    {
        joht = joht->GetSuccessor(joI_arr[tI]);
        tI++;
    }
    return(joht);
} 

/**Calculates the jaohI corresponding to jaI_arr and joI_arr and also 
 * returnes the P(jaohI) and the expected obtained reward for previous
 * time steps GIVEN this joint action history.
 *
 * basically this function is a form of
 *      PlanningUnitMADPDiscrete::GetJAOHProbs(Recursively)
 * that also computes the reward.
 *
 * */
JointBeliefInterface*
GMAA_MAA_ELSI::ProbRewardForjoahI(
        //input args
        Index ts, Index jtI, Index* jaI_arr,Index* joI_arr, 
        //output args
        Index& jaohI, double& PjaohI, double& ExpR_0_prevTS_thisJAOH )
{
    //first we get the initial jaoh
    JointActionObservationHistoryTree * jaoht = 
        GetJointActionObservationHistoryTree(0);

    double CPjaohI = 1.0; 
    PjaohI = CPjaohI; // == 1.0

    // get the inital belief
    JointBeliefInterface* jb = GetNewJointBeliefFromISD();

    Index tI = 0;
    while(tI < ts)
    {
        //calculate the R for tI
        double ExpR_0_prevTS_thisJAOH_thisT = 0.0;
        for(Index sI=0; sI < GetNrStates(); sI++)
        {
            double R_si_ja = GetReward(sI, jaI_arr[tI]);
#if DEBUG_GMAA4            
            if(DEBUG_GMAA4) 
                cout << "R(s="<<sI<<",ja="<<jaI_arr[tI]<<")="<< R_si_ja << "\n";
#endif                 
            ExpR_0_prevTS_thisJAOH_thisT += jb->Get(sI) * R_si_ja;
        }
        ExpR_0_prevTS_thisJAOH += ExpR_0_prevTS_thisJAOH_thisT;
#if DEBUG_GMAA4            
        if(DEBUG_GMAA4)
        {
            cout << "calculating expected reward R(oaHist,a) for tI="<<tI
                <<"oaHist:"; jaoht->GetJointActionObservationHistory()->Print();
            cout << endl; cout << "R(b,a) (exp reward for jtI="  << jtI << 
                ", tI="<<tI<<") is "<< ExpR_0_prevTS_thisJAOH_thisT <<endl;

        }
#endif                 
        jaoht = jaoht->GetSuccessor( jaI_arr[tI], joI_arr[tI] );
        jaohI = CastLIndexToIndex(jaoht->GetIndex());

        CPjaohI = jb->Update( *GetDPOMDPD(), jaI_arr[tI], joI_arr[tI]  );
        PjaohI =  PjaohI * CPjaohI;     
        tI++;
    }
#if DEBUG_GMAA4            
    if(DEBUG_GMAA4)
    {
        cout << "expected previous reward (up to ts-1) for (jtI="  << jtI << 
            ") ";
        jaoht->GetJointActionObservationHistory()->Print();
        cout << " is "<< ExpR_0_prevTS_thisJAOH <<endl << endl;
    }
#endif                 
    return(jb);
}


