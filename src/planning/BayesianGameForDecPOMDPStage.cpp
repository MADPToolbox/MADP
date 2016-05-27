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

#include "BayesianGameForDecPOMDPStage.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointPolicyDiscretePure.h"
#include "PartialJointPolicyPureVector.h"
#include "JointPolicyPureVector.h"
#include "JointObservationHistoryTree.h"
#include "JointActionObservationHistoryTree.h"
#include "JointBeliefInterface.h"
#include "QFunctionJAOHInterface.h"
#include "BeliefIteratorGeneric.h"

#define DEBUG_BG4DECPOMDP1 0
#define DEBUG_BG4DECPOMDP2 0
#define DEBUG_BG4DECPOMDP3 0
#define DEBUG_BG4DECPOMDP4 0

using namespace std;

//Default constructor
BayesianGameForDecPOMDPStage::BayesianGameForDecPOMDPStage(
        const PlanningUnitDecPOMDPDiscrete* pu,
        const QFunctionJAOHInterface* q,
        const boost::shared_ptr<const PartialJointPolicyDiscretePure> &pastJPol
    )
        :
            BayesianGameForDecPOMDPStageInterface(pastJPol),
            //BayesianGameBase( 
                //pu->GetNrAgents(), 
                //pu->GetNrActions(), 
                //pu->GetNrObservationHistoriesVector( pastJPol->GetDepth() )),
            BayesianGameIdenticalPayoff( 
                pu->GetNrAgents(), 
                pu->GetNrActions(), 
                pu->GetNrObservationHistoriesVector( pastJPol->GetDepth() )
                //the depth of the past policy = the stage we construct the BG 
                //for ( jpol^ts-1 has depth ts )
            )           
            ,_m_pu(pu)
            ,_m_qHeuristic(q)
            //,_m_pastReward(0.0)
            ,_m_JBs( GetNrJointTypes() )
            ,_m_areCachedImmediateRewards(false)
{
    //extra stuff we need to do...
    Initialize();

}
BayesianGameForDecPOMDPStage::BayesianGameForDecPOMDPStage(
        const PlanningUnitDecPOMDPDiscrete* pu
    )
        :
    BayesianGameForDecPOMDPStageInterface(static_cast<Index>(0)),
            BayesianGameIdenticalPayoff( 
                pu->GetNrAgents(), 
                pu->GetNrActions(), 
                pu->GetNrObservationHistoriesVector(0)
            )           
            ,_m_pu(pu)
            ,_m_qHeuristic(0)
            ,_m_JBs( 0 )
            ,_m_areCachedImmediateRewards(false)
{
}
BayesianGameForDecPOMDPStage::BayesianGameForDecPOMDPStage(
        const PlanningUnitDecPOMDPDiscrete* pu,
        const QFunctionJAOHInterface* q,
        Index t,
        size_t nrAgents,
        const vector<size_t>& nrActions,
        const vector<size_t>& nrTypes
        
    )
        :
            BayesianGameForDecPOMDPStageInterface(t)
            //,BayesianGameBase( nrAgents, nrActions, nrTypes )
            ,BayesianGameIdenticalPayoff( nrAgents, nrActions, nrTypes )
            ,_m_pu(pu)
            ,_m_qHeuristic(q)
            ,_m_JBs( GetNrJointTypes() )
            ,_m_areCachedImmediateRewards(false)
{
    //here we do NOT do any extra stuff! The derived class (that called this 
    //protected constructor) is responsible for filling 
    //-the utilities
    //-the probabilities
    //-_m_JBs
    //-...etc.
}
//Copy constructor.    
BayesianGameForDecPOMDPStage::BayesianGameForDecPOMDPStage(const BayesianGameForDecPOMDPStage& o) 
    :
        BayesianGameForDecPOMDPStageInterface( o ),
        //BayesianGameBase( o ),
        BayesianGameIdenticalPayoff ( o )
        ,_m_pu(o._m_pu)
        ,_m_qHeuristic(o._m_qHeuristic)
        ,_m_areCachedImmediateRewards(o._m_areCachedImmediateRewards)
        ,_m_immR(o._m_immR) //does this work for std::vector< std::vector<double> > ? gues so?
{
    //make deep copy of beliefs in _m_JB
    _m_JBs=std::vector< JointBeliefInterface* >(o._m_JBs.size(),0);
    for(Index k=0;k!=o._m_JBs.size();++k)
    {
        if(o._m_JBs.at(k))
            _m_JBs.at(k)=o._m_JBs.at(k)->Clone();
    }
}

//Destructor
BayesianGameForDecPOMDPStage::~BayesianGameForDecPOMDPStage()
{
    //free all the joint beliefs:
    std::vector< JointBeliefInterface* >::iterator it =  _m_JBs.begin();
    std::vector< JointBeliefInterface* >::iterator last =  _m_JBs.end();
    while(it != last)
    {
        delete *it;
        it++;
    }
}
//Copy assignment operator
BayesianGameForDecPOMDPStage& BayesianGameForDecPOMDPStage::operator= (const BayesianGameForDecPOMDPStage& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    this->BayesianGameIdenticalPayoff::operator=( o );
    _m_pu = o._m_pu;
    _m_qHeuristic = o._m_qHeuristic;    
    //_m_pastReward = o._m_pastReward;
    _m_areCachedImmediateRewards = o._m_areCachedImmediateRewards;
    _m_immR = o._m_immR; //does this work for std::vector< std::vector<double> > ? gues so?
    throw E("BayesianGameForDecPOMDPStage::operator= not fully implemented yet...");

    return *this;
}



void  BayesianGameForDecPOMDPStage::Fill_FirstOHtsI(Index ts, 
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
        Index fI = CastLIndexToIndex(_m_pu->GetFirstObservationHistoryIndex(agI, ts));
        firstOHtsI.push_back(fI); 
    }
}


//this function extends a previous policy jpolPrevTs for ts-1 with the behavior specified by the policy of the BayesianGame for time step ts (jpolBG).
PartialJointPolicyDiscretePure* 
BayesianGameForDecPOMDPStage::ConstructExtendedPolicy(
        PartialJointPolicyDiscretePure& jpolPrevTs, 
        JointPolicyDiscretePure& jpolBG, 
        vector<size_t>& nrOHts, 
        vector<Index>& firstOHtsI
        )
{
    //check policy types
    if(jpolPrevTs.GetPolicyDomainCategory() != PolicyGlobals::OHIST_INDEX)
        throw E("BayesianGameForDecPOMDPStage::ConstructExtendedPolicy --- jpolPrevTs.GetPolicyDomainCategory() != PolicyGlobals::OHIST_INDEX ");
    if(jpolBG.GetPolicyDomainCategory() != PolicyGlobals::TYPE_INDEX)
        throw E("BayesianGameForDecPOMDPStage::ConstructExtendedPolicy --- jpolPrevTs.GetPolicyDomainCategory() != PolicyGlobals::TYPE_INDEX ");
    //construct a policy for the DecPOMDP: 
    //a copy of jpolPrevTs with extended to this time step (ts) by 
    //jpolBG
    PartialJointPolicyDiscretePure* jpolTs = new 
        PartialJointPolicyPureVector(jpolPrevTs);
    jpolTs->SetDepth( jpolTs->GetDepth()+1 );
    for(Index agentI=0; agentI < GetNrAgents(); agentI++)
    {
        for(Index type = 0; type < nrOHts[agentI]; type++)
        {
            Index ohI = type + firstOHtsI[agentI];
            jpolTs->SetAction(agentI, ohI, 
                    jpolBG.GetActionIndex(agentI, type) );
        }         
    }
    return(jpolTs);
}

void BayesianGameForDecPOMDPStage::Fill_joI_Array(const Index ts, 
        const vector<Index>& indTypes, 
        const vector<Index>& firstOHtsI, 
        Index* joI_arr)
{           
    //convert indiv type indices to ind. observation history indices:
    vector<Index> indOHI = vector<Index>(indTypes);
    // indivObservations[ti][agI]  will contain the observation agI received at tI+1
    vector< vector<Index> > indivObservations(ts,vector<Index>(GetNrAgents()) );
    for(Index agentI=0; agentI < GetNrAgents(); agentI++)
    {
        indOHI[agentI] += firstOHtsI[agentI];
        Index obsArr[ts];
        _m_pu->GetObservationHistoryArrays(agentI, indOHI[agentI], ts, obsArr);
        //now obsArr is filled and can be copied into indivObservations
        for(Index tI=0; tI < ts; tI++)
            indivObservations.at(tI).at(agentI) = obsArr[tI];
    }

    for(Index tI=0; tI < ts; tI++)
        joI_arr[tI] = _m_pu->IndividualToJointObservationIndices(
                indivObservations[tI] );
}

//compute the joint actions taken bu jpolPrevTs when joIs is the true joint 
//observation history at stage ts.
void BayesianGameForDecPOMDPStage::Fill_jaI_Array(
        Index ts, 
        Index joIs[], //the array of joint observations issued 
        boost::shared_ptr<const JointPolicyDiscretePure> jpolPrevTs, 
        Index* jaI_arr
        )
{
    Index johI = 0;
    Index t = 0;
    while(t < ts)
    {
        Index ja = jpolPrevTs->GetJointActionIndex(johI);
        jaI_arr[t] = ja;

        Index next_joI = joIs[t];
        johI = _m_pu->GetSuccessorJOHI(johI, next_joI);
        t++;
    }
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
void BayesianGameForDecPOMDPStage::ProbRewardForjoahI(
        //input args
        Index ts, Index jtI, Index* jaI_arr,Index* joI_arr, 
        //output args
        Index& jaohI, double& PjaohI, double& ExpR_0_prevTS_thisJAOH )
{
    //first we get the initial jaoh
    JointActionObservationHistoryTree * jaoht = 
        _m_pu->GetJointActionObservationHistoryTree(0);

    double CPjaohI = 1.0; 
    PjaohI = CPjaohI; // == 1.0

    // get the initaal belief
    JointBeliefInterface* jb = _m_pu->GetNewJointBeliefFromISD();

    Index tI = 0;
    while(tI < ts)
    {
        //calculate the R for tI
        double ExpR_0_prevTS_thisJAOH_thisT = 0.0;
#if USE_BeliefIteratorGeneric
        BeliefIteratorGeneric it=jb->GetIterator();
        do
        {
            double R_si_ja = _m_pu->GetReward(it.GetStateIndex(), jaI_arr[tI]); 
            if(DEBUG_BG4DECPOMDP4) 
                cout << "R(s="<<it.GetStateIndex()<<",ja="<<jaI_arr[tI]<<")="<< R_si_ja << "\n";
            ExpR_0_prevTS_thisJAOH_thisT += it.GetProbability() * R_si_ja;
        } while(it.Next());
#else
        for(Index sI=0; sI < _m_pu->GetNrStates(); sI++)
        {
            double R_si_ja = _m_pu->GetReward(sI, jaI_arr[tI]); 
            if(DEBUG_BG4DECPOMDP4) 
                cout << "R(s="<<sI<<",ja="<<jaI_arr[tI]<<")="<< R_si_ja << "\n";
            ExpR_0_prevTS_thisJAOH_thisT += jb->Get(sI) * R_si_ja;
        }
#endif
        ExpR_0_prevTS_thisJAOH += ExpR_0_prevTS_thisJAOH_thisT;
        if(DEBUG_BG4DECPOMDP4)
        {
            cout << "calculating expected reward R(oaHist,a) for tI="<<tI
                <<"oaHist:"; jaoht->GetJointActionObservationHistory()->Print();
            cout << endl; cout << "R(b,a) (exp reward for jtI="  << jtI << 
                ", tI="<<tI<<") is "<< ExpR_0_prevTS_thisJAOH_thisT <<endl;

        }
            jaoht = jaoht->GetSuccessor( jaI_arr[tI], joI_arr[tI] );
            jaohI = CastLIndexToIndex(jaoht->GetIndex());

            CPjaohI = jb->Update( *_m_pu->GetDPOMDPD(), jaI_arr[tI], joI_arr[tI]  );
        PjaohI =  PjaohI * CPjaohI;     
        tI++;
    }
    delete jb; //free the belief allocated with 'new'
    if(DEBUG_BG4DECPOMDP4)
    {
        cout << "expected previous reward (up to ts-1) for (jtI="  << jtI << 
            ") ";
        jaoht->GetJointActionObservationHistory()->Print();
        cout << " is "<< ExpR_0_prevTS_thisJAOH <<endl << endl;
    }
}

/*
// ExpR_0_prevTS is returned by this function.
BayesianGameIdenticalPayoff *
BayesianGameForDecPOMDPStage::ConstructBayesianGame  (
        //input
        const JointPolicyDiscretePure* jpolPrevTs, 
        //output arguments:
        vector<size_t>& nrOHts,
        size_t& nrJOHts,
        vector<Index>& firstOHtsI,
        double &ExpR_0_prevTS)// const //Should be fixed: can't make const because of timers...
{
    size_t depth = jpolPrevTs->GetDepth(); // = depth = ts(jpolPrevTs) + 1

    //stringstream ss;
    //ss << "GMAA::ConstructBG_ts" << depth;
    //_m_pu->StartTimer(ss.str());

    if(DEBUG_BG4DECPOMDP1)
        cout << ">>>ConstructBayesianGame(from ConstructAndValuateNextPolicies)"
             << " ts=" << depth << endl;
    if(DEBUG_BG4DECPOMDP2)
        cout <<" - previous policy: " << jpolPrevTs->SoftPrintBrief() << endl;

    Index ts = depth; //jpol = jpol^ts-1, we construct BG for ts == depth
    //vector<size_t> nrOHts; //the number of types for the BG we're constructing

    for(Index agentI = 0; agentI < GetNrAgents(); agentI++)
        nrOHts.push_back( _m_pu->GetNrObservationHistories(agentI, ts) );
    //size_t 
    nrJOHts = _m_pu->GetNrJointObservationHistories(ts);
    
    //stores the indices of the first OH for time step ts for each agent
    //vector<Index> firstOHtsI;
    Fill_FirstOHtsI(ts, firstOHtsI);
    
    //the expected reward for time-steps 0...ts-1
    //this will be calculated as Sum_jaoh P(jaoh) R_{0...ts-1}(jaoh)
    ExpR_0_prevTS = 0.0;

    //initialize the BG:
    BayesianGameIdenticalPayoff *bg_ts=this;
//        new BayesianGameIdenticalPayoff(GetNrAgents(), GetNrActions(), nrOHts);

    if(DEBUG_BG4DECPOMDP2)
        cout <<"constructing the BG for time-step..." <<ts<<endl;

    //for each joint obs. history (type of the BG), we determine the actions
    //that jpolPrevTs would have specified (i.e., we determine JAOH, the act-obs.
    //history). This is then used to compute:
    //  -the probability of this joint obs. history (given jpolPrevTs)
    //  -the expected reward over 0...ts-1 GIVEN that this JAOH occurs
    for(Index jtI = 0; jtI < nrJOHts; jtI++)
    {
        if(DEBUG_BG4DECPOMDP2)
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
                double ut = _m_qHeuristic->GetQ(jaohI, jaI);
                bg_ts->SetUtility(jtI, jaI, ut );
            }
            else
                bg_ts->SetUtility(jtI, jaI, 0);
        }
    
    }//end for jtI
    //now the Bayesian game is constructed completely.

    if(DEBUG_BG4DECPOMDP2)
        cout <<"BG for time-step" <<ts<<" constructed, nrJOHts="
             << nrJOHts << ", nrJPols=" << bg_ts->GetNrJointPolicies() << endl;
#if DEBUG_BG4DECPOMDP3
    if(DEBUG_BG4DECPOMDP3)
        cout << "previously obtained expected reward="<<ExpR_0_prevTS<<endl;
#endif
 
    if(bg_ts->GetNrJointPolicies()==0)
        throw(E("BayesianGameForDecPOMDPStage:ConstructBayesianGame nrJPols==0, possible overflow?"));

    //StopTimer(ss.str());
    //_m_bgCounter++;
    //if(_m_bgBaseFilename!="")
    //{
        //stringstream ss;
        //ss << _m_bgBaseFilename << _m_bgCounter;
        //BayesianGameIdenticalPayoff::Save(*bg_ts,ss.str());
    //}
    return(bg_ts);
}
*/

void BayesianGameForDecPOMDPStage::Initialize()
{
    //stringstream ss;
    //ss << "GMAA::ConstructBG_ts" << depth;
    //_m_pu->StartTimer(ss.str());

    Index ts = GetStage();
    if(GetPastJointPolicy() == 0)
    {
        cerr << "Warning Initialize called without past joint policy: aborting."
            <<endl;
        return;
    }

    if(DEBUG_BG4DECPOMDP1)
        cout << ">>>BayesianGameForDecPOMDPStage::Initialize() called for"
             << " ts=" << ts << endl;
    if(DEBUG_BG4DECPOMDP2)
        cout <<" - previous policy: " << GetPastJointPolicy()->SoftPrintBrief() << endl;

    size_t nrJOHts = _m_pu->GetNrJointObservationHistories(ts);
    boost::shared_ptr<const JointPolicyDiscretePure> jpolPrevTs = GetPastJointPolicy();

    //stores the indices of the first OH for time step ts for each agent
    vector<Index> firstOHtsI;
    Fill_FirstOHtsI(ts, firstOHtsI);
    
    //the expected reward for time-steps 0...ts-1
    //this will be calculated as Sum_jaoh P(jaoh) R_{0...ts-1}(jaoh)
    //double ExpR_0_prevTS = 0.0;


    BayesianGameIdenticalPayoff *bg_ts=this;
    //for each joint obs. history (type of the BG), we determine the actions
    //that jpolPrevTs would have specified (i.e., we determine JAOH, the 
    //act-obs. history). This is then used to compute:
    //  -the probability of this joint obs. history (given jpolPrevTs)
    //  -the expected reward over 0...ts-1 GIVEN that this JAOH occurs
    for(Index jtI = 0; jtI < nrJOHts; jtI++)
    {
        if(DEBUG_BG4DECPOMDP2)
            PrintProgress("jtI",jtI,nrJOHts, 10);
        
        //we loop over Joint Type indices - these correspond to 
        //joint observation history indices, but non-trivially, so let's first
        //compute the joint observation history 
        const vector<Index> indTypes = bg_ts->JointToIndividualTypeIndices(jtI);

        //array for the joint observations at ts=1,...,ts
        Index joI_arr[ts];
        Fill_joI_Array(ts, indTypes, firstOHtsI, joI_arr);

        //we don't want to be dependent on the generation of joint
        //observation histories
        //const JointObservationHistoryTree* joht = Get_joht(ts, joI_arr);
        //Index johI = _m_pu->GetJointObservationHistoryIndex(ts, joI_arr);

        //see what joint action-observation history corresponds to 
        // previous policy jpolPrevTs

        //first get all actions taken
        Index jaI_arr[ts];//the taken joint actions at t=0,...,ts-1
        Fill_jaI_Array(ts, joI_arr, jpolPrevTs, jaI_arr);
        //now we know the taken actions and the observation history, so we 
        //can reconstruct the joint action-observation history and its 
        //probability.

        //the cumulative reward over 0...ts-1 GIVEN that this JAOH occurs.
        //double ExpR_0_prevTS_thisJAOH = 0.0;
        //get the joah Index and corresponding reward and prob.
        //new:
        Index jaohI = 0;
        double PjaohI = 1.0;
        JointBeliefInterface* jb = _m_pu->GetNewJointBeliefFromISD();
        if(ts > 0)
        {
            vector< Index > jaI_vec(&jaI_arr[0],&jaI_arr[ts]);
            vector< Index > joI_vec(&joI_arr[0],&joI_arr[ts]);
            jaohI = _m_pu->GetJointActionObservationHistoryIndex(ts, jaI_vec, 
                    joI_vec);
            PjaohI = _m_pu->GetJAOHProbsRecursively(jb, jaI_arr, joI_arr, 0,ts);
        }
        if(PjaohI>0)
        {
            if(jb->SanityCheck())
                _m_JBs.at(jtI) = jb;
            else
                throw(E("BayesianGameForDecPOMDPStage::Initialize() joint belief not valid"));
        }

/*        //old:
        ProbRewardForjoahI(ts, jtI, jaI_arr,joI_arr, jaohI, PjaohI, 
              ExpR_0_prevTS_thisJAOH );
        ExpR_0_prevTS += ExpR_0_prevTS_thisJAOH * PjaohI;
*/

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
                double ut = _m_qHeuristic->GetQ(jaohI, jaI);
                bg_ts->SetUtility(jtI, jaI, ut );
            }
            else
                bg_ts->SetUtility(jtI, jaI, 0);
        }
    
    }//end for jtI
    //now the Bayesian game is constructed completely.

    //perhaps store the previous reward somewhere?
    //_m_pastReward = ExpR_0_prevTS;
}


void BayesianGameForDecPOMDPStage::
ComputeAllImmediateRewards(const FactoredDecPOMDPDiscreteInterface *fd)
{
    if(_m_areCachedImmediateRewards)
        _m_immR.clear();

    size_t nrJT = GetNrJointTypes();
    size_t nrJA = GetNrJointActions();
    _m_immR = vector< vector <double> >(nrJT, vector<double>(nrJA, 0.0) );

    for(Index jtI=0; jtI< nrJT; jtI++)
        for(Index jaI=0; jaI< nrJA; jaI++)
            _m_immR.at(jtI).at(jaI) = ComputeImmediateReward(jtI, jaI);

    _m_areCachedImmediateRewards = true;
}

double BayesianGameForDecPOMDPStage::
ComputeImmediateReward(Index jtI, Index jaI) const
{
    //Index jaohI = _m_jaohReps.at(jtI);
    JointBeliefInterface* jb = _m_JBs.at(jtI);
    double r = 0.0;
    if(jb)
    {    
        BeliefIteratorGeneric bit = jb->GetIterator();
        do{
            Index s = bit.GetStateIndex();
            double p = bit.GetProbability();
            r += p * _m_pu->GetReward(s, jaI);
        }while (bit.Next() );
    }
    return r;
    
}
double BayesianGameForDecPOMDPStage::
ComputeDiscountedImmediateRewardForJPol(const boost::shared_ptr<JointPolicyDiscretePure> &jpolBG,
                                        const PlanningUnitFactoredDecPOMDPDiscrete *pu) const
{
    //eval the expected future payoff of this jpolBG
    double r = 0.0; //immediate reward (exact)
    size_t nrJT = this->GetNrJointTypes();
    for(Index jt = 0; jt < nrJT ; jt++)
    {
        Index jaI = jpolBG->GetJointActionIndex(jt);
        double jt_prob = this->GetProbability(jt);
        double jt_r = this->GetImmediateReward(jt, jaI);
        r += jt_prob * jt_r;
    }
    double discount = _m_pu->GetDiscount();
    double discT = pow(discount , (double)(GetStage()) );
    double discounted_r = discT * r;
    return discounted_r;    
}



string BayesianGameForDecPOMDPStage::SoftPrint() const
{
    stringstream ss;
    ss << "Bayesian Game for stage t="<<GetStage()<<" of a Dec-POMDP"<<endl;
    if(GetPastJointPolicy() != 0)
        ss<< "Past policy that lead to this BG=" << GetPastJointPolicy()->SoftPrint()<<endl;
    ss << BayesianGameIdenticalPayoff::SoftPrint();    
    return(ss.str());
}
