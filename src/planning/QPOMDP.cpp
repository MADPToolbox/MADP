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

#include "QPOMDP.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointActionObservationHistoryTree.h"
#include "JointObservationHistoryTree.h"
#include "JointObservation.h"
#include "JointBeliefInterface.h"
#include "BeliefIteratorGeneric.h"
#include <float.h>

using namespace std;

#define DEBUG_QPOMDP 0
#define DEBUG_QPOMDP_COMP 0
#define DEBUG_QPOMDP_COMPREC 0

#if DEBUG_QPOMDP_COMPREC
#include "BayesianGameIdenticalPayoff.h"
#endif

//Default constructor
QPOMDP::QPOMDP(const PlanningUnitDecPOMDPDiscrete* pu)
    :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOHTree(pu) 
{
}

QPOMDP::QPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
    :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOHTree(pu) 
{
}

//Destructor
QPOMDP::~QPOMDP()
{
}

//this uses cached joint beliefs per def.:
#if QFunctionJAOH_useIndices
double QPOMDP::ComputeRecursively(size_t time_step, 
                                  LIndex jaohI,
                                  Index lastJAI)
#else
double QPOMDP::ComputeRecursively(size_t time_step, 
                                  JointActionObservationHistoryTree* jaoht,
                                  Index lastJAI)
#endif
// time_step    is the time-step of the BG that is constructed (and solved) in 
//              this function
//              
// jah          joint action history at t=time_step-1
// joh          joint obser. history at t=time_step-1
// (together they are denoted jaoh, the joint act-obs hist. at t-1 )
//
// jB           jaoh induces a probability over states, the joint belief
//
// lastJA       ja at t=time_step-1 (so the JA taken at joah )
{
    bool last_t = ( (time_step + 1) == GetPU()->GetHorizon()) ;

#if !QFunctionJAOH_useIndices
    JointActionObservationHistory* jaoh = jaoht->
        GetJointActionObservationHistory();
    Index jaohI = jaoht->GetIndex();
#endif

#if DEBUG_QPOMDP_COMPREC
    cout << "QPOMDP::ComputeRecursively:"<< endl 
         << "time_step t="<<time_step << ", prev. jaoh index jaoh^(t-1)="<<jaohI
         << ", prev. ja="<<lastJAI <<endl;

    BayesianGameIdenticalPayoff bg_time_step(GetPU()->GetNrAgents(), 
                                             GetPU()->GetNrActions(),
                                             GetPU()->GetNrObservations());
#endif

    double v = 0.0;
    double discount = GetPU()->GetDiscount();
    JointBeliefInterface* newJB = GetPU()->GetNewJointBeliefInterface();
    //for all jointobservations newJO (jo^time_step)
    for(Index newJOI=0; newJOI < GetPU()->GetNrJointObservations(); newJOI++)
    {
        if(DEBUG_QPOMDP_COMPREC){ 
            cout << "looking for joint observationI="<<newJOI <<" (=";
            GetPU()->GetJointObservation(newJOI)->Print();cout <<")"<<endl;}

        Index new_jaohI;
#if QFunctionJAOH_useIndices
        new_jaohI = CastLIndexToIndex(GetPU()->GetSuccessorJAOHI(jaohI, lastJAI, newJOI));
#else    
        JointActionObservationHistoryTree* new_jaoht = jaoht->GetSuccessor(
                lastJAI,newJOI);
        new_jaohI = new_jaoht->GetIndex();
#endif
        
        //get the new joint belief at this time-step resulting from lastJAI, 
        //newJOI...(the true prob. dist over states for the actions and obser-
        //vations as given by the history < jaoh, lastJA, newJOI > )
            
        //double Po_ba = GetPU()->GetJBeliefConditionalProb(new_jaohI);


        double Po_ba = GetPU()->GetJAOHProbs(newJB, new_jaohI, jaohI);

        // if the probability of this observation occurring is zero,
        // the belief is not defined, and don't have to consider this
        // part of the tree anymore
        if(Po_ba<PROB_PRECISION)
            continue;

        if(DEBUG_QPOMDP_COMPREC){
            cout << "the new jaoh (for this joint observationI="<<newJOI<<")\n"
                "new jaohI="<< new_jaohI <<
                endl<<" new belief newJB="<< newJB->SoftPrint() << endl; }
#if DEBUG_QPOMDP_COMPREC
            bg_time_step.SetProbability(newJOI, Po_ba);
#endif
        double maxQ = -DBL_MAX;
        for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
        {
            //calculate R(joah',newJA) - expected immediate reward for time_step
            double exp_imm_R = 0.0;

#if USE_BeliefIteratorGeneric
            BeliefIteratorGeneric it=newJB->GetIterator();
            do exp_imm_R += it.GetProbability() *
                GetPU()->GetReward(it.GetStateIndex(), newJAI);
            while(it.Next());
#else
            for(Index sI=0; sI < GetPU()->GetNrStates(); sI++)
//--------------------------------------------
//NOTE:
//the following line crashes with sparse beliefs, because sparse beliefs
//are initialized with
//        return (new JointBeliefSparse() );
//( rather then new JointBelief(GetNrStates()) )
//Therefore the Get() request here can fail:
//--------------------------------------------
                exp_imm_R += newJB->Get(sI)*GetPU()->GetReward(sI, newJAI);
#endif
            
            //calculate Q(jaoh', newJA) =  R(joah',newJA) + exp. future R
            //  and the exp. future R = ComputeRecursively(t+1, jaoh', newJA)
            double exp_fut_R = 0.0;
            if(!last_t)
#if QFunctionJAOH_useIndices
                exp_fut_R = ComputeRecursively(time_step+1, new_jaohI, newJAI);
#else
                exp_fut_R = ComputeRecursively(time_step+1, new_jaoht, newJAI);
#endif
            double Q = exp_imm_R + discount * exp_fut_R;
            if(Q > maxQ)
                maxQ = Q;
            _m_QValues(new_jaohI,newJAI)=Q;
#if DEBUG_QPOMDP_COMPREC
                bg_time_step.SetUtility(newJOI, newJAI, Q);
#endif
        }//end for newJAI
#if DEBUG_QPOMDP_COMPREC
        {
            //BG used to store and then print
            bg_time_step.PrintUtilForJointType(newJOI);
            cout << "->max = " << maxQ<<endl;
        }
#endif
        // v = v + P(jo|b,a) * max_a Q(b'_jo,a)
        v += Po_ba * maxQ;
    }//end for newJOI

    delete newJB;

    if(DEBUG_QPOMDP_COMPREC)
    {
        cout << "QPOMDP::ComputeRecursively:"<< endl << "time_step t="<<
            time_step << ", prev. jaoh index jaoh^(t-1)="<<jaohI
             << ", prev. ja="<<lastJAI <<endl
             <<"FINISHED - v="<<v<<endl<<endl;
    }
    return( v );
}

/*
void QPOMDP::ComputeNoCache()
//size_t time_step = 0,
// Index jahI = 0, 
// Index johI = 0,
// const vector<double> JB = ISD, 
// Index lastJAI = none)
{
    if(_m_initialized == false)
        throw E("QPOMDP::Compute - QBG not initialized");    
    if(GetPU() == 0)
        throw E("QPOMDP::Compute - GetPU() returns 0; no PlanningUnit available!");

    size_t time_step = 0;
    Index init_johI = 0;
    Index init_jahI = 0;
    
    bool last_t = false;
    if( (time_step + 1) == GetPU()->GetHorizon()) //unlikely, but possible
        last_t = true;

    if(DEBUG_QPOMDP_COMP){
        cout << "QPOMDP::Compute() called" << endl;
    }

    JointBeliefInterface* b0 = GetPU()->GetISD(); 


    //in the first time_step t=0, there is no previous action and there is only
    //the empty observation action history.
    //Therefore we're going to construct a Bayesian game where there is only 1
    //type for each agent.
    vector<size_t> nrTypes = vector<size_t>(GetPU()->GetNrAgents(), 1);
    BayesianGameIdenticalPayoff bg_time_step(GetPU()->GetNrAgents(), 
            GetPU()->GetNrActions(), nrTypes);

    size_t empty_jaohI = 0;
    //probability of initial empty aoHist is 1.0:
    bg_time_step.SetProbability(empty_jaohI, 1.0);
    //and the belief over states is the initial belief/
    
    //for all joint actions newJA
    for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
    {
        //calculate R(joah',newJA) - expected immediate reward for time_step
        double exp_imm_R = 0.0;
        for(Index sI=0; sI < GetPU()->GetNrStates(); sI++)
            exp_imm_R += b0[sI] * GetPU()->GetReward(sI, newJAI);
        
        //calculate Q(jaoh', newJA) =  R(joah',newJA) + exp. future R
        //  and the exp. future R = 
        //      ComputeRecursively(time_step+1, jah', joh', newJA)
        double exp_fut_R = 0.0;
        if(!last_t)
            exp_fut_R = ComputeRecursivelyNoCache( 1, 0, 0, b0, newJAI);
        double Q = exp_imm_R + exp_fut_R;
        //Store the Q value
        _m_QValues(empty_jaohI,newJAI)=Q;

        //add the Q value to the BayesianGame
        bg_time_step.SetUtility(empty_jaohI, newJAI, Q);
    }//end for newJAI
    if(DEBUG_QPOMDP_COMP)
    {
        cout << "QPOMDP::Compute() for..."<<endl<<
            " time_step=0  called, with ISD=";
        b0.Print();
        cout <<endl;
        bg_time_step.Print();
    }
    //solve this bayesian game
    BGIP_SolverBruteForceSearch bgs(bg_time_step);
    double v = bgs.Plan();
    cout << "QPOMDP::Compute() - Expected V(b0) = " << v << endl<< endl;
    return;
}



double QPOMDP::ComputeRecursivelyNoCache(size_t time_step, Index jahI, 
        Index johI,const JointBeliefInterface* JB, Index lastJAI)
// time_step    is the time-step of the BG that is constructed (and solved) in 
//              this function
//              
// jah          joint action history at t=time_step-1
// joh          joint obser. history at t=time_step-1
// (together they are denoted jaoh, the joint act-obs hist. at t-1 )
//
// jB           jaoh induces a probability over states, the joint belief
//
// lastJA       ja at t=time_step-1 (so the JA taken at joah )
{
    bool last_t = false;
    if( (time_step + 1) == GetPU()->GetHorizon())
        last_t = true;


    if(DEBUG_QPOMDP_COMPREC){
        cout << "QPOMDP::ComputeRecursively("<< endl << "time_step="<<
            time_step << ", jahI="<< jahI <<", johI="<< johI
            <<", JB, lastJAI="<<lastJAI
            <<") called, with JB=";
        JB.Print();
        cout <<endl;
    }

    //we're going to construct a Bayesian game where the types are the observa-
    //tions for o^t for t=time_step. (i.e. each joint observation is a joint 
    //type). 
    //These observations are following the history indicated by:
    // <jahI,johI>=jaoh, lastJAI (= jaoh^t-1, ja^t-1)
    //

    BayesianGameIdenticalPayoff bg_time_step(GetPU()->GetNrAgents(), 
            GetPU()->GetNrActions(), GetPU()->GetNrObservations());

    //the total expected reward for this time step (given jahI, johI, JB and
    //lastJAI)
    double v = 0.0;
    //for all jointobservations newJO (jo^time_step)
    for(Index newJOI=0; newJOI < GetPU()->GetNrJointObservations(); newJOI++)
    {
        if(DEBUG_QPOMDP_COMPREC){ 
            cout << "looking for joint observationI="<<newJOI <<" (=";
            GetPU()->GetJointObservation(newJOI)->Print(); cout <<")"<<endl;   }

        JointActionHistoryTree* new_jaht;
        Index new_jahI = 0;
        JointObservationHistoryTree* new_joht;
        Index new_johI = 0;
        if(!last_t)
        {
            //jaoh' = jaoh + lastJA + newJO
            new_jaht = GetPU()->GetJointActionHistoryTree(jahI)->
                GetSuccessor(lastJAI);
            new_jahI = new_jaht->GetIndex();
            new_joht = GetPU()->GetJointObservationHistoryTree(johI)->
                GetSuccessor(newJOI);
            new_johI = new_joht->GetIndex();
        }

        //calculate the new joint belief at this time-step 
        //resulting from lastJAI, newJOI...
        //(this is the true prob. dist over states for the actions and obser-
        //vations as given by the history < (johI,jahI), lastJA, newJOI > )
        JointBelief newJB=JB;
        double Po_ba = newJB.Update(*GetPU(), lastJAI, newJOI);

        if(DEBUG_QPOMDP_COMPREC){
            cout << "new belief newJB=";
            newJB.Print();
            cout << endl;
        }

        bg_time_step.SetProbability(newJOI, Po_ba);
        
        double maxQ = -INFTY;
        //for all joint actions newJA
        for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
        {
            //calculate R(joah',newJA) - expected immediate reward for time_step
            double exp_imm_R = 0.0;
            for(Index sI=0; sI < GetPU()->GetNrStates(); sI++)
                exp_imm_R += newJB[sI] * GetPU()->GetReward(sI, newJAI);
            
            //calculate Q(jaoh', newJA) =  R(joah',newJA) + exp. future R
            //  and the exp. future R = 
            //      ComputeRecursively(time_step+1, jah', joh', newJA)
            double exp_fut_R = 0.0;
            if(!last_t)
                exp_fut_R = ComputeRecursivelyNoCache(time_step+1, new_jahI, 
                new_johI, newJB, newJAI);
            double Q = exp_imm_R + exp_fut_R;
            if(Q > maxQ)
                maxQ = Q;
            
            //BG used to store and then print
            bg_time_step.SetUtility(newJOI, newJAI, Q);
        }//end for newJAI
        if(DEBUG_QPOMDP_COMPREC)
        {
            bg_time_step.PrintUtilForJointType(newJOI);
            cout << "->max = " << maxQ<<endl;
        }
        // v = v + P(jo|b,a) * max_a Q(b'_jo,a)
        v += Po_ba * maxQ;
    }//end for newJOI

    if(DEBUG_QPOMDP_COMPREC)
    {
        cout << "QPOMDP::ComputeRecursively for..."<<endl<<" time_step="<<
            time_step << ", Index jahI="<< jahI <<", Index johI="<< johI
            <<",const vector<double> JB, Index lastJAI="<<lastJAI
            <<") called, with JB=";
        JB.Print();
        cout <<endl << "Total expected reward: " << v << endl<< endl;
        //bg_time_step.Print();
    }

    //QPOMDP: BG not solved at the end but in the for-(newJOI-)loop 
    //BGIP_SolverBruteForceSearch bgs(bg_time_step);
    //double v = bgs.Plan();
    //if(DEBUG_QPOMDP_COMPREC)
    //    cout << "Expected reward under best policy for sub-BG="<<v<<endl;
    
    //return the expected reward under the best policy.
    return( v );
}
*/
