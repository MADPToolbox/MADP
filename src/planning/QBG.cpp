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

#include "QBG.h"
#include "JointBelief.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointActionObservationHistoryTree.h"
#include "JointObservationHistoryTree.h"
#include "JointObservation.h"
#include "JointAction.h"
#include "BeliefIteratorGeneric.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"

using namespace std;

#define DEBUG_QBG 0
#define DEBUG_QBG_COMP 0
#define DEBUG_QBG_COMPREC 0

//Default constructor
QBG::QBG(const PlanningUnitDecPOMDPDiscrete* pu)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOHTree(pu) 
{
}

QBG::QBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
    : 
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJAOHTree(pu) 
{
}

//Destructor
QBG::~QBG()
{    
}

//In general, we want to
//      calculate Q(jaoh', newJA) =  R(joah',newJA) + exp. future R
// here the exp. future R = ComputeRecursively(t+1, root, newJA)
//
// is given by the following function
//
//this uses cached joint beliefs per def.:
#if QFunctionJAOH_useIndices
double QBG::ComputeRecursively(size_t time_step, LIndex jaohI, Index lastJAI)
#else    
double QBG::ComputeRecursively(size_t time_step, JointActionObservationHistoryTree* jaoht, Index lastJAI)
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
    bool last_t = false;
    if( (time_step + 1) == GetPU()->GetHorizon())
        last_t = true;

#if !QFunctionJAOH_useIndices
    JointActionObservationHistory* jaoh = jaoht->
        GetJointActionObservationHistory();
    Index jaohI = jaoht->GetIndex();
#endif

    if(DEBUG_QBG_COMPREC){
        cout << "QBG::ComputeRecursively:"<< endl << "time_step t="
             << time_step << ", prev. jaoh index jaoh^(t-1)="<<jaohI
             << ", prev. ja="<<lastJAI<<"="<<
                GetPU()->GetJointAction(lastJAI)->SoftPrint()
             << ", now starting the computation of the future reward."
             <<endl;
    }

    //we're going to construct a Bayesian game where the types are the observa-
    //tions for o^t for t=time_step. (i.e. each joint observation is a joint 
    //type). 
    //These observations are following the history indicated by:
    // jaoh, lastJAI (= jaoh^t-1, ja^t-1)
    BGIP_sharedPtr bg_time_step=BGIP_sharedPtr(
        new BayesianGameIdenticalPayoff(GetPU()->GetNrAgents(), 
                                        GetPU()->GetNrActions(),
                                        GetPU()->GetNrObservations()));

    double discount = GetPU()->GetDiscount();

    //double v = 0.0; - we don't need to maintain this (not aver. over o)
    //for all jointobservations newJO (jo^time_step)
    for(Index newJOI=0; newJOI < GetPU()->GetNrJointObservations(); newJOI++)
    {
        if(DEBUG_QBG_COMPREC){ 
            cout << "looking for joint observationI="<<newJOI <<" (=";
            cout << GetPU()->GetJointObservation(newJOI)->SoftPrint();
            cout <<")"<<endl;
        }

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
        
        JointBeliefInterface* new_jbi = GetPU()->GetJointBeliefInterface(new_jaohI);
        JointBeliefInterface& newJB = *new_jbi;
        double Po_ba = GetPU()->GetJAOHProbGivenPred(new_jaohI);

        // if the probability of this observation occurring is zero,
        // the belief is not defined, and don't have to consider this
        // part of the tree anymore
        if(Po_ba<PROB_PRECISION)
            continue;

        if(DEBUG_QBG_COMPREC){
            cout << "new belief newJB=";
            newJB.Print();
            cout << endl;
        }
        bg_time_step->SetProbability(newJOI, Po_ba);
        //for all joint actions newJA
        for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
        {
            //calculate R(joah',newJA) - expected immediate reward for time_step
            double exp_imm_R = 0.0;

#if USE_BeliefIteratorGeneric
            BeliefIteratorGeneric it=newJB.GetIterator();
            do
            {
                double r_s_ja = GetPU()->GetReward(it.GetStateIndex(), newJAI);
                double prob_s = it.GetProbability();
                exp_imm_R += r_s_ja * prob_s;
            } while(it.Next());
#else
            for(Index sI=0; sI < GetPU()->GetNrStates(); sI++)
            {
                double r_s_ja = GetPU()->GetReward(sI, newJAI);
                double prob_s = newJB.Get(sI);
                exp_imm_R += r_s_ja * prob_s;
            }
#endif
            if(DEBUG_QBG_COMPREC){
                cout << "Expected imm reward for new JA"<< 
                        GetPU()->GetJointAction(newJAI)->SoftPrint()
                    << "exp_imm_R="<< exp_imm_R << endl
                    << "about to start recursively computing future reward..."<<
                    endl;
            }

            
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
            if(DEBUG_QBG_COMPREC){
                cout << "Returned to QBG::ComputeRecursively(ts="<< 
                    time_step << ", prev. jaohI="<<jaohI
                    << ", prev. ja="<<lastJAI <<")"<<endl
                    << "computed the future reward for "
                    << GetPU()->GetJointObservation(newJOI)->SoftPrint()
                    << "and "<< GetPU()->GetJointAction(newJAI)->SoftPrint()
                    << endl;
                cout << "Q = exp_imm_R + discount * exp_fut_R = "
                    << Q << " = " 
                    << exp_imm_R << " + "
                    << discount << " * "
                    << exp_fut_R
                    << endl;
            }
            _m_QValues(new_jaohI,newJAI)=Q;
            bg_time_step->SetUtility(newJOI, newJAI, Q);
        }//end for newJAI
        
        //joint belief no longer needed:
        delete new_jbi;

    }//end for newJOI

    //solve this bayesian game
    BGIP_SolverBruteForceSearch<JointPolicyPureVector> bgs(bg_time_step,0,1);
    double v = bgs.Solve();
    if(DEBUG_QBG_COMPREC){
        cout << "QBG::ComputeRecursively:"<< endl << "time_step t="<<
            time_step << ", prev. jaoh index jaoh^(t-1)="<<jaohI
             << ", prev. ja="<<lastJAI <<endl
        <<"constructed BG:";
        bg_time_step->Print();
        cout << "Expected reward under best policy for sub-BG="<<v<<endl<< endl;
    }
    
    return( v );
}


void QBG::ComputeNoCache()
{
    throw E("not implemented - should be copy/pasted from QPOMDP::ComputeNoCache() and then have minor adjustments");

}
double QBG::ComputeRecursivelyNoCache(size_t time_step, Index jahI, 
        Index johI, const JointBelief &JB, Index lastJAI)
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


    if(DEBUG_QBG_COMPREC){
        cout << "QBG::ComputeRecursively("<< endl << "time_step="<<
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
    BGIP_sharedPtr bg_time_step=BGIP_sharedPtr(
        new BayesianGameIdenticalPayoff(GetPU()->GetNrAgents(), 
                                        GetPU()->GetNrActions(),
                                        GetPU()->GetNrObservations()));
        
    double discount = GetPU()->GetDiscount();

    //for all jointobservations newJO (jo^time_step)
    for(Index newJOI=0; newJOI < GetPU()->GetNrJointObservations(); newJOI++)
    {
        if(DEBUG_QBG_COMPREC){ cout << "looking for joint observationI="<<newJOI
            <<" (=";
            GetPU()->GetJointObservation(newJOI)->Print();
            cout <<")"<<endl;
        }

        JointActionHistoryTree* new_jaht;
        Index new_jahI = 0;
        JointObservationHistoryTree* new_joht;
        Index new_johI = 0;
        if(!last_t)
        {
            //jaoh' = jaoh + lastJA + newJO
            new_jaht = GetPU()->GetJointActionHistoryTree(jahI)->
                GetSuccessor(lastJAI);
            new_jahI = CastLIndexToIndex(new_jaht->GetIndex());
            new_joht = GetPU()->GetJointObservationHistoryTree(johI)->
                GetSuccessor(newJOI);
            new_johI = CastLIndexToIndex(new_joht->GetIndex());
        }

        //calculate the new joint belief at this time-step 
        //resulting from lastJAI, newJOI...
        //(this is the true prob. dist over states for the actions and obser-
        //vations as given by the history < (johI,jahI), lastJA, newJOI > )
        JointBelief newJB=JB;
        double Po_ba = newJB.Update(*GetPU()->GetDPOMDPD(), lastJAI, newJOI);

        if(DEBUG_QBG_COMPREC){
            cout << "new belief newJB=";
            newJB.Print();
            cout << endl;
        }

        bg_time_step->SetProbability(newJOI, Po_ba);
        
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
            double Q = exp_imm_R + discount * exp_fut_R;
            //add the Q value to the BayesianGame
            bg_time_step->SetUtility(newJOI, newJAI, Q);
        }//end for newJAI
    }//end for newJOI

    if(DEBUG_QBG_COMPREC)
    {
        cout << "QBG::ComputeRecursively for..."<<endl<<
           "time_step="<<
            time_step << ", Index jahI="<< jahI <<", Index johI="<< johI
            <<",const vector<double> JB, Index lastJAI="<<lastJAI
            <<") called, with JB=";
        JB.Print();
        cout <<endl;
        bg_time_step->Print();
    }

    //solve this bayesian game
    BGIP_SolverBruteForceSearch<JointPolicyPureVector> bgs(bg_time_step);
    double v = bgs.Solve();
    if(DEBUG_QBG_COMPREC)
        cout << "Expected reward under best policy for sub-BG="<<v<<endl<< endl;
    
    //return the expected reward under the best policy.
    return( v );
}
