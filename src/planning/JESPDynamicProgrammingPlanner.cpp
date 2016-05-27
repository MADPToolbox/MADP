/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JESPDynamicProgrammingPlanner.h"
#include "ValueFunctionDecPOMDPDiscrete.h"
#include "JointPolicyPureVector.h"
#include "JointBeliefInterface.h"
#include "IndividualBeliefJESP.h"
#include "Scope.h"
#include <float.h>

using namespace std;

#define DEBUG_DPBR 0
#define DEBUG_DPJESP 0

JESPDynamicProgrammingPlanner::JESPDynamicProgrammingPlanner(
    const PlanningUnitMADPDiscreteParameters &params,
    size_t horizon,
    DecPOMDPDiscreteInterface* p
    ) :
    PlanningUnitDecPOMDPDiscrete(params, horizon, p),
    _m_foundPolicy()
    //,_m_exhBRBestPol(*this)
{
}

JESPDynamicProgrammingPlanner::JESPDynamicProgrammingPlanner(
    int horizon,
    DecPOMDPDiscreteInterface* p
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon, p),
    _m_foundPolicy()
    //,_m_exhBRBestPol(*this)
{
}

void JESPDynamicProgrammingPlanner::Plan()
{
    if(DEBUG_DPJESP){ 
        cout << "\n---------------------------------"<<endl;
        cout << "Exhaustive JESP  - Plan() started"<<endl;
        cout << "---------------------------------"<<endl;
    }
    double v_best = -DBL_MAX;
    JointPolicyPureVector* jpol = new JointPolicyPureVector(this);
    JointPolicyPureVector* best = new JointPolicyPureVector(this);
    jpol->RandomInitialization();
    //jpol->ZeroInitialization();
    
    if(DEBUG_DPJESP) {cout << "joint policy randomly initialized to:";
        jpol->Print();}
    
    int stop = 0;
    size_t nr_non_improving_agents = 0;
    while(nr_non_improving_agents < GetDPOMDPD()->GetNrAgents() -1
            && stop++ < 1000) 
    {
        int agentI = GetNextAgentIndex();
        double v = DynamicProgrammingBestResponse(jpol, agentI);
        if(v > v_best + 1e-9)
        {  
            (*best) = (*jpol);
            if(DEBUG_DPJESP)
                {cout << ">>>Plan: new best policy:"<<endl; best->Print();}
            v_best = v;
            nr_non_improving_agents = 0;
        }        
        else
            nr_non_improving_agents++;
    }
    _m_foundPolicy = JPPV_sharedPtr(best);
    _m_expectedRewardFoundPolicy=v_best;
    

    if(DEBUG_DPJESP){ 
        cout << "Exhaustive JESP  - resulting policy:"<<endl;
        cout << "------------------------------------"<<endl;
        best->Print();
    }
    delete jpol;
}

double JESPDynamicProgrammingPlanner::DynamicProgrammingBestResponse(
        JointPolicyPureVector* jpol, Index agentI)
{
#if DEBUG_DPBR
        cout << "JESPDynamicProgrammingPlanner::ExhaustiveBestResponse called "
             << "for agent " << agentI << endl;
#endif

    //create initial *augmented* POMDP belief B^0(b^0, oHist_{!=i}^0)
    //(B is an augmented POMDP belief, b is a joint belief (just over states))
    //
    //DP
    //compute
    //V(B^0)= \max_a Q(B^0, a)
    //      = \max_a [ R(B^0, a) + sum_o P(o|B^0,a) V(B^1)
    //
    //  where a,o are individual actions/observations.
    //
    //BELIEF UPDATE
    //  the belief update (computing B^1 from B^0) should be done as follows
    //  (we denote this agent with i, and the other (we assume 1 for simplicity)
    //  with j)
    //
    //  Bi^1(s',oHistj') = (1/ P(oi|Bi^0,ai) * 
    //      sum_s Bi^0(s, oHistj) P(s',oi,oj|s,ai,polj(oHistj))
    //
    
    //BELIEF implementation
    //in order to maintain a probability for each <s,ohist_j^t> pair
    //we need to enumerate them
    IndividualBeliefJESP B0( agentI, 0, *this );
    B0.Set( *GetProblem()->GetISD() );
    vector<Index> newpol (GetNrActionObservationHistories(agentI), 0);
    double v0 = DPBestResponseRecursively(agentI, 0, B0, 0, jpol, newpol);
#if DEBUG_DPBR
//    {   cout << "Best response V="<<v0<<endl;}
//    { cout << "policy="; jpol->Print();}
#endif    
    ConstructPolicyRecursively(agentI, 0, 0, 0, jpol, newpol);
    return(v0);
}

double JESPDynamicProgrammingPlanner::DPBestResponseRecursively
    (
     const Index agentI, //the agent we are computing the best response for
     const Index aohI,    //the oservation history of agentI
     const IndividualBeliefJESP& B,//for which we compute the value+action
     const Index stage, //the stage of B
     JointPolicyPureVector* jpol,//the joint policy
     vector<Index>& new_pol
    )
{

#if DEBUG_DPBR    
stringstream tabsss;
for(Index tab=0; tab < stage; tab++)
    tabsss << "\t";
string tabss = tabsss.str();
cout << tabss<<">>DPBestResponseRecursively(ag="<<agentI<<", aoh="<<aohI<<", B, stage="<<stage<< ", jpol) called, with " << endl <<tabss<<"B="<<endl;
B.Print();
#endif    
    Scope otherAgentIndices;
    for(Index agI=0; agI < GetNrAgents(); agI++)
        if(agI != agentI)
            otherAgentIndices.push_back(agI);

    size_t nrA = GetDPOMDPD()->GetNrActions(agentI);
    size_t nrO = GetDPOMDPD()->GetNrObservations(agentI);
    size_t nrAgents = GetNrAgents();
    double v_max = -DBL_MAX; //higest expected value
    Index a_br = 0; //and the corresponding best-response action
    for(Index actionI=0; actionI < nrA; actionI++)//compute value of this action
    {
        double v_a = 0;
        //Compute the expected immediate reward
        double Rba = 0.0;
        for(Index eI=0; eI < B.Size(); eI++) //eI is an index over e=<s,oHistJ>
        {
//-> check if this should be put in a function ? 
//( duplicated in IndividualBeliefJESP.Update() )
            Index sI = B.GetStateIndex(eI);
            vector<Index> oHistI_others = B.GetOthersObservationHistIndex(eI);
            vector<Index> actions(nrAgents);
            actions.at(agentI) = actionI;
            for(Index j=0; j < otherAgentIndices.size(); j++)
            {
                Index agJ = otherAgentIndices[j];
                Index oHistJ = oHistI_others[j];//not agJ!!!
                Index actJ = jpol->GetActionIndex(agJ, oHistJ);
                actions.at(agJ) = actJ;

            }
            Index jaI = IndividualToJointActionIndices(actions);
// <-             
            Rba += B.Get(eI) * GetReward(sI, jaI);
        }

        //Compute the future reward
        double F = 0.0;
        if(stage < GetHorizon() - 1)
        {
            double check_p = 0.0;
            for(Index observI=0; observI < nrO; observI++)
            {
                IndividualBeliefJESP Bao(agentI, stage+1, *this);
                double Po_ba = Bao.Update(B, actionI, observI, jpol);
                Index next_aohI = 
                    GetSuccessorAOHI(agentI, aohI, actionI, observI);
                double F_ao = DPBestResponseRecursively(agentI, next_aohI,
                        Bao, stage+1, jpol, new_pol);
                F += Po_ba * F_ao;
                check_p += Po_ba;
            }
            if( abs(check_p - 1) > 1e7)
                throw E("Po_ba not summing to 1");

        }
        v_a = Rba + F;
#if DEBUG_DPBR
cout << tabss<<"(stage="<<stage<<")actionI="<<actionI<<", Q(b,a)="<< v_a <<
    " (= R+F  =  "<<Rba<<" + "<< F <<")"<<endl;
#endif    
        if(v_a > v_max)
        {
            v_max = v_a;
            a_br = actionI;
        }
    }
#if DEBUG_DPBR
cout << tabss<<">>ENDED DPBestResponseRecursively(ag="<<agentI<<", aoh="<<aohI
    <<", B, stage="<<stage<< ", jpol) called, with " <<endl<<tabss<<"B="<<endl;
cout << tabss<<"Selected actionI="<<a_br<<" with Q(b,a)="
    << v_max<< endl;
#endif    
    //NO!!! when we arrived at this ohI with some action history
    //we are overwriting the best action taken from a previous. one
    //jpol->SetAction(agentI, aohI, a_br);
    new_pol.at(aohI) = a_br;
    return v_max;
}


void JESPDynamicProgrammingPlanner::ConstructPolicyRecursively
    (
     const Index agentI, //the agent we are computing the best response for
     const Index aohI,    //the action-observation history of agentI
     const Index ohI,    //the observation history of agentI
     const Index stage, //the stage of B
     JointPolicyPureVector* jpol,//the joint policy
     vector<Index>& new_pol
    )
{
#if DEBUG_DPBR    
stringstream tabsss;
for(Index tab=0; tab < stage; tab++)
    tabsss << "\t";
string tabss = tabsss.str();
#endif    
    //the action that is best at this aohI
    Index best_a = new_pol[aohI];
    jpol->SetAction(agentI, ohI, best_a);

    size_t nrO = GetDPOMDPD()->GetNrObservations(agentI);
    if(stage < GetHorizon() - 1)
    {
        for(Index oI=0; oI < nrO; oI++)
        {
            Index next_aohI = GetSuccessorAOHI(agentI, aohI, best_a, oI);
            Index next_ohI = GetSuccessorOHI(agentI, ohI, oI);
            ConstructPolicyRecursively(agentI, next_aohI, next_ohI, stage+1,
                    jpol, new_pol);
        }
    }
#if DEBUG_DPBR
#endif    
}
