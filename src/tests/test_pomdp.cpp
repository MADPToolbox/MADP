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
 * Bas Terwijn
 *
 * For contact information please see the included AUTHORS file.
 */


#include "PerseusPOMDPPlanner.h"
#include "ProblemDecTiger.h"
#include "MADPParser.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"
#include "QAV.h"
#include "QMonahanPOMDP.h"
#include "QMonahanBG.h"
#include "QMDP.h"
#include "directories.h"
#include "NullPlanner.h"
#include "GMAA_kGMAA.h"
#include "AlphaVectorPlanning.h"
#include "JointBelief.h"

using namespace std;

int main(void)
{
    try
    {
        
        //string unixName="heavenOrHell_2_4_0.00_0.00";
        //string unixName="dectiger";
        //string unixName="oneDoor_2_7_0.20_0.20_0_2.dpomdp";
        string unixName="dectigerDiscounted";

        DecPOMDPDiscrete *decpomdp=
            new DecPOMDPDiscrete("","",
                                 directories::MADPGetProblemFilename(unixName));
        MADPParser parser(decpomdp);
   
#if 1

    NullPlanner *np;
    np=new NullPlanner(decpomdp);

    QAV<PerseusPOMDPPlanner>* q = 0;
    q = new QAV<PerseusPOMDPPlanner>(np);

    q->GetPlanner()->Initialize();

    q->GetPlanner()->ExportPOMDPFile("../../results/pomdp/test/test.POMDP");

    ValueFunctionPOMDPDiscrete V=q->GetPlanner()->GetImmediateRewardValueFunction();
    vector<AlphaVector>::const_iterator it=V.begin();
    while(it!=V.end())
    {
        cout << it->SoftPrint() << endl;
        it++;
    }

    GaoVectorSet G=q->GetPlanner()->BackProject(V);
#endif

#if 1 // print GaoVectorSet
    const boost::multi_array_types::size_type *GaoShape=G.shape();
    int nrA=GaoShape[0];
    int nrO=GaoShape[1];
    VectorSet *VS;

    for(GaoVectorSetIndex a=0;a!=nrA;a++)
        for(GaoVectorSetIndex o=0;o!=nrO;o++)
        {
            cout << "Gao a " << a << " o " << o << endl;

            VS=G[a][o];
            for(unsigned int k=0;k!=VS->size1();k++)
            {
                for(unsigned int s=0;s!=VS->size2();s++)
                    cout << (*VS)(k,s) << " ";
                cout << endl;
            }
        }
#endif
    
#if 1 // backup a single belief
    JointBelief b= *decpomdp->GetISD();

    b.Print();cout << endl;

    AlphaVector alpha=q->GetPlanner()->BeliefBackup(b,G);
    alpha.Print();
#endif

#if 0
    try {
        decpomdp.ConvertFiniteToInfiniteHorizon(h);
        q->ExportPOMDPFile("../../results/pomdp/test/test.POMDP");
        nrS=decpomdp.GetNrStates();
//        decpomdp.PrintInfo();
    }
    catch(E& e){ e.Print();}
#endif

#if 0 // sample a set of beliefs
    int nrB=10;
    VectorSet S=q->SampleBeliefs(nrB);
    for(VectorSetIndex k=0;k!=nrB;k++)
    {
        for(VectorSetIndex s=0;s!=nrS;s++)
            cout << S[k][s] << " ";
        cout << endl;
    }

    V=q->GetImmediateRewardValueFunction();
    vector<double> values=q->GetValues(S,V);
    for(VectorSetIndex k=0;k!=nrB;k++)
        cout << values[k] << " ";
    cout << endl;
#endif

#if 0 // run Perseus finite horizon

    GMAA_kGMAA *gmaa;
    PerseusPOMDPPlanner *perseus;

    PlanningUnitMADPDiscreteParameters params;
//    params.SetComputeAll(false);
    params.SetComputeAll(true);

    size_t h;
    h = 4;
    decpomdp->ConvertFiniteToInfiniteHorizon(h);
    gmaa=new GMAA_kGMAA(h, decpomdp, &params);

    perseus=new PerseusPOMDPPlanner(*gmaa);

    perseus->ExportPOMDPFile("../../results/pomdp/test/test.POMDP");
    BeliefSet S=perseus->SampleBeliefs(1000,true);
    perseus->SetBeliefSet(S);
    perseus->ExportBeliefSet("../../results/pomdp/test/beliefs.txt");
    perseus->Plan();
    perseus->ExportValueFunction("../../results/pomdp/test/test.alpha");

    QAV<PerseusPOMDPPlanner>* q = 0;
    q = new QAV<PerseusPOMDPPlanner>(*gmaa,perseus->GetValueFunction());
    gmaa->SetQHeuristic(q);

    gmaa->Plan();
    double v = gmaa->GetExpectedReward();
    cout << "v = " << v << endl;

    SimulationDecPOMDPDiscrete sim(*gmaa,h,1000);
    JointPolicyPureVector *jp=gmaa->GetJointPolicyPureVector();
    SimulationResult result=
        sim.RunSimulations(jp);

    cout << "avg reward " << result.GetAvgReward() << endl;
#endif

#if 0 // test backup

//    AlphaVector alpha0(nrS,-100);
    decpomdp->ConvertFiniteToInfiniteHorizon(h);
    q = new QAV<PerseusPOMDPPlanner>(*np);

    q->GetPlanner()->ExportPOMDPFile("../../results/pomdp/test/test.POMDP");
    
    ValueFunctionPOMDPDiscrete V0=q->GetPlanner()->GetImmediateRewardValueFunction();
    AlphaVectorPlanning::ExportValueFunction("../../results/pomdp/test/V0.alpha",V0);

//    V0.push_back(alpha0);
    GaoVectorSet G=q->GetPlanner()->BackProject(V0);
    JointBelief b=decpomdp->GetISD();
    AlphaVector alpha1=q->GetPlanner()->BeliefBackup(b,G);
    ValueFunctionPOMDPDiscrete V1;
    V1.push_back(alpha1);
    AlphaVectorPlanning::ExportValueFunction("../../results/pomdp/test/test.alpha",V1);

#endif
    
#if 0 // test Monahan

    QMonahanPOMDP * qm = 0;
    qm = new QMonahanPOMDP(*np,false);
    qm->Compute();
    qm->GetPlanner()->ExportValueFunction("../../results/pomdp/test/test.alpha",
                                          qm->GetValueFunction(2));
    np->SetQHeuristic(qm);

    double v=np->Plan();
    cout << "v = " << v << endl;

    SimulationDecPOMDPDiscrete sim(np);
    JointPolicyPureVector *jp=np->GetJointPolicyPureVector();
    SimulationResult result=
        sim.RunSimulations(jp,h,1000);

    cout << "avg reward " << result.GetAvgReward() << endl;
#endif

#if 0 // Test QMDP

    QMDP* qm = 0;
    qm = new QMDP(*np);

    qm->Compute();
    np->SetQHeuristic(qm);

    double v=np->Plan();
    cout << "v = " << v << endl;

    SimulationDecPOMDPDiscrete sim(np,h,1000);
    JointPolicyPureVector *jp=np->GetJointPolicyPureVector();
    SimulationResult result=
        sim.RunSimulations(jp);

    cout << "avg reward " << result.GetAvgReward() << endl;
#endif

#if 0 // Test MonahanBG

    QMonahanBG* qm = 0;
    qm = new QMonahanBG(*np);

    qm->Compute();
    qm->GetPlanner()->ExportValueFunction("../../results/pomdp/test/test.alpha",
                                          qm->GetPlanner()->
                                          GetValueFunction(1));
    np->SetQHeuristic(qm);

    double v=np->Plan();
    cout << "v = " << v << endl;

    SimulationDecPOMDPDiscrete sim(np);
    JointPolicyPureVector *jp=np->GetJointPolicyPureVector();
    SimulationResult result=
        sim.RunSimulations(jp,h,1000);

    cout << "avg reward " << result.GetAvgReward() << endl;
#endif

#if 0 // Test PerseusBG

    QAV<PerseusBGPlanner>* qpbg = 0;
    qpbg = new QAV<PerseusBGPlanner>(*np);

    decpomdp->ConvertFiniteToInfiniteHorizon(h);
    qpbg->GetPlanner()->ExportPOMDPFile("../../results/pomdp/test/test.POMDP");
    bool calculateJointBeliefs = false;
//    np->CreateActionObservationHistories();
//    np->CreateJointActionObservationHistories(calculateJointBeliefs);
    BeliefSet S=qpbg->GetPlanner()->SampleBeliefs(1000,true);
    qpbg->GetPlanner()->SetBeliefSet(S);
//    qpbg->ExportBeliefSet("../../results/pomdp/test/beliefs.txt");

    qpbg->Compute();
    qpbg->GetPlanner()->Perseus::ExportValueFunction("../../results/pomdp/test/test.alpha");
    np->SetQHeuristic(qpbg);

    double v=np->Plan();
    cout << "v = " << v << endl;

    SimulationDecPOMDPDiscrete sim(np);
    JointPolicyPureVector *jp=np->GetJointPolicyPureVector();
    SimulationResult result=
        sim.RunSimulations(jp,h,1000);

    cout << "avg reward " << result.GetAvgReward() << endl;

#endif

#if 0 // test approximate bg backup

    decpomdp->ConvertFiniteToInfiniteHorizon(h);
    QAV<PerseusBGPlanner>* qpbg = 0;
    qpbg = new QAV<PerseusBGPlanner>(*np);
   
    ValueFunctionPOMDPDiscrete V0=qpbg->GetPlanner()->
        GetImmediateRewardValueFunction();

    GaoVectorSet G=qpbg->GetPlanner()->BackProject(V0);
    JointBeliefSparse b=decpomdp->GetISD();
    AlphaVector alpha1=qpbg->GetPlanner()->
        BeliefBackup(b,0,G,V0,
                     EXHAUSTIVE_ONLYKEEPMAX);
    AlphaVector alpha2=qpbg->GetPlanner()->
        BeliefBackup(b,0,G,V0,
                     EXHAUSTIVE_STOREALL); 
    AlphaVector alpha3=qpbg->GetPlanner()->
        BeliefBackup(b,0,G,V0,
                     BGIP_SOLVER_EXHAUSTIVE);
    AlphaVector alpha4=qpbg->GetPlanner()->
        BeliefBackup(b,0,G,V0,
                     BGIP_SOLVER_ALTERNATINGMAXIMIZATION);
    ValueFunctionPOMDPDiscrete V1;
    V1.push_back(alpha1);
    V1.push_back(alpha2);
    V1.push_back(alpha3);
    V1.push_back(alpha4);
    AlphaVectorPlanning::ExportValueFunction("../../results/pomdp/test/test.alpha",V1);

#endif

#if 0 // Test PerseusImplicitWaiting

    double waitPenalty=1;

    QAV<PerseusImplicitWaitingPlanner>* q = 0;
    q = new QAV<PerseusImplicitWaitingPlanner>(*np,waitPenalty);

    decpomdp->ConvertFiniteToInfiniteHorizon(h);
//    q->GetPlanner()->ExportPOMDPFile("../../results/pomdp/test/test.POMDP");
//    bool calculateJointBeliefs = false;
    BeliefSet S=q->GetPlanner()->SampleBeliefs(1000,true);
    q->GetPlanner()->SetBeliefSet(S);
//    qpbg->ExportBeliefSet("../../results/pomdp/test/beliefs.txt");

    q->Compute();
    q->GetPlanner()->Perseus::ExportValueFunction("../../results/pomdp/test/test.alpha");

    params.SetComputeAll(false);
    PlanningUnitDecPOMDPDiscrete *np=new NullPlanner(h,decpomdp);
           
    SimulationDecPOMDPDiscrete sim(np,1000,42);
    SimulationResult result;

    vector<AgentDelayedAndCurrentSharedObs*> agents;
    for(unsigned int i=0;i!=decpomdp->GetNrAgents();++i)
    {
        AgentDelayedAndCurrentSharedObs *agent1;
        AgentImplicitWaiting agent(*np,0,q);
        agent1=new AgentImplicitWaiting(agent);
        agent1->SetIndex(i);
        agents.push_back(agent1);
    }
    result=sim.RunSimulations(agents);

//     switch(useBG)
//     {
//     case 0:
//     {
//         vector<AgentSharedObservations*> agents;
//         for(unsigned int i=0;i!=decpomdp->GetNrAgents();++i)
//         {
//             AgentSharedObservations *agent1;
//             QAV<PerseusPOMDPPlanner> *Qpomdp=new QAV<PerseusPOMDPPlanner>(*np,valueFunction.str());
//             AgentPOMDP agent(*np,0,Qpomdp);
//             agent1=new AgentPOMDP(agent);
//             agent1->SetIndex(i);
//             agents.push_back(agent1);
//         }
//         result=sim.RunSimulations(agents);
//         break;
//     }
//     default:
//     {
//         vector<AgentDelayedSharedObservations*> agents;
//         for(unsigned int i=0;i!=decpomdp->GetNrAgents();++i)
//         {
//             AgentDelayedSharedObservations *agent1;
//             QAV<PerseusBGPlanner> *Qbg=new QAV<PerseusBGPlanner>(*np,valueFunction.str());
//             AgentBG agent(*np,0,Qbg);
//             agent1=new AgentBG(agent);
//             agent1->SetIndex(i);
//             agents.push_back(agent1);
//         }
//         result=sim.RunSimulations(agents);
//         break;
//     }
//     }

    cout << "avg reward " << result.GetAvgReward() << endl;

#endif

    }
    catch(E& e){ e.Print(); }

    return(0);
}
