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

#include <iostream>
#include <string>
#include <vector>

#include "JESPDynamicProgrammingPlanner.h"
#include "MADPParser.h"
#include "DecPOMDPDiscrete.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "ProblemFireFightingGraph.h"
#include "NullPlanner.h"
#include <sys/times.h>

void TestInitialStateSampling(DecPOMDPDiscrete *decpomdp);
void TestSimulations(PlanningUnitDecPOMDPDiscrete* pu, boost::shared_ptr<JointPolicyDiscrete> jp, size_t nrRuns=10000);
void TestSimulations(PlanningUnitDecPOMDPDiscrete* pu, std::vector< AgentLocalObservations* >& agents, size_t nrRuns);
void TestSuccessorSampling(DecPOMDPDiscrete *decpomdp);
void TestObservationSampling(DecPOMDPDiscrete *decpomdp);

void TestDecTiger(size_t horizon);
void TestDecTigerAgentSimulation();
void TestDecTigerInfiniteHor();
void TestFFG(size_t nrAgents, size_t nrFLs, size_t horizon);

using namespace std;

#define NRRUNS 100000
size_t nrSimRuns = 100000;
tms timeStruct;
clock_t ticks_before, ticks_after, period;

int main(void)
{
    try
    {
        size_t nrAgents = 2, nrFLs = 3, horizon = 4;
#if 1
        TestDecTiger(horizon);

#endif
#if 1
        
        //size_t nrAgents = 4, size_t nrFLs = 3, size_t horizon = 4;
        TestFFG(nrAgents,  nrFLs,  horizon);
        TestFFG(6,  nrFLs,  horizon);
        
#endif                 


    }
    catch(E& e){ e.Print();}
    
}//end of main


void TestDecTiger(size_t horizon)
{
    cout << "==========================================================" << endl;
    cout << "==TestDecTiger for horizon " << horizon << " =============================" << endl;
    DecPOMDPDiscrete dectiger("DecTiger problem", "simple toy problem with 2 agents who have to jointly open the good (non-tiger) door", "../../problems/dectiger.dpomdp");
    MADPParser dpomdpd_parser (&dectiger);
    JESPDynamicProgrammingPlanner JESPe(horizon,&dectiger);
    JESPe.SetSeed(time(0));
    //JESPe.SetProblem(&dectiger);
    //JESPe.SetHorizon(3);
    cout << "Planning with JESP..." << endl;
    JESPe.Plan();
    double val = JESPe.GetExpectedReward();
    cout << "...done - expected value =" << val << endl;
  
//        TestInitialStateSampling(&dectiger);
//        TestSuccessorSampling(&dectiger);
//        TestObservationSampling(&dectiger);

    boost::shared_ptr<JointPolicyDiscrete> jp=JESPe.GetJointPolicyDiscrete();
    //jp->Print();
    
    ticks_before = times(&timeStruct);
    TestSimulations(&JESPe,jp, nrSimRuns);
    ticks_after = times(&timeStruct);
    period = ticks_after - ticks_before;
    cout << "Sampling "<< nrSimRuns << " simulation runs of DecTiger " <<
        "costs "<< period << " ticks"<<endl;
}

void TestFFG(size_t nrAgents, size_t nrFLs, size_t horizon)
{
    cout << "==========================================================" << endl;
    cout << "==TestFFG for agents=" << nrAgents << ", nrFls=" << nrFLs << ", horizon=" << horizon << " ===============" << endl;

    ProblemFireFightingGraph* p = 
                new ProblemFireFightingGraph(nrAgents, nrFLs);
    NullPlanner np(horizon, p);
    boost::shared_ptr<JointPolicyPureVector> jpol=
        boost::shared_ptr<JointPolicyPureVector>(new JointPolicyPureVector(&np));
    ticks_before = times(&timeStruct);
    TestSimulations(&np, jpol, nrSimRuns);
    ticks_after = times(&timeStruct);
    period = ticks_after - ticks_before;
    cout << "Sampling "<< nrSimRuns << " simulation runs of FFG " <<
        "costs "<< period << " ticks"<<endl;
}

void TestInitialStateSampling(DecPOMDPDiscrete *decpomdp)
{
    int i;
    vector<int> states(decpomdp->GetNrStates(),0);
    Index s;
    vector<double> isd(2);
    StateDistribution *originalISD;

    cout << "Test with sampling from different initial state distributions" << endl;

    originalISD = decpomdp->GetISD();

    states[0]=0;
    states[1]=0;
    cout << SoftPrintVector(decpomdp->GetISD()) << endl;
    for(i=0;i<NRRUNS;i++)
    {
        s=decpomdp->SampleInitialState();
        states[s]++;
    }
    cout << states[0] << " " << states[1] << endl;

    isd[0]=0;
    isd[1]=1;
    {
        StateDistributionVector *isdv=new StateDistributionVector(isd);
        decpomdp->SetISD(isdv);
    }

    states[0]=0;
    states[1]=0;
    cout << SoftPrintVector(decpomdp->GetISD()) << endl;
    for(i=0;i<NRRUNS;i++)
    {
        s=decpomdp->SampleInitialState();
        states[s]++;
    }
    cout << states[0] << " " << states[1] << endl;

    isd[0]=1;
    isd[1]=0;
    StateDistributionVector *isdv=new StateDistributionVector(isd);
    decpomdp->SetISD(isdv);
    states[0]=0;
    states[1]=0;
    cout << SoftPrintVector(decpomdp->GetISD()) << endl;
    for(i=0;i<NRRUNS;i++)
    {
        s=decpomdp->SampleInitialState();
        states[s]++;
    }
    cout << states[0] << " " << states[1] << endl;

    decpomdp->SetISD(originalISD);
}

void TestObservationSampling(DecPOMDPDiscrete *decpomdp)
{
    cout << "jo\tja\ts'\tP"<<endl;
    for(Index jo_i = 0; jo_i < decpomdp->GetNrJointObservations(); jo_i++)
        for(Index ja_i = 0; ja_i < decpomdp->GetNrJointActions(); ja_i++)
            for(Index s_ip = 0; s_ip < decpomdp->GetNrStates(); s_ip++)
                cout << jo_i << "\t" << ja_i << "\t" << s_ip << "\t" <<
                    decpomdp->GetObservationProbability(ja_i, s_ip, jo_i) << endl;
  
    int nrJO=decpomdp->GetNrJointObservations();
    vector<Index> count(nrJO,0);
    Index states[10]={0,0,0,0,0,1,1,1,1,1};
    Index jointActions[10]={0,1,2,3,4,6,4,1,2,4};
    Index jo,si,ja;
    int nrRuns=NRRUNS;

    for(int j=0;j<10;j++)
    {
        si=states[j];
        ja=jointActions[j];
        for(int k=0;k<nrJO;k++)
            count[k]=0;

        for(int i=0;i<nrRuns;i++)
        {
            jo=decpomdp->SampleJointObservation(ja,si);
            count[jo]++;
        }
        cout << ja << "\t" << si << "\t";
        for(int k=0;k<nrJO;k++)
            cout << count[k] << "\t";
        cout << " prob ";
        for(int k=0;k<nrJO;k++)
            cout << ((double)count[k]/nrRuns) << "\t";
        cout << endl;
    }

}

void TestSuccessorSampling(DecPOMDPDiscrete *decpomdp)
{
    double p = 0.0;
    cout << "s\tja\ts'\tP"<<endl;
    for(Index sI = 0; sI < decpomdp->GetNrStates(); sI++)
        for(Index jaI = 0; jaI < decpomdp->GetNrJointActions(); jaI++)
            for(Index sIp = 0; sIp < decpomdp->GetNrStates(); sIp++)
            {
                p = decpomdp->GetTransitionProbability(sI, jaI, sIp); 
                cout << sI << "\t" << jaI << "\t" << sIp << "\t" << p << endl;
            }
    
    vector<Index> count(decpomdp->GetNrStates(),0);
    Index states[10]={0,0,0,0,0,1,1,1,1,1};
    Index jointActions[10]={0,1,2,3,4,6,4,1,2,4};
    Index s,si,ja;
    int nrRuns=NRRUNS;

    for(int j=0;j<10;j++)
    {
        si=states[j];
        ja=jointActions[j];
        count[0]=0;
        count[1]=0;
        for(int i=0;i<nrRuns;i++)
        {
            s=decpomdp->SampleSuccessorState(si,ja);
            count[s]++;
        }
        cout << si << "\t" << ja << "\t" << count[0] << "\t" << count[1] << "\t" 
             << ((double)count[0]/nrRuns) << "\t" << ((double)count[1]/nrRuns) << endl;
    }

}

void TestSimulations(PlanningUnitDecPOMDPDiscrete* pu,
                     vector<AgentLocalObservations*>& agents, 
                     size_t nrRuns)
{
    cout << "Starting simulation of "<<nrRuns<<" runs"<<endl;  

    SimulationDecPOMDPDiscrete sim(*pu, nrRuns);
    //sim.SetVerbose(true);
    SimulationResult result;
    result=sim.RunSimulations(agents);

    cout << "Averaged reward: " << result.GetAvgReward() << endl;
    double er = 0.0;
    try{
        er = pu->GetExpectedReward();
        cout << "Expected reward: " << er  << endl;
    } catch(E& e) {
        //NullPlanner throws exception
        cout << "Expected reward was not defined by planning unit" << endl;
    }
    result.Save("/tmp/tst_sim_results");
}

void TestSimulations(PlanningUnitDecPOMDPDiscrete* pu,
                     boost::shared_ptr<JointPolicyDiscrete> jp, size_t nrRuns)
{
//    int nrRuns=10000;
    cout << "Starting simulation of "<<nrRuns<<" runs"<<endl;  

    SimulationDecPOMDPDiscrete sim(*pu,nrRuns);

    SimulationResult result;
    result=sim.RunSimulations(jp);

    cout << "Averaged reward: " << result.GetAvgReward() << endl;
    double er = 0.0;
    try{
        er = pu->GetExpectedReward();
        cout << "Expected reward: " << er  << endl;
    } catch(E& e) {
        //NullPlanner throws exception
        cout << "Expected reward was not defined by planning unit" << endl;
    }


    result.Save("/tmp/tst_sim_results");
}
