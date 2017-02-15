/**\file test_common_functions.cpp
 *
 * Authors:
 * Frans Oliehoek <faolieho@science.uva.nl>
 * Matthijs Spaan <mtjspaan@isr.ist.utl.pt>
 *
 * Copyright 2008 Universiteit van Amsterdam, Instituto Superior Tecnico
 *
 * This file is part of MultiAgentDecisionProcess.
 *
 * MultiAgentDecisionProcess is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * MultiAgentDecisionProcess is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MultiAgentDecisionProcess.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * $Id$
 */

#include <sys/times.h>
#include "ActionHistory.h"
#include <float.h>

using namespace std;

#include "argumentHandlers.h"

void TestActionHistories(const ArgumentHandlers::Arguments& args)
{
        cout << "-------------------"<<endl;
        cout << "TestActionHistories()"<<endl;
        cout << "-------------------"<<endl;
        int horizon = 3;
        cout << endl << "horizon="<< horizon <<endl;
        ProblemDecTiger pdt;
        JESPExhaustivePlanner plan(horizon, &pdt);
    //    plan.SetProblem(&pdt);
        
        Index agentI = 0;
        ActionHistory ah0 = ActionHistory(plan, agentI++);
        ActionHistory ah1 = ActionHistory(plan, agentI);
        Index actionI = 0;
        ActionHistory ah0t1 = ActionHistory(actionI,&ah0);
        ActionHistory ah0t2 = ActionHistory(actionI,&ah0t1);
        ah0t2.Print();  
        cout << "\nJESPExhaustivePlanner plan: "<<endl;
        plan.Print();

        cout << "\nActionObservation Histories:\n";
        cout << "----------------------------\n";
        plan.PrintActionObservationHistories();

        if (!args.testMode)
        {
            cout << "press <enter> to continue..."<<endl;
            cin.get();
        }
/*
 *      JESPExhaustivePlanner doesn't use JointActionObservationHistories
 *         cout << "\nJOINTActionObservation Histories:\n";
 *         cout << "----------------------------\n";
 *         plan.GetJointActionObservationHistoryTree(0)->Print();
 */
        

}
void TestValueFunction(const ArgumentHandlers::Arguments& args)
{
    cout << "-------------------"<<endl;
    cout << "TestValueFunction()"<<endl;
    cout << "-------------------"<<endl;
    int horizon = 4;
    cout << endl << "horizon="<< horizon <<endl;
    ProblemDecTiger pu;
    JESPExhaustivePlanner plan(horizon, &pu);
    plan.SetProblem(&pu);

    pu.Print();
    plan.Print();

    JointPolicyPureVector jpol(&plan);
    
    //ValueFunctionDecPOMDPDiscrete Vtest(plan, jpol);
    //Vtest.Test();

    //the time structure...
    tms timeStruct;
    

    int nrLoops = 1000;
    if (!args.testMode)
    {
        cout << endl <<"Press <enter> to start value (v0) calculation of "<<nrLoops
             <<" joint policies without caching."<<endl;
        cin.get();
    }

    clock_t ticks_before, ticks_after;
    ticks_before = times(&timeStruct);
    for(int i=0; i < nrLoops; i++)
    {
        jpol.RandomInitialization();
        ValueFunctionDecPOMDPDiscrete V(plan, jpol);
        /*double val =*/ V.CalculateV<false>();
    }
    ticks_after = times(&timeStruct);
    if (!args.testMode)
        cout << "done in "<< ticks_after - ticks_before << " clock ticks."<<endl;
    cout << "Now starting value (v0) calculation of "<<nrLoops<<
        " joint policies with caching."<<endl;
    ticks_before = times(&timeStruct);
    for(int i=0; i < nrLoops; i++)
    {
        jpol.RandomInitialization();
        ValueFunctionDecPOMDPDiscrete V(plan, jpol);
        /*double val =*/ V.CalculateV<true>();
    }    
    ticks_after = times(&timeStruct);
    if (!args.testMode)
        cout << "done in "<< ticks_after - ticks_before << " clock ticks."<<endl;

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
    
}


void TestNaming(const ArgumentHandlers::Arguments& args)
{   
    cout << "----------------------------------"<<endl;
    cout << "-         TestNaming()           -"<<endl;
    cout << "----------------------------------"<<endl;

    string s1("test - The Dec-Tiger Problem");
    string s2("test - A toy problem, 2-agent Dec-POMDP. 2 agents have to select 1 out of 2 doors, behind one is a tiger, behind the other treasure.");
    string s3("test - no file");

    DecPOMDPDiscrete dpd2(s1, s2, s3);
    string sb;
    sb = dpd2.GetName();
    cout << "name ="<<sb<<endl ;

    cout << "dpd.Print():" << endl ;
    dpd2.Print();

    cout << "done - dpd.Print()" << endl ;

}


void TestModelCreation(const ArgumentHandlers::Arguments& args)
{
    cout << "----------------------------------"<<endl;
    cout << "-       TestModelCreation()      -"<<endl;
    cout << "----------------------------------"<<endl;

    ProblemDecTiger pdt;
    cout << "----------------------------------"<<endl;
    cout << "->>>      pdt.Print():       -"<<endl;
    cout << "----------------------------------"<<endl;
    pdt.Print();
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeJointBeliefs(false);
    JESPExhaustivePlanner JESPe(2, &pdt, &params);
    cout << "----------------------------------"<<endl;
    cout << "->>> before SetHorizon()    JESPe.Print():       -"<<endl;
    cout << "----------------------------------"<<endl;
    JESPe.Print();     
//    JESPe.SetProblem(&pdt);
    JESPe.SetHorizon(2);
    cout << "----------------------------------"<<endl;
    cout << "->>> after SetHorizon()    JESPe.Print():"<<endl;
    cout << "----------------------------------"<<endl;
    JESPe.Print();
//Testing problems with the TransitonModelMapping
if(0){    
    cout << "creating some TransitionModelMapping..."<< endl; 
    TransitionModelMapping a(2,2);    
    a.Print();
}
if(0){//this is not leaking
    JointActionDiscrete* jai;
    ActionDiscrete* ai;
    cout << endl << "Joint action Memory leak test..." << endl; 
    for(int i=0; i < 10000; i++)
    {
    jai = new JointActionDiscrete();
    for(int j=0; j < 1000; j++)
    {
        ai = new ActionDiscrete(j);
        jai->AddIndividualAction(ai,j); 
         // DO delete ai: pointer to ai is stored (in _m_apVector), but
        // individual actions are not deleted when deleting joint action.
        // (multiple joint actions refer to the same indiv. action)
    }
    jai->Print();
    jai->DeleteIndividualActions();
    delete jai;
    }
    if(BEEP) cout <<"\nBeep!\a\n";
}
    //Mem. leak test - tests whether reinitialization of planning unit does not
    //leak
if(0){    
    int i=0;
    while(i < 100000)
    {
        JESPe.SetHorizon(10);
        i++;
    }
}    
    //Model (problem) creation mem. leak test
    //doesn't leak 2006-07-28
if(0){    
    int i=0;
    while(i < 100000) //comment out what you don't want to test.
    {
        delete ( new MultiAgentDecisionProcessDiscrete(2,2) );
//    delete ( new DecPOMDPDiscrete(2,2) );
//    delete ( new JESPExhaustivePlanner(3));
//    delete ( new ProblemDecTiger() );
        cout <<endl<< "-----------------starting new-----------------"<<endl;
    //    TestConstructActions* test_p_a =  new TestConstructActions();
        ProblemDecTiger* mt_problem_p = new ProblemDecTiger();
        JESPExhaustivePlanner* mt_jesp_p = new JESPExhaustivePlanner(3, 
            mt_problem_p);

        cout <<endl<< "-----------------ending new-----------------"<<endl;
        cout <<endl<< "-----------------starting delete-----------------"<<endl;
    //    delete test_p_a;        
        delete mt_jesp_p;
        delete mt_problem_p;
        cout <<endl<< "-----------------ending delete-----------------"<<endl;
        i++;
    }
}

#if 0
{ //bruteforce plan   h 3 
    BruteForceSearchPlanner bfs;
    bfs.Test();
    bfs.SetHorizon(3);
    cout << "#joint policies = "<<(int)bfs.GetNrJointPolicies()<<endl;
    bfs.Plan();
    double bfsVoptimal3 = bfs.GetExpectedReward();
    cout << endl<<">>> h=3 -> Optimal value = " << bfsVoptimal3 <<" <<<"<<endl;
}
#endif


}// end of TestModelCreation
  
void TestBruteForceSearch(const ArgumentHandlers::Arguments& args){
    int h = 2;
    ProblemDecTiger pdt;
    BruteForceSearchPlanner bfs(h, &pdt);
    cout << "---------------------------------------"<<endl;
    cout << "-BruteForceSearch Plan()-horizon="<<h<<"-----"<<endl;
    cout << "---------------------------------------"<<endl;
    bfs.Plan();
    cout << "\n---------------------------------------"<<endl;
    cout << "Max. Expected value: " << bfs.GetExpectedReward() <<"\n\n"<<endl;
}//endof TestBruteForceSearch




#include "QMDP.h"
#include "QPOMDP.h"
#include "QBG.h"
#include "GeneralizedMAAStarPlanner.h"
#include "GMAA_MAAstarClassic.h"
#include "GMAA_kGMAA.h"
#include "BGIP_SolverCreator_AM.h"
void TestGMAA(const ArgumentHandlers::Arguments& args)
{
    ProblemDecTiger pdt;
//    GeneralizedMAAStarPlanner gmaa(2, &pdt);

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    GMAA_kGMAA gmaa_em(new BGIP_SolverCreator_AM<JointPolicyPureVector>(), 2, &pdt,&params);

    QMDP qmdp = QMDP(&gmaa_em);
    qmdp.Compute();
    gmaa_em.SetQHeuristic(&qmdp);
    
    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_em.Print()---------------------"<<endl;
    gmaa_em.Print();

    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_em.Plan()- QMDP - horizon=2------"<<endl;
    gmaa_em.SetSeed(time(0));
    double V = -DBL_MAX;
    gmaa_em.Plan();
    V = gmaa_em.GetExpectedReward();
    if (!args.testMode)
        cout << "\napprox. value=" << V << endl;
    
    QPOMDP qpomdp = QPOMDP(&gmaa_em);
    qpomdp.Compute();
    gmaa_em.SetQHeuristic(&qpomdp);
    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_em.Plan()- QPOMDP - horizon=2----"<<endl;
    gmaa_em.Plan();
    V = gmaa_em.GetExpectedReward();
    if (!args.testMode)
        cout << "\napprox. value="<< V << endl;

    QBG qbg = QBG(&gmaa_em);
    qbg.Compute();
    gmaa_em.SetQHeuristic(&qbg);
    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_em.Plan()- QBG - horizon=2----"<<endl;
    gmaa_em.Plan();
    V = gmaa_em.GetExpectedReward();
    if (!args.testMode)
        cout << "\napprox. value="<< V << endl;
    
//    gmaa_em.ClearActionObservationHistories();
//    gmaa_em.ClearJointActionObservationHistories();       

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
    
    GMAA_MAAstarClassic gmaa_maa(2, &pdt,&params);
//    bool calculateJointBeliefs = true;
//    gmaa_maa.CreateActionObservationHistories();
//    gmaa_maa.CreateJointActionObservationHistories(calculateJointBeliefs);    
//    QMDP qmdp = QMDP(gmaa_maa);
    qmdp.SetPU(&gmaa_maa);
    qmdp.Compute();
    gmaa_maa.SetQHeuristic(&qmdp);
    
    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_maa.Print()---------------------"<<endl;
    gmaa_maa.Print();

    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_maa.Plan()- QMDP - horizon=2----------------"<<endl;
    gmaa_maa.SetSeed(time(0));
//    double V = -DBL_MAX;
    V = gmaa_maa.GetExpectedReward();
    if (!args.testMode)
        cout << "\nexact. value="<< V << endl;
//    gmaa_maa.ClearActionObservationHistories();
//    gmaa_maa.ClearJointActionObservationHistories();    

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
//    gmaa_maa.SetHorizon(3);
//    gmaa_maa.SetInitialized(true);
    GMAA_MAAstarClassic gmaa_maa3(3, &pdt, &params);
//    gmaa_maa3.CreateActionObservationHistories();
//    gmaa_maa3.CreateJointActionObservationHistories(calculateJointBeliefs);    

    qbg.SetPU(&gmaa_maa3);
    qbg.Compute();
    gmaa_maa3.SetQHeuristic(qbg);    
    cout << "---------------------------------------"<<endl;
    cout << "-gmaa_maa3.Print()---------------------"<<endl;
    gmaa_maa3.Print();
    cout << "-gmaa_maa.Plan()- QMDP - horizon=3----------------"<<endl;
    gmaa_maa3.Plan();
    V = gmaa_maa3.GetExpectedReward();
    if (!args.testMode)
        cout << "\nexact. value="<< V << endl;
//    gmaa_maa3.ClearActionObservationHistories();
//    gmaa_maa3.ClearJointActionObservationHistories();    


    
    return;
} // end of TestGMAA()

class TestPU : public PlanningUnitMADPDiscrete
{
    public: 
    void Plan(){}  
    double GetExpectedReward(){return 0.0;}
    boost::shared_ptr<JointPolicy> GetJointPolicy(){return boost::shared_ptr<JointPolicy>();}
    TestPU(
        PlanningUnitMADPDiscreteParameters params,
        size_t horizon=3, 
        MultiAgentDecisionProcessDiscrete* p=0
        ) 
        : PlanningUnitMADPDiscrete(horizon, p, &params)
    {}

};

void  TestPUMADP_histOptions(const ArgumentHandlers::Arguments& args)
{
    int h = 3;
    PlanningUnitMADPDiscreteParameters params;

    cout << "TestPUMADP_histOptions "<<endl;
    cout << "---------------------- "<<endl;
    ProblemDecTiger pdt;
   params.SetComputeAll(true);
    TestPU pu_all (params, h, &pdt);
    cout << "->Planning Unit with everything generated"<< endl;
    cout << "-----------------------------------------"<< endl;
    pu_all.Print();

    params.SetComputeAll(false);
    TestPU pu (params, h, &pdt);
    cout << "->Planning Unit with nothing generated"<< endl;
    cout << "--------------------------------------"<< endl;
    pu.Print();
    cout << "TestPUMADP_histOptions ended "<<endl<<endl<<endl<<endl<<endl;

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
}

void TestPUMADP_jointToIndHistIndices(const ArgumentHandlers::Arguments& args)
{
    int h = 3;

    cout << "TestPUMADP_jointToIndHistIndices"<<endl;
    cout << "---------------------- "<<endl;
    ProblemDecTiger pdt;
    PlanningUnitMADPDiscreteParameters params;

    params.SetComputeAll(true);
    TestPU pu_a (params, h, &pdt);
    params.SetComputeAll(false);
    TestPU pu_n (params, h, &pdt);
    cout << "# joint observation histories = "<< 
        pu_a.GetNrJointObservationHistories() << " = " <<
        pu_n.GetNrJointObservationHistories() << endl;

    for(Index johI=0; johI< pu_a.GetNrJointObservationHistories(); johI++)
    {
        cout << "joh Index:"<<johI<<", individual ref:";
        const vector<Index>& v_a = 
            pu_a.JointToIndividualObservationHistoryIndicesRef(johI);
        PrintVectorCout(v_a);
        cout <<", individual calc:";
        const vector<Index>& v_n = 
            pu_n.JointToIndividualObservationHistoryIndices(johI);
        PrintVectorCout(v_n);
        cout << endl;

    }

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
}

#include "JointBeliefSparse.h"
#include "JointBelief.h"
void TestJointBeliefSparse(const ArgumentHandlers::Arguments& args)
{
    ProblemDecTiger pdt;
    int h = 10;
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(false);
    TestPU pu(params,h,&pdt);

    JointBelief bf(*pu.GetMADPDI()->GetISD());
    JointBeliefSparse bs(*pu.GetMADPDI()->GetISD());
    bf.Print();
    bs.Print();
    for(int t=0;t!=h;++t)
    {
        bf.Update(*pu.GetMADPDI(), 0, 0);
        bf.Print();
        cout << endl;
        bs.Update(*pu.GetMADPDI(), 0, 0);
        bs.Print();
        cout << endl;
    }

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
}

#include "NullPlanner.h"
#include "MDPValueIteration.h"
#include "JointBelief.h"
#include "JointBeliefSparse.h"
#include "BeliefIteratorGeneric.h"
void TestBeliefIterators(const ArgumentHandlers::Arguments& args)
{
    ProblemDecTiger pdt;
    NullPlanner np(4,&pdt);
    MDPValueIteration vi(np);
    vi.Plan();

    JointBelief jb(2);
    jb.Set(0,0.8);
    jb.Set(1,0.2);

    JointBeliefSparse jbs(2);
    jbs.Set(0,0.8);
    jbs.Set(1,0.2);

    double Qok=0, QiteratorDense=0, QiteratorSparse=0;
    for(Index s=0;s!=np.GetNrStates();++s)
        Qok+=jb[s]*vi.GetQ(s,0);

    BeliefIteratorGeneric it=jb.GetIterator();
    do QiteratorDense+=it.GetProbability() * vi.GetQ(it.GetStateIndex(),0);
    while(it.Next());

    BeliefIteratorGeneric its=jbs.GetIterator();
    do QiteratorSparse+=its.GetProbability() * vi.GetQ(its.GetStateIndex(),0);
    while(its.Next());

    cout << Qok << " " << QiteratorDense << " " << QiteratorSparse << endl;

    if (!args.testMode)
    {
        cout << "press <enter> to continue..."<<endl;
        cin.get();
    }
}
