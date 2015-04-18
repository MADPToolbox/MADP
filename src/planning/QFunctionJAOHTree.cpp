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

#include "QFunctionJAOHTree.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BayesianGameIdenticalPayoff.h"
#include "JointBeliefInterface.h"
#include "BGIP_SolverBruteForceSearch.h"

using namespace std;

#define DEBUG_QHEUR_COMP_TREE 0

//Default constructor
QFunctionJAOHTree::
QFunctionJAOHTree(const PlanningUnitDecPOMDPDiscrete *pu) :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOH(pu)
{
    _m_initialized = false;
}

QFunctionJAOHTree::
QFunctionJAOHTree(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOH(pu)
{
    _m_initialized = false;
}

//Destructor
QFunctionJAOHTree::~QFunctionJAOHTree()
{
    DeInitialize();
}

void QFunctionJAOHTree::DeInitialize()
{
    _m_QValues.clear();
    _m_initialized=false;
}

void QFunctionJAOHTree::Initialize()
{
    _m_QValues.resize(GetPU()->GetNrJointActionObservationHistories(),
                      GetPU()->GetNrJointActions(),
                      false);
    _m_initialized = true;
}

void QFunctionJAOHTree::SetPU(const PlanningUnitDecPOMDPDiscrete* pu)
{
    DeInitialize();
    QFunctionJAOH::SetPU(pu);
}

void QFunctionJAOHTree::SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
{
    DeInitialize();
    QFunctionJAOH::SetPU(pu);
}

void QFunctionJAOHTree::Compute()
{
    if(!_m_initialized)
        Initialize();

    ComputeQ();
}

void QFunctionJAOHTree::Save(const string &filename) const
{
    QTable::Save(_m_QValues,filename);
}

void QFunctionJAOHTree::Load(const string &filename)
{
    if(_m_initialized)
        Initialize();
#if 0 // don't use this, it makes a copy (so you need twice the memory)

    _m_QValues=MDPSolver::LoadQTable(filename,
                                     GetPU()->
                                     GetNrJointActionObservationHistories(),
                                     GetPU()->GetNrJointActions());
#else
    QTable::Load(filename,
                 GetPU()->GetNrJointActionObservationHistories(),
                 GetPU()->GetNrJointActions(),
                 _m_QValues);
#endif
}

void QFunctionJAOHTree::ComputeQ()
{
    if(GetPU() == 0)
        throw E("QFunctionJAOHTree::ComputeQ - GetPU() returns 0; no PlanningUnit available!");

    size_t time_step = 0;
#if QFunctionJAOH_useIndices
#else    
    JointActionObservationHistoryTree* root = GetPU()->
        GetJointActionObservationHistoryTree(Globals::INITIAL_JAOHI);
#endif
    JointBeliefInterface* b0p = GetPU()->GetNewJointBeliefFromISD(); 
    JointBeliefInterface& b0 =  *b0p;
    
    bool last_t = false;
    if( (time_step + 1) == GetPU()->GetHorizon()) //unlikely, but possible
        last_t = true;

    if(DEBUG_QHEUR_COMP_TREE){cout << "QFunctionJAOHTree::Compute() called" << endl;}

    //in the first time_step t=0, there is no previous action and there is only
    //the empty observation action history.
    //Therefore we're going to construct a Bayesian game where there is only 1
    //type for each agent.
    vector<size_t> nrTypes = vector<size_t>(GetPU()->GetNrAgents(), 1);
    BGIP_sharedPtr bg_time_step=BGIP_sharedPtr(
        new BayesianGameIdenticalPayoff(GetPU()->GetNrAgents(), 
                                        GetPU()->GetNrActions(),
                                        nrTypes));

    size_t empty_jaohI = 0;
    bg_time_step->SetProbability(empty_jaohI, 1.0);
    for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
    {
        //calculate R(joah',newJA) - expected immediate reward for time_step
        double exp_imm_R = 0.0;
#if USE_BeliefIteratorGeneric
        BeliefIteratorGeneric it=b0.GetIterator();
        do exp_imm_R += it.GetProbability() *
            GetPU()->GetReward(it.GetStateIndex(), newJAI);
        while (it.Next());
#else
        for(Index sI=0; sI < GetPU()->GetNrStates(); sI++)
            exp_imm_R += (b0.Get(sI)) * GetPU()->GetReward(sI, newJAI);
#endif        
        //calculate Q(jaoh', newJA) =  R(joah',newJA) + exp. future R
        //  and the exp. future R = ComputeRecursively(t+1, root, newJA)
        double exp_fut_R = 0.0;
        if(!last_t)
#if QFunctionJAOH_useIndices
            exp_fut_R = ComputeRecursively( 1, Globals::INITIAL_JAOHI, newJAI);
#else    
            exp_fut_R = ComputeRecursively( 1, root, newJAI);
#endif
        double Q = exp_imm_R + GetPU()->GetDiscount() * exp_fut_R;
        _m_QValues(empty_jaohI,newJAI)=Q;
        bg_time_step->SetUtility(empty_jaohI, newJAI, Q);
    }//end for newJAI
    if(DEBUG_QHEUR_COMP_TREE)
    {
        cout << "QFunctionJAOHTree::ComputeQ() for..."<<endl<<
            " time_step=0  called, with ISD=";
        b0.Print();
        cout <<endl;
        bg_time_step->Print();
    }
    //solve this bayesian game
    BGIP_SolverBruteForceSearch<JointPolicyPureVector> bgs(bg_time_step);
    double v = bgs.Solve();
    if(DEBUG_QHEUR_COMP_TREE)
        cout << "QFunctionJAOHTree::ComputeQ() - Expected V(b0) = " << v << endl<< endl;
    delete b0p;
    return;
}
