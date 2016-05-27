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

#include "MDPSolver.h"
#include <float.h>
#include <fstream>
#include <limits.h>
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointBeliefInterface.h"
#include "JointAction.h"
#include "State.h"
#include "BeliefIteratorGeneric.h"

using namespace std;

//Destructor
MDPSolver::~MDPSolver()
{
}

double MDPSolver::GetQ(Index time_step, const JointBeliefInterface& jb,
                       Index jaI) const
{
    double Q = 0.0;
#if USE_BeliefIteratorGeneric
    BeliefIteratorGeneric it=jb.GetIterator();
    do Q+=it.GetProbability() * GetQ(time_step,it.GetStateIndex(),jaI);
    while(it.Next());
#else
    for(Index sI=0; sI < jb.Size(); sI++)
        Q += jb.Get(sI) * GetQ(time_step,sI,jaI);
#endif
    return(Q);
}

double MDPSolver::GetQ(const JointBeliefInterface& jb,
                       Index jaI) const
{
    double Q = 0.0;
#if USE_BeliefIteratorGeneric
    BeliefIteratorGeneric it=jb.GetIterator();
    do Q+=it.GetProbability() * GetQ(it.GetStateIndex(),jaI);
    while(it.Next());
#else
    for(Index sI=0; sI < jb.Size(); sI++)
        Q += jb.Get(sI) * GetQ(sI,jaI);
#endif

    return(Q);
}

void MDPSolver::Print() const
{
    size_t horizon = GetPU()->GetHorizon();
    size_t nrS = GetPU()->GetNrStates();
    size_t nrJA =  GetPU()->GetNrJointActions();

    cout << "States: ";
    for(Index sI = 0; sI < nrS; sI++)
        cout << _m_pu->GetState(sI)->SoftPrintBrief() << " ";
    cout << endl;
        
    if(horizon!=MAXHORIZON)
    {
        for(size_t t = 0; t!=horizon; t++)
            for(Index jaI = 0; jaI < nrJA; jaI++)
            {            
                cout << "Q(t=" << t << ",:," << jaI << ") =\t";
                for(Index sI = 0; sI < nrS; sI++)
                    cout << " " << GetQ(t,sI,jaI);
                cout << " " << _m_pu->GetJointAction(jaI)->SoftPrintBrief();
                cout << endl;
            }
    }
    else
    {
        for(Index jaI = 0; jaI < nrJA; jaI++)
        {            
            cout << "Q(:," << jaI << ") =\t";
            for(Index sI = 0; sI < nrS; sI++)
                cout << " " << GetQ(sI,jaI);
            cout << " " << _m_pu->GetJointAction(jaI)->SoftPrintBrief();
            cout << endl;
        }
        
        cout << "Policy: ";
        for(Index sI = 0; sI < nrS; sI++)
        {
            cout << _m_pu->GetState(sI)->SoftPrintBrief() << "->";
            double q,v=-DBL_MAX;
            Index aMax=INT_MAX;

            for(Index jaI = 0; jaI < nrJA; jaI++)
            {
                q=GetQ(sI,jaI);
                if(q>v)
                {
                    v=q;
                    aMax=jaI;
                }
            }
            cout << _m_pu->GetJointAction(aMax)->SoftPrintBrief() << " ";
        }
        cout << endl;
    }
}

Index MDPSolver::GetMaximizingAction(Index time_step, Index sI)
{
    double q,v=-DBL_MAX;
    Index aMax=INT_MAX;

    for(size_t a=0;a!=GetPU()->GetNrJointActions();++a)
    {
        q=GetQ(time_step,sI,a);
        if(q>v)
        {
            v=q;
            aMax=a;
        }
    }

    return(aMax);
}

void MDPSolver::LoadQTable(const string &filename,
                           QTable &Q)
{
    QTable::Load(filename,
                 GetPU()->GetNrStates(),
                 GetPU()->GetNrJointActions(),
                 Q);
}

void MDPSolver::LoadQTables(const string &filename, int nrTables,
                            QTables &Qs)
{
    QTable::Load(filename,
                 GetPU()->GetNrStates(),
                 GetPU()->GetNrJointActions(),
                 nrTables,
                 Qs);
}
