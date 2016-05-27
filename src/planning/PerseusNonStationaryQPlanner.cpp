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

#include "PerseusNonStationaryQPlanner.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BeliefSetNonStationary.h"

using namespace std;

PerseusNonStationaryQPlanner::
PerseusNonStationaryQPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    PerseusNonStationary(pu)
{
}

PerseusNonStationaryQPlanner::
PerseusNonStationaryQPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    PerseusNonStationary(pu)
{
}

//Destructor
PerseusNonStationaryQPlanner::~PerseusNonStationaryQPlanner()
{
}

void PerseusNonStationaryQPlanner::Plan()
{
    if(_m_computeVectorForEachBelief)
        PlanAll();
    else
        PlanSampling();
}

void PerseusNonStationaryQPlanner::PlanAll()
{
    PlanLeadIn();

    QFunctionsDiscreteNonStationary Q(GetPU()->GetHorizon());
    QFunctionsDiscrete Qt,Qt1;
    // records the value of each belief in the belief set
    vector<double> VB,VBnew;
    
    VB=BeliefValue::GetValues(_m_beliefs->Get(GetPU()->GetHorizon()),Qt);

    int lastTimestep=GetPU()->GetHorizon();
    for(int t=lastTimestep;t>=0;--t)
    {
        // print out some info
        PlanStartOfIteration(t,VB,Qt);

        // the real thing: compute the next stage value function
        if(t==lastTimestep)
            Qt=GetInitialQFunctions();
        else
        {
            Qt=BackupStageAll(_m_beliefs->Get(t),Qt1);

            // compute the maximum difference in the values for all
            // beliefs: for the convergence test
            VBnew=BeliefValue::GetValues(_m_beliefs->Get(t),Qt);
            
            VB=VBnew;
            Q.at(t)=Qt;
        }

        Qt1=Qt;
        PlanEndOfIteration();
    }

    StoreValueFunction(Q);
    if(!_m_dryrun)
        ExportValueFunction(_m_valueFunctionFilename);
            
    JointBeliefInterface* jb0 = GetPU()->GetNewJointBeliefFromISD();
    cout << GetIdentification() << ": final Vjb0 = "
         << BeliefValue::GetValue( *jb0, Q.at(0)) <<  endl;
    delete jb0;

    PlanLeadOut();
}

void PerseusNonStationaryQPlanner::PlanSampling()
{
    PlanLeadIn();

    QFunctionsDiscreteNonStationary Q(GetPU()->GetHorizon());
    QFunctionsDiscrete Qt,Qt1;
    // records the value of each belief in the belief set
    vector<double> VB,VBnew;
    

    int lastTimestep=GetPU()->GetHorizon();
    for(int t=lastTimestep;t>=0;--t)
    {
        // the real thing: compute the next stage value function
        if(t==lastTimestep)
        {
            Qt=GetInitialQFunctions();
            Qt1=Qt;
        }
        else
        {
            VB=BeliefValue::GetValues(_m_beliefs->Get(GetPU()->GetHorizon()),Qt);
            bool done=false;
            int iter=0;                      
            while(!done)
            {
                // print out some info
                PlanStartOfIteration(t,VB,Qt);

                Qt=BackupStageSampling(_m_beliefs->Get(t),Qt1);

                // compute the maximum difference in the values for all
                // beliefs: for the convergence test
                VBnew=BeliefValue::GetValues(_m_beliefs->Get(t),Qt);
            
                VB=VBnew;
                iter++;
                Qt1=Qt;

                // test for convergence
                if(CheckConvergence(VB,VBnew,iter))
                    done=true;
               
                PlanEndOfIteration();
            }
            Q.at(t)=Qt;
        }
    }

    StoreValueFunction(Q);
    if(!_m_dryrun)
        ExportValueFunction(_m_valueFunctionFilename);
            
    JointBeliefInterface* jb0 = GetPU()->GetNewJointBeliefFromISD();
    cout << GetIdentification() << ": final Vjb0 = "
         << BeliefValue::GetValue( *jb0, Q.at(0)) <<  endl;
    delete jb0;

    PlanLeadOut();
}
