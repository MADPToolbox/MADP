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

#include "PerseusQFunctionPlanner.h"
#include "PlanningUnitDecPOMDPDiscrete.h"

using namespace std;

PerseusQFunctionPlanner::
PerseusQFunctionPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    PerseusStationary(pu)
{
}

PerseusQFunctionPlanner::
PerseusQFunctionPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    PerseusStationary(pu)
{
}

//Destructor
PerseusQFunctionPlanner::~PerseusQFunctionPlanner()
{
}

void PerseusQFunctionPlanner::Plan()
{
    PlanLeadIn();

    QFunctionsDiscrete Q0,Q1;
    // records the value of each belief in the belief set
    vector<double> VB,VBnew;
    
    // get initial value function
    Q1=GetInitialQFunctions();
    VB=BeliefValue::GetValues(*_m_beliefs,Q1);

    int iter=0;                      
    bool done=false;
    while(!done)
    {
        // print out some info
        PlanStartOfIteration(iter,VB,Q1);

        // the real thing: compute the next stage value function
        Q0=Q1;
        Q1=BackupStage(*_m_beliefs,Q0);

        // compute the maximum difference in the values for all
        // beliefs: for the convergence test
        VBnew=BeliefValue::GetValues(*_m_beliefs,Q1);
       
        // test for convergence
        if(CheckConvergence(VB,VBnew,iter))
            done=true;

        VB=VBnew;
        iter++;

        PlanEndOfIteration(Q1);
    }

    PlanLeadOut();
}
