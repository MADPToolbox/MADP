/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
