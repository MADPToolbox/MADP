/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "QFunctionJointBelief.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointBeliefInterface.h"

double QFunctionJointBelief::GetQ(Index jaohI, Index jaI) const
{
    JointBeliefInterface *b=GetPU()->GetJointBeliefInterface(jaohI);
    double q=GetQ(*b,jaI);
    delete b;
    return(q);
}
