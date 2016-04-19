/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PlanningUnitTOIDecPOMDPDiscrete.h"

using namespace std;

//Default constructor
PlanningUnitTOIDecPOMDPDiscrete::PlanningUnitTOIDecPOMDPDiscrete(
    const PlanningUnitMADPDiscreteParameters &params,
    size_t horizon,
    TOIDecPOMDPDiscrete* p
    ) :
    Referrer<TOIDecPOMDPDiscrete>(p),
    PlanningUnitDecPOMDPDiscrete(params,horizon,p) 
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitTOIDecPOMDPDiscrete(PlanningUnitMADPDiscreteParameters params, size_t horizon, DecPOMDPDiscreteInterface* p)  called" << endl;
    if(p!=0)
        SanityCheck();
}

PlanningUnitTOIDecPOMDPDiscrete::PlanningUnitTOIDecPOMDPDiscrete(
    size_t horizon,
    TOIDecPOMDPDiscrete* p
    ) :
    Referrer<TOIDecPOMDPDiscrete>(p),
    PlanningUnitDecPOMDPDiscrete(horizon,p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitTOIDecPOMDPDiscrete(size_t horizon, DecPOMDPDiscreteInterface* p)  called" << endl;
    if(p!=0)
        SanityCheck();
}

void PlanningUnitTOIDecPOMDPDiscrete::SetProblem(TOIDecPOMDPDiscrete* p)
{
    if(p == GetReferred())
        return;
    SetReferred(p);
#if 0
    //set (and initialize) the problem at PlanningUnitDecPOMDPDiscrete level:
    DecPOMDPDiscreteInterface* p2 = 
        static_cast<DecPOMDPDiscreteInterface*>(p);
#endif
    PlanningUnitDecPOMDPDiscrete::SetProblem(p);

    SanityCheck();
}

bool PlanningUnitTOIDecPOMDPDiscrete::SanityCheck() const
{
    bool sane=PlanningUnitDecPOMDPDiscrete::SanityCheck();

    return(sane);
}

