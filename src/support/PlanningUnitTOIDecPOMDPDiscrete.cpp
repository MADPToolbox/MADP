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

#include "PlanningUnitTOIDecPOMDPDiscrete.h"

using namespace std;

//Default constructor
PlanningUnitTOIDecPOMDPDiscrete::PlanningUnitTOIDecPOMDPDiscrete(
    size_t horizon,
    TOIDecPOMDPDiscrete* p,
    const PlanningUnitMADPDiscreteParameters* params
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon, p, params),
    _m_TOIDecPOMDPDiscrete(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitTOIDecPOMDPDiscrete(PlanningUnitMADPDiscreteParameters params, size_t horizon, DecPOMDPDiscreteInterface* p)  called" << endl;
    if(p!=0)
        SanityCheck();
}
/*
PlanningUnitTOIDecPOMDPDiscrete::PlanningUnitTOIDecPOMDPDiscrete(
    size_t horizon,
    TOIDecPOMDPDiscrete* p
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon,p),
    _m_TOIDecPOMDPDiscrete(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitTOIDecPOMDPDiscrete(size_t horizon, DecPOMDPDiscreteInterface* p)  called" << endl;
    if(p!=0)
        SanityCheck();
}
*/
void PlanningUnitTOIDecPOMDPDiscrete::SetProblem(TOIDecPOMDPDiscrete* p)
{
    if(p == _m_TOIDecPOMDPDiscrete)
        return;
    _m_TOIDecPOMDPDiscrete=p;
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

