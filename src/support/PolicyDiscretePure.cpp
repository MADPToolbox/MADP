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

#include "PolicyDiscretePure.h"

//Default constructor
PolicyDiscretePure::PolicyDiscretePure(
        const Interface_ProblemToPolicyDiscretePure* pu,
        PolicyGlobals::PolicyDomainCategory idc,
        Index agentI ) :
    PolicyDiscrete( pu , idc, agentI )
{
}
PolicyDiscretePure::PolicyDiscretePure(
        const I_PtPD_constPtr &pu,
        PolicyGlobals::PolicyDomainCategory idc,
        Index agentI ) :
    PolicyDiscrete( pu , idc, agentI )
{
}

//Copy constructor.    
PolicyDiscretePure::PolicyDiscretePure(const 
        PolicyDiscretePure& o) 
    :
    PolicyDiscrete( o )
{
}

double PolicyDiscretePure::GetActionProb(Index i, Index aI ) const
{
    if(GetActionIndex(i) == aI)
        return(1.0);

    return(0.0);
}
