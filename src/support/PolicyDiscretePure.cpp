/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
