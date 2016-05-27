/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PolicyDiscrete.h"
#include <stdlib.h>


Index PolicyDiscrete::SampleAction( Index i ) const
{
    size_t nrA = GetInterfacePTPDiscrete()->GetNrActions(_m_agentI);
    double randNr= ((double)rand()) / (RAND_MAX + 1.0);
    double sum=0;
    Index selected_a=0;

    for(Index a=0; a<nrA; a++)
    {
        sum+=GetActionProb(i, a);
        if(randNr<=sum)
        {
            selected_a=a;
            break;
        }
    }
    return(selected_a);
}
