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
