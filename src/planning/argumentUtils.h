/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

/* Only include this header file once. */
#ifndef _ARGUMENTUTILS_H_
#define _ARGUMENTUTILS_H_ 1

/* the include directives */
#include "Globals.h"
 
class DecPOMDPDiscreteInterface;
class FactoredDecPOMDPDiscreteInterface;
class QFunctionJAOHInterface;
class PlanningUnitDecPOMDPDiscrete;

#include "argumentHandlers.h"

/** \brief ArgumentUtils provides a way to get problem instantations 
 * directly from the command-line arguments. */
namespace ArgumentUtils 
{
    DecPOMDPDiscreteInterface* GetDecPOMDPDiscreteInterfaceFromArgs(
            const ArgumentHandlers::Arguments& args);
    
    FactoredDecPOMDPDiscreteInterface* 
        GetFactoredDecPOMDPDiscreteInterfaceFromArgs(
            const ArgumentHandlers::Arguments& args);

    QFunctionJAOHInterface* GetQheuristicFromArgs(
        const PlanningUnitDecPOMDPDiscrete* pu,
        const ArgumentHandlers::Arguments &args);

    QFunctionJAOHInterface* GetHybridQheuristicFromArgs(
        const PlanningUnitDecPOMDPDiscrete* pu,
        const ArgumentHandlers::Arguments &args);

}


#endif /* !_ARGUMENTUTILS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
