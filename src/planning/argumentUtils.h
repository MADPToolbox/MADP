/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
