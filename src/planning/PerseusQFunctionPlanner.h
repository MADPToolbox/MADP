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
#ifndef _PERSEUSQFUNCTIONPLANNER_H_
#define _PERSEUSQFUNCTIONPLANNER_H_ 1

/* the include directives */
#include "Globals.h"
#include "PerseusStationary.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief PerseusQFunctionPlanner is a Perseus planner that uses
 * QFunctions. */
class PerseusQFunctionPlanner : public PerseusStationary

{
private:    

    virtual QFunctionsDiscrete
    BackupStage(const BeliefSet &S,
                const QFunctionsDiscrete &V) const = 0;
    
protected:
    
public:

    PerseusQFunctionPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusQFunctionPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~PerseusQFunctionPlanner();

    void Plan();

};


#endif /* !_PERSEUSQFUNCTIONPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
