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

/* Only include this header file once. */
#ifndef _PERSEUSNONSTATIONARYQPLANNER_H_
#define _PERSEUSNONSTATIONARYQPLANNER_H_ 1

/* the include directives */
#include "Globals.h"
#include "PerseusNonStationary.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief PerseusNonStationaryQPlanner is a Perseus planner that uses
 * non-stationary QFunctions. */
class PerseusNonStationaryQPlanner : public PerseusNonStationary
{
private:    

    virtual QFunctionsDiscrete
    BackupStageAll(const BeliefSet &S,
                const QFunctionsDiscrete &Q) const = 0;

    virtual QFunctionsDiscrete
    BackupStageSampling(const BeliefSet &S,
                        const QFunctionsDiscrete &Q) const = 0;

    void PlanAll();
    void PlanSampling();

protected:
    
public:

    PerseusNonStationaryQPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusNonStationaryQPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~PerseusNonStationaryQPlanner();

    void Plan();

};


#endif /* !_PERSEUSNONSTATIONARYQPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
