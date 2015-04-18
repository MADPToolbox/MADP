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
#ifndef _PERSEUSBGPOMDPPLANNER_H_
#define _PERSEUSBGPOMDPPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "PerseusQFunctionPlanner.h"

/**PerseusBGPOMDPPlanner implements the Perseus planning algorithm 
 * with for mixed BG/POMDP backups. */
class PerseusBGPOMDPPlanner : public PerseusQFunctionPlanner
{
private:    

    /// Compute a Perseus backup stage.
    QFunctionsDiscrete
    BackupStage(const BeliefSet &S,
                const QFunctionsDiscrete &V) const;

    QFunctionsDiscrete
    BackupStageAll(const BeliefSet &S,
                   const QFunctionsDiscrete &V) const;

    QFunctionsDiscrete
    BackupStageSamplingAlt(const BeliefSet &S,
                           const QFunctionsDiscrete &V) const;

protected:

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusBGPOMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusBGPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~PerseusBGPOMDPPlanner();

    virtual AlphaVector BeliefBackup(const JointBeliefInterface &b,
                                     Index a,
                                     const GaoVectorSet &G,
                                     const QFunctionsDiscrete &Q) 
        const = 0;

};


#endif /* !_PERSEUSBGPOMDPPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
