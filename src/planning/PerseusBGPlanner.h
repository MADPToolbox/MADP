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
#ifndef _PERSEUSBGPLANNER_H_
#define _PERSEUSBGPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorBG.h"
#include "PerseusQFunctionPlanner.h"

/**PerseusBGPlanner implements the Perseus planning algorithm for
 * BGs.  */
class PerseusBGPlanner : public AlphaVectorBG,
                         public PerseusQFunctionPlanner
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
    BackupStageSampling(const BeliefSet &S,
                        const QFunctionsDiscrete &V) const;
    QFunctionsDiscrete
    BackupStageSamplingAlt(const BeliefSet &S,
                           const QFunctionsDiscrete &V) const;

    const BGBackupType _m_backupType;

protected:

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusBGPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                     const QAVParameters& params);

    PerseusBGPlanner(const PlanningUnitDecPOMDPDiscrete* pu);

    PerseusBGPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                     const QAVParameters& params);

    PerseusBGPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    ~PerseusBGPlanner();

    virtual std::string SoftPrintBrief() const { return("PerseusBG"); }
    
};


#endif /* !_PERSEUSBGPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
