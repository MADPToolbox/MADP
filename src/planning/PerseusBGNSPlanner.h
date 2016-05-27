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
#ifndef _PERSEUSBGNSPLANNER_H_
#define _PERSEUSBGNSPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorBG.h"
#include "PerseusNonStationaryQPlanner.h"

/**PerseusBGNSPlanner implements the Perseus planning algorithm for
 * BGs with non-stationary QFunctions.  */
class PerseusBGNSPlanner : public AlphaVectorBG,
                           public PerseusNonStationaryQPlanner
{
private:    

    QFunctionsDiscrete
    BackupStage(const BeliefSet &S,
                const QFunctionsDiscrete &Q) const;

    QFunctionsDiscrete
    BackupStageAll(const BeliefSet &S,
                   const QFunctionsDiscrete &Q) const;

    QFunctionsDiscrete
    BackupStageSampling(const BeliefSet &S,
                        const QFunctionsDiscrete &Q) const;

    const BGBackupType _m_backupType;

protected:

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusBGNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                       const QAVParameters& params);
    
    PerseusBGNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusBGNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                       const QAVParameters& params);
    
    PerseusBGNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    ~PerseusBGNSPlanner();

    virtual std::string SoftPrintBrief() const { return("PerseusBGNS"); }
    
};


#endif /* !_PERSEUSBGNSPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
