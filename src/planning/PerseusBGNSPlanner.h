/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
