/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
