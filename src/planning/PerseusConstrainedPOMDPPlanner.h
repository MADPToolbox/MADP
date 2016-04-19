/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PERSEUSCONSTRAINEDPOMDPPLANNER_H_
#define _PERSEUSCONSTRAINEDPOMDPPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorConstrainedPOMDP.h"
#include "PerseusQFunctionPlanner.h"

/// The PerseusConstrainedPOMDPPlanner is a Perseus variant which skips
/// action selection if the agent receives a "false-negative" observation,
/// which in practice means that the agent cannot react to an event which
/// it failed to detect.
class PerseusConstrainedPOMDPPlanner : public AlphaVectorConstrainedPOMDP,
                                       public PerseusQFunctionPlanner
{
private:    

    /// Compute a Perseus backup stage.
    QFunctionsDiscrete
    BackupStage(const BeliefSet &S,
                const QFunctionsDiscrete &Q) const;
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusConstrainedPOMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu, const QAVParameters& params);
    PerseusConstrainedPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu, const QAVParameters& params);
    /// Destructor.
    ~PerseusConstrainedPOMDPPlanner();

    //void Plan();

    virtual std::string SoftPrintBrief() const { return("PerseusPOMDP"); }
};


#endif /* !_PERSEUSCONSTRAINEDPOMDPPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
