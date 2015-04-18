/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
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
