/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PERSEUSWEIGHTEDPLANNER_H_
#define _PERSEUSWEIGHTEDPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorWeighted.h"
#include "PerseusBGPOMDPPlanner.h"

class CommModel;

/**PerseusWeightedPlanner implements the Perseus planning algorithm 
 * with a weighted BG/POMDP backup. */
class PerseusWeightedPlanner : public AlphaVectorWeighted,
                               public PerseusBGPOMDPPlanner
{
private:    

protected:

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusWeightedPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                           const QAVParameters& params);

    PerseusWeightedPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                           const CommModel& comm);

    PerseusWeightedPlanner(const PlanningUnitDecPOMDPDiscrete* pu);

    PerseusWeightedPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                           const QAVParameters& params);

    PerseusWeightedPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                           const CommModel& comm);

    PerseusWeightedPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    ~PerseusWeightedPlanner();

    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G,
                             const QFunctionsDiscrete& Q) const;

    virtual std::string SoftPrintBrief() const { return("PerseusWeighted"); }
    
};


#endif /* !_PERSEUSWEIGHTEDPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
