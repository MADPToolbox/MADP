/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PERSEUSWEIGHTEDNSPLANNER_H_
#define _PERSEUSWEIGHTEDNSPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorWeighted.h"
#include "PerseusBGPOMDPNSPlanner.h"

class CommModel;

/**PerseusWeightedNSPlanner implements the Perseus planning algorithm 
 * with a weighted BG/POMDP backup. */
class PerseusWeightedNSPlanner : public AlphaVectorWeighted,
                                 public PerseusBGPOMDPNSPlanner
{
private:    

protected:

public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusWeightedNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                             const QAVParameters& params);

    PerseusWeightedNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                             const CommModel& comm);

    PerseusWeightedNSPlanner(const PlanningUnitDecPOMDPDiscrete* pu);

    PerseusWeightedNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                             const QAVParameters& params);

    PerseusWeightedNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                             const CommModel& comm);

    PerseusWeightedNSPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    ~PerseusWeightedNSPlanner();

    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G,
                             const QFunctionsDiscrete& Q) const;

    virtual std::string SoftPrintBrief() const { return("PerseusWeightedNS"); }
    
};


#endif /* !_PERSEUSWEIGHTEDNSPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
