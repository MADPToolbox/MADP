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
