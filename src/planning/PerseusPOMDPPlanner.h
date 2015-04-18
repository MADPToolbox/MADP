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
#ifndef _PERSEUSPOMDPPLANNER_H_
#define _PERSEUSPOMDPPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorPOMDP.h"
#include "PerseusStationary.h"

/**PerseusPOMDPPlanner implements the Perseus planning algorithm for
 * POMDPs.  */
class PerseusPOMDPPlanner : public AlphaVectorPOMDP,
                            public PerseusStationary
{
private:    

    /// Compute a Perseus backup stage.
    ValueFunctionPOMDPDiscrete 
    BackupStage(const BeliefSet &S,
                const ValueFunctionPOMDPDiscrete &V);

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusPOMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    ~PerseusPOMDPPlanner();

    void Plan();

    virtual std::string SoftPrintBrief() const { return("PerseusPOMDP"); }

};


#endif /* !_PERSEUSPOMDPPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
