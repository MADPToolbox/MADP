/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
