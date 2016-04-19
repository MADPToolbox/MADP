/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QPOMDP_H_
#define _QPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "QFunctionJAOHTree.h"


/**\brief QPOMDP is a class that represents the QPOMDP heuristic.
 *
 * It is associated with a PlanningUnitDecPOMDPDiscrete which it uses for things
 * as horizon, action-/observation(history) indices, etc.
 */
class QPOMDP : public QFunctionJAOHTree
{
private:

    /**Recursively compute the heuristic. This is called by Compute(). */
#if QFunctionJAOH_useIndices
    double ComputeRecursively(size_t time_step, 
                              LIndex joahI,
                              Index lastJAI);
#else    
    double ComputeRecursively(size_t time_step, 
                              JointActionObservationHistoryTree* jaoht, 
                              Index lastJAI);
#endif
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QPOMDP(const PlanningUnitDecPOMDPDiscrete* pu);
    QPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    
    // Destructor.
    ~QPOMDP();

    std::string SoftPrintBrief() const { return("QPOMDP"); }

};


#endif /* !_QPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
