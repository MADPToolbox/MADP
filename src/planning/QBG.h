/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QBG_H_
#define _QBG_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "QFunctionJAOHTree.h"

class JointBelief;

/**\brief QBG is a class that represents the QBG heuristic.
 *
 * It is associated with a PlanningUnitDecPOMDPDiscrete which it uses for things
 * as horizon, action-/observation(history) indices, etc.
 */
class QBG : public QFunctionJAOHTree
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
    QBG(const PlanningUnitDecPOMDPDiscrete* pu);
    QBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    
    /// Destructor.
    ~QBG();

    //operators:
    
    //data manipulation (set) functions:
    
    /**Compute the heuristic. (after associated with an initialized 
     * PlanningUnitDecPOMDPDiscrete) */
    void ComputeNoCache();
    /**Recursively compute the heuristic. This is called by Compute(). */
    double ComputeRecursivelyNoCache(size_t time_step, Index jahI, 
                                     Index johI, const JointBelief &JB, 
                                     Index lastJAI);

    std::string SoftPrintBrief() const { return("QBG"); }

};


#endif /* !_QBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
