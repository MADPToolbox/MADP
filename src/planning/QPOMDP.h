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
