/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ALPHAVECTORPOMDP_H_
#define _ALPHAVECTORPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorPlanning.h"

class PlanningUnitDecPOMDPDiscrete;
class AlphaVector;



/**AlphaVectorPOMDP implements POMDP specific functionality for
 * alpha-vector based planning. */
class AlphaVectorPOMDP : virtual public AlphaVectorPlanning
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorPOMDP(const PlanningUnitDecPOMDPDiscrete* pu);
    AlphaVectorPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~AlphaVectorPOMDP();

    /// Computes the AlphaVector resulting from backing up JointBeliefInterface
    /// \a b with the back-projected vectors in \a G.
    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             const GaoVectorSet &G) const;
    
    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G) const;

};


#endif /* !_ALPHAVECTORPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
