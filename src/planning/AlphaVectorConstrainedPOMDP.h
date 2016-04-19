/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ALPHAVECTORCONSTRAINEDPOMDP_H_
#define _ALPHAVECTORCONSTRAINEDPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorPlanning.h"

class PlanningUnitDecPOMDPDiscrete;
class AlphaVector;



/**AlphaVectorConstrainedPOMDP implements Constrained POMDP specific functionality for
 * alpha-vector based planning. */
class AlphaVectorConstrainedPOMDP : virtual public AlphaVectorPlanning
{
private:    
    
    ///The observation symbol which represents a false negative detection
    ///of an event
    int _m_falseNegativeObs;
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorConstrainedPOMDP(const PlanningUnitDecPOMDPDiscrete* pu, int falseNegativeObs);
    AlphaVectorConstrainedPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu, int falseNegativeObs);

    /// Destructor.
    virtual ~AlphaVectorConstrainedPOMDP();

    /// Computes the AlphaVector resulting from backing up JointBeliefInterface
    /// \a b with the back-projected vectors in \a G.
    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             const GaoVectorSet &G) const;
    
    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G) const;

    VectorSet BeliefBackupQ(const JointBeliefInterface &b,
                            const GaoVectorSet &G,
                            const ValueFunctionPOMDPDiscrete &V = ValueFunctionPOMDPDiscrete()) const;

    int GetFalseNegativeObs()
    {return _m_falseNegativeObs;}
};


#endif /* !_ALPHAVECTORCONSTRAINEDPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
