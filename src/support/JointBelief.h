/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTBELIEF_H_
#define _JOINTBELIEF_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "Belief.h"
#include "JointBeliefInterface.h"

class MultiAgentDecisionProcessDiscreteInterface; //forward declaration to avoid including each other

/**
 * \brief JointBelief stores a joint belief, represented as a regular
 * (dense) vector of doubles.
 */
class JointBelief : virtual public JointBeliefInterface,
                    virtual public Belief
{
private:    
    
protected:
    
public:        
        
    /// Constructor which sets the \a size of the joint belief.
    JointBelief(size_t size=0);

    /// Constructor which copies \a belief in this joint belief.
    JointBelief(const std::vector<double> &belief);

    /// Constructor which copies \a belief in this joint belief.
    JointBelief(const JointBeliefInterface &belief);
    JointBelief(const StateDistribution& belief);

    /// Destructor
    ~JointBelief();

    JointBelief& operator= (const JointBelief& o);
    JointBeliefInterface& operator= (const JointBeliefInterface& o);

    double Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                  Index lastJAI, Index newJOI);

    /// Returns a pointer to a copy of this class.
    virtual JointBelief* Clone() const
        { return new JointBelief(*this); }


};

#endif /* !_JOINTBELIEF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
