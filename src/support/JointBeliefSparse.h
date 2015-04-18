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
#ifndef _JOINTBELIEFSPARSE_H_
#define _JOINTBELIEFSPARSE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointBeliefInterface.h"
#include "BeliefSparse.h"

class MultiAgentDecisionProcessDiscreteInterface; //forward declaration to avoid including each other

/// JointBeliefSparse represents a sparse joint belief.
class JointBeliefSparse : virtual public JointBeliefInterface,
                          virtual public BeliefSparse
{
private:    

    /// Slow version of Update(), if GetTransitionModelDiscretePtr() is 0.
    double UpdateSlow(const MultiAgentDecisionProcessDiscreteInterface &pu,
                      Index lastJAI, Index newJOI);

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// Default Constructor
    JointBeliefSparse();

    /// Constructor which sets the \a size of the joint belief.
    JointBeliefSparse(size_t size);
        
    /// Constructor which copies \a belief in this joint belief.
    JointBeliefSparse(const std::vector<double> &belief);

    /// Constructor which copies \a belief in this joint belief.
    JointBeliefSparse(const JointBeliefInterface &belief);
    JointBeliefSparse(const StateDistribution& belief);

    /// Destructor.
    ~JointBeliefSparse();

    // operators:
    using BeliefSparse::operator=;
    JointBeliefSparse& operator= (const JointBeliefSparse& o);
    JointBeliefInterface& operator= (const JointBeliefInterface& o);

    double Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                  Index lastJAI, Index newJOI);

    /// Returns a pointer to a copy of this class.
    virtual JointBeliefSparse* Clone() const
        { return new JointBeliefSparse(*this); }

};


#endif /* !_JOINTBELIEFSPARSE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
