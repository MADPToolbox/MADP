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
#ifndef _JOINTBELIEFINTERFACE_H_
#define _JOINTBELIEFINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BeliefInterface.h"

class MultiAgentDecisionProcessDiscreteInterface; //forward declaration to avoid including each other

/// JointBeliefInterface represents an interface for joint beliefs.
class JointBeliefInterface : virtual public BeliefInterface
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    JointBeliefInterface(){};

    /// Destructor.
    virtual ~JointBeliefInterface(){};

    /**\brief assignment operator.
     *
     * This must be implemented by the derived class (with this prototype).
     *
     * For now, this function is purely abstract. Might there be some 
     * members added to this (base) class, then an implementation could
     * be made. This should then be called using
     *          PolicyPoolInterface::operator=(o)
     * from the copy assignment operator of the derived class. See also
     * http://www.icu-project.org/docs/papers/cpp_report/the_assignment_operator_revisited.html .
     */    
    virtual JointBeliefInterface& operator= (const JointBeliefInterface& o)=0;

    /// Performs a joint belief update on this joint belief.
    /**Performs the belief update. I.e., given the current joint belief b
     * (which is represented by \e this), 
     * we compute P(s'|b, a, o) for all new states s'.
     * This function updates this belief (alter this object).
     * Returned is the normalization factor P(o|b,a), which is also to 
     * probability that this new history (belief) occurs given the previous 
     * one.
     */
    virtual double Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                          Index lastJAI, Index newJOI) = 0;

    /// Returns a pointer to a copy of this class.
    virtual JointBeliefInterface* Clone() const = 0;

};


#endif /* !_JOINTBELIEFINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
