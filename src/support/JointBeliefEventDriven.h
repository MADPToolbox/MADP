/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _JOINTBELIEFEVENTDRIVEN_H_
#define _JOINTBELIEFEVENTDRIVEN_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "Belief.h"
#include "JointBeliefInterface.h"

class MultiAgentDecisionProcessDiscreteInterface; //forward declaration to avoid including each other

/**
 * \brief JointBeliefEventDriven stores a joint belief, represented as a regular
 * (dense) vector of doubles.
 */
class JointBeliefEventDriven : virtual public JointBeliefInterface,
                               virtual public Belief
{
private:    
    int _m_falseNegativeObs;
protected:
    
public:        
        
    /// Constructor which sets the \a size of the joint belief.
    JointBeliefEventDriven(size_t size=0, int falseNegativeObs=-1);

    /// Constructor which copies \a belief in this joint belief.
    JointBeliefEventDriven(const std::vector<double> &belief, int falseNegativeObs=-1);

    /// Constructor which copies \a belief in this joint belief.
    JointBeliefEventDriven(const JointBeliefInterface &belief, int falseNegativeObs=-1);
    JointBeliefEventDriven(const StateDistribution& belief, int falseNegativeObs=-1);

    /// Destructor
    ~JointBeliefEventDriven();

    JointBeliefEventDriven& operator= (const JointBeliefEventDriven& o);
    JointBeliefInterface& operator= (const JointBeliefInterface& o);

    double Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                  Index lastJAI, Index newJOI);

    /// Returns a pointer to a copy of this class.
    virtual JointBeliefEventDriven* Clone() const
        { return new JointBeliefEventDriven(*this); }


};

#endif /* !_JOINTBELIEFEVENTDRIVEN_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
