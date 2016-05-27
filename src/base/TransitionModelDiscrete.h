/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TRANSITIONMODELDISCRETE_H_
#define _TRANSITIONMODELDISCRETE_H_ 1

/* the include directives */
#include "boost/numeric/ublas/matrix.hpp"
#include "Globals.h"
#include "TransitionModelDiscreteInterface.h"

/// TransitionModelDiscrete represents a discrete transition model.
class TransitionModelDiscrete : public TransitionModelDiscreteInterface
{
private:

    /// The number of states.
    int _m_nrStates;
    /// The number of joint actions.
    int _m_nrJointActions;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// Constructor with the dimensions of the transition model.
    TransitionModelDiscrete(int nrS = 1, int nrJA = 1);

    virtual ~TransitionModelDiscrete();    

    /// Sample a successor state.
    Index SampleSuccessorState(Index sI, Index jaI);
       
    /// Returns a pointer to a copy of this class.
    virtual TransitionModelDiscrete* Clone() const = 0;

    /// SoftPrints tabular transition model.
    std::string SoftPrint() const;
};

#endif /* !_TRANSITIONMODELDISCRETE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
