/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _OBSERVATIONMODELDISCRETE_H_
#define _OBSERVATIONMODELDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "ObservationModelDiscreteInterface.h"

/// ObservationModelDiscrete represents a discrete observation model.
class ObservationModelDiscrete : public ObservationModelDiscreteInterface
{
private:    
    
    /// The number of states.
    int _m_nrStates;
    /// The number of joint actions.
    int _m_nrJointActions;    
    /// The number of joint observations
    int _m_nrJointObservations;

protected:
    
public:
    /// Constructor with the dimensions of the observation model.
    ObservationModelDiscrete(int nrS = 1, int nrJA = 1, int nrJO = 1);

    /// Destructor.
    virtual ~ObservationModelDiscrete();
    
    /// Sample a joint observation.
    Index SampleJointObservation(Index jaI, Index sucI);

    /// Sample a joint observation.
    Index SampleJointObservation(Index sI, Index jaI, Index sucI);

    /// Returns a pointer to a copy of this class.
    virtual ObservationModelDiscrete* Clone() const = 0;

    /// SoftPrints tabular observation model.
    std::string SoftPrint() const;
};


#endif /* !_OBSERVATIONMODELDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
