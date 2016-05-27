/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _OBSERVATIONMODELDISCRETEINTERFACE_H_
#define _OBSERVATIONMODELDISCRETEINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "ObservationModel.h"

/// ObservationModelDiscreteInterface represents a discrete observation model.
class ObservationModelDiscreteInterface : public ObservationModel
{
private:    

protected:
    
public:
    /// Constructor with the dimensions of the observation model.
    ObservationModelDiscreteInterface(){};

    /// Destructor.
    virtual ~ObservationModelDiscreteInterface(){};
    
    /// Returns P(jo|ja,s')
    virtual double Get(Index ja_i, Index suc_s_i, Index jo_i) const = 0;
    virtual double Get(Index s_i, Index ja_i, Index suc_s_i, Index jo_i) const
          {return Get(ja_i, suc_s_i, jo_i); }

    //data manipulation funtions:
    /// Sets P(o|ja,s')
    /** Index jo_i, Index ja_i, Index suc_s_i, are indices of the
     * joint observation, taken joint action and resulting successor
     * state. prob is the probability. The order of events is ja, s',
     * o, so is the arg. list
     */
    virtual void Set(Index ja_i, Index suc_s_i, Index jo_i, double prob) = 0;
    virtual void Set(Index s_i, Index ja_i, Index suc_s_i, Index jo_i, double prob)
          {Set(ja_i, suc_s_i, jo_i, prob); }
        
    /// Returns a pointer to a copy of this class.
    virtual ObservationModelDiscreteInterface* Clone() const = 0;

};


#endif /* !_OBSERVATIONMODELDISCRETEINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
