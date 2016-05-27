/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _OBSERVATIONDISCRETE_H_
#define _OBSERVATIONDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <string>

#include "Globals.h"
#include "Observation.h"
#include "DiscreteEntity.h"

/// ObservationDiscrete represents discrete observations.
/** 
 * ObservationDiscrete is a class that represent observations in a
 * discrete observation set, which are identified by their index.  */
class ObservationDiscrete : public Observation,
                            public DiscreteEntity
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        ObservationDiscrete(Index i=INDEX_MAX,
                            const std::string &name=std::string("undefined"),
                            const std::string &description=std::string("undefined")) :
            Observation(name, description),
            DiscreteEntity(i){};

};

#endif /* !_OBSERVATIONDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
