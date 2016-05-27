/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _OBSERVATION_H_
#define _OBSERVATION_H_ 1

/* the include directives */
#include <iostream>
#include <string>

#include "NamedDescribedEntity.h"

/// Observation represents observations.
class Observation : public NamedDescribedEntity
{
    private:
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    Observation(const std::string &name=std::string("undefined"),
                const std::string &description=std::string("undefined")) :
        NamedDescribedEntity(name, description){};

};

#endif /* !_OBSERVATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
