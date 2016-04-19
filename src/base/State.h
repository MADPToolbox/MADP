/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _STATE_H_
#define _STATE_H_ 1

/* the include directives */
#include <iostream>
#include <string>

#include "NamedDescribedEntity.h"

/// State is a class that represent states.
class State : public NamedDescribedEntity
{
private:
protected:
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    State(const std::string &name=std::string("undefined"),
          const std::string &description=std::string("undefined")) :
        NamedDescribedEntity(name, description){};

};

#endif /* !_STATE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
