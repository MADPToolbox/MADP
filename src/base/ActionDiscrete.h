/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ACTIONDISCRETE_H_
#define _ACTIONDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <string>

#include "Globals.h"
#include "Action.h"
#include "DiscreteEntity.h"

/// ActionDiscrete represents discrete actions.

/** 
 * ActionDiscrete is a class that represent actions in a discrete
 * action set, which are identified by their index.  */
class ActionDiscrete : public Action,
                       public DiscreteEntity
{
private:    
    
protected:

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ActionDiscrete(Index i=INDEX_MAX,
                   const std::string &name=std::string("undefined"),
                   const std::string &description=std::string("undefined")) :
        Action(name, description),
        DiscreteEntity(i){};

};

#endif /* !_ACTIONDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
