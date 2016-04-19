/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _AGENT_H_
#define _AGENT_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "NamedDescribedEntity.h"
#include "DiscreteEntity.h"


/// Agent represents an agent.
/**Agent is a class that represents an agent, which can be identified
 * by its index. */
class Agent : public NamedDescribedEntity,
              public DiscreteEntity
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    Agent(Index i=INDEX_MAX,
          const std::string &name=std::string("undefined"),
          const std::string &description=std::string("undefined")) :
        NamedDescribedEntity(name, description),
        DiscreteEntity(i){};

};

#endif /* !_AGENT_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
