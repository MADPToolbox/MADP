/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _INDIVIDUALHISTORY_H_
#define _INDIVIDUALHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "History.h"

///IndividualHistory represents a history for a single agent.
class IndividualHistory : public History
{
private:    
    
protected:
    
    ///The agent this history belongs to.
    Index _m_agentI;
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    IndividualHistory(Index agentI) : _m_agentI(agentI){};
    /// Destructor.
    virtual ~IndividualHistory(){};
    
};


#endif /* !_INDIVIDUALHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
