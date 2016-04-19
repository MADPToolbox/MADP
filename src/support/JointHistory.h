/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTHISTORY_H_
#define _JOINTHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "History.h"

///JointHistory represents a joint history, i.e., a history for each agent.
class JointHistory : public History
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    JointHistory(){};
    /// Destructor.
    virtual ~JointHistory(){};

};


#endif /* !_JOINTHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
