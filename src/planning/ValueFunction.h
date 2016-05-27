/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _VALUEFUNCTION_H_
#define _VALUEFUNCTION_H_ 1

/* the include directives */
#include <iostream>
#include "PlanningUnit.h"


/** \brief ValueFunction is a class that represents a value function
 * of a joint policy.
 *
 * As the value function also depends on the problem type (DecPOMDP, POSG,etc.)
 * this is an abstract base class.*/
class ValueFunction  
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.

    /// Destructor.
    virtual ~ValueFunction(){};

    ///Return a reference to the planning unit.
    virtual PlanningUnit* GetPU() const = 0;
};


#endif /* !_VALUEFUNCTION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
