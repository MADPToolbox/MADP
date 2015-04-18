/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
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
