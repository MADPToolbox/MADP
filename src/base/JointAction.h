/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTACTION_H_
#define _JOINTACTION_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

/// JointAction represents a joint action.
class JointAction
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.

    /// Destructor.
    virtual ~JointAction(){}

    /// Returns a pointer to a copy of this class.
    virtual JointAction* Clone() const = 0;

    virtual std::string SoftPrint() const = 0;
    virtual std::string SoftPrintBrief() const = 0;
    virtual void Print() const { std::cout << SoftPrint();}
    virtual void PrintBrief() const{ std::cout << SoftPrintBrief();}

};


#endif /* !_JOINTACTION_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
