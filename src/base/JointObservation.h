/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JOINTOBSERVATION_H_
#define _JOINTOBSERVATION_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

/// JointObservation is represents joint observations.
class JointObservation
{
    private:    
    
    protected:
    
    public:

    /// Destructor.
    virtual ~JointObservation() {};

    /// Returns a pointer to a copy of this class.
    virtual JointObservation* Clone() const = 0;

    virtual std::string SoftPrint() const = 0;
    virtual std::string SoftPrintBrief() const = 0;
    virtual void Print() const { std::cout << SoftPrint();}
    virtual void PrintBrief() const { std::cout << SoftPrintBrief();}
};


#endif /* !_JOINTOBSERVATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
