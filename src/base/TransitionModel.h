/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TRANSITIONMODEL_H_
#define _TRANSITIONMODEL_H_ 1

/* the include directives */

#include <iostream>
#include <string>
#include "Globals.h"

/// TransitionModel represents the transition model in a decision process.
class TransitionModel
{
private:

protected:
    
public:

    /// default Constructor
    TransitionModel(){};

    /// Destructor.
    virtual ~TransitionModel(){}
    
    /// Returns a pointer to a copy of this class.
    virtual TransitionModel* Clone() const = 0;

    virtual std::string SoftPrint() const = 0;
    void Print() const
        { std::cout << SoftPrint();}
};

#endif /* !_TRANSITIONMODEL_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
