/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _OBSERVATIONMODEL_H_
#define _OBSERVATIONMODEL_H_ 1

/* the include directives */

#include <iostream>
#include "Globals.h"

/// ObservationModel represents the observation model in a decision process.
class ObservationModel
{
private:
    
protected:
    
public:
    /// default Constructor
    ObservationModel(){};

    /// Destructor.
    virtual ~ObservationModel(){}
    
    /// Returns a pointer to a copy of this class.
    virtual ObservationModel* Clone() const = 0;
   
    virtual std::string SoftPrint() const = 0;
    void Print() const
        {std::cout << SoftPrint();}
};

#endif /* !_OBSERVATIONMODEL_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
