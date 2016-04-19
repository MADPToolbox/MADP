/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QFUNCTION_H_
#define _QFUNCTION_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "QFunctionInterface.h"

/**\brief QFunction is an abstract base class containing nothing. 
 * It only groups together all Q-Functions.
 * */
class QFunction :
    virtual public QFunctionInterface
{
private:
    
protected:
    
public:

};


#endif /* !_QFUNCTION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
