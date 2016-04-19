/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _EOVERFLOW_H_
#define _EOVERFLOW_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "E.h"

/// EOverflow represents an integer overflow exception.
class EOverflow : public E 
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.    
    /// Constructor with a C-style string
    EOverflow(const char* arg):E(arg) {}
    /// Constructor with an STL string
    EOverflow(std::string arg):E(arg) {}
    /// Constructor with an STL stringstream
    EOverflow(const std::stringstream& arg) : E(arg) {}

};


#endif /* !_EOVERFLOW_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
