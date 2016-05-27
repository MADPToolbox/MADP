/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _EPARSE_H_
#define _EPARSE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "E.h"

///EParse represents a parser exception.
class EParse : public E 
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.    
    /// Constructor with a C-style string
    EParse(const char* arg):E(arg) {}
    /// Constructor with an STL string
    EParse(std::string arg):E(arg) {}
    /// Constructor with an STL stringstream
    EParse(const std::stringstream& arg) : E(arg) {}
};


#endif /* !_EPARSE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
