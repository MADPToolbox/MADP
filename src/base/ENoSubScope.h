/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ENOSUBSCOPE_H_
#define _ENOSUBSCOPE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "E.h"

/// ENoSubScope represents an invalid index exception.
class ENoSubScope : public E 
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.    
    /// Constructor with a C-style string
    ENoSubScope(const char* arg):E(arg) {}
    /// Constructor with an STL string
    ENoSubScope(std::string arg):E(arg) {}
    /// Constructor with an STL stringstream
    ENoSubScope(const std::stringstream& arg) : E(arg) {}

};


#endif /* !_ENOSUBSCOPE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
