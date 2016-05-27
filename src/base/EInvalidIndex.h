/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _EINVALIDINDEX_H_
#define _EINVALIDINDEX_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "E.h"

/// EInvalidIndex represents an invalid index exception.
class EInvalidIndex : public E 
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.    
    /// Constructor with a C-style string
    EInvalidIndex(const char* arg):E(arg) {}
    /// Constructor with an STL string
    EInvalidIndex(std::string arg):E(arg) {}
    /// Constructor with an STL stringstream
    EInvalidIndex(const std::stringstream& arg) : E(arg) {}

};


#endif /* !_EINVALIDINDEX_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
