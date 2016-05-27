/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ENOTCACHED_H_
#define _ENOTCACHED_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "E.h"

/// ENotCached represents an invalid index exception.
class ENotCached : public E 
{
    private:    
    
    protected:
    
    public:
    // Constructor, destructor and copy assignment.    
    /// Constructor with a C-style string
    ENotCached(const char* arg):E(arg) {}
    /// Constructor with an STL string
    ENotCached(std::string arg):E(arg) {}
    /// Constructor with an STL stringstream
    ENotCached(const std::stringstream& arg) : E(arg) {}

};


#endif /* !_ENOTCACHED_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
