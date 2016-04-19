/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _EDEADLINE_H_
#define _EDEADLINE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "E.h"

/// EDeadline represents a deadline exceeded expection.
class EDeadline : public E 
{
    private:    
    
    protected:
    
    public:

    double _m_expectedTimeForCompletion;


    // Constructor, destructor and copy assignment.    
    /// Constructor with a C-style string
    EDeadline(const char* arg, double expectedTimeForCompletion=0):
        E(arg),
        _m_expectedTimeForCompletion(expectedTimeForCompletion)
        {}
    /// Constructor with an STL string
    EDeadline(std::string arg, double expectedTimeForCompletion=0):
        E(arg),
        _m_expectedTimeForCompletion(expectedTimeForCompletion)
        {}
    /// Constructor with an STL stringstream
    EDeadline(const std::stringstream& arg, double expectedTimeForCompletion=0) :
        E(arg),
        _m_expectedTimeForCompletion(expectedTimeForCompletion)
        {}

};


#endif /* !_EDEADLINE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
