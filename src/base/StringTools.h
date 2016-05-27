/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _STRINGTOOLS_H_
#define _STRINGTOOLS_H_ 1

/* the include directives */
#include "Globals.h"
#include <string>

/** \brief StringTools is a namespace that contains utility functions for
 * strings.
 * */
namespace StringTools 
{
    std::string Append(const std::string& ioString, int inValue);
    std::string Trim(const std::string& ioString);
};


#endif /* !_STRINGTOOLS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
