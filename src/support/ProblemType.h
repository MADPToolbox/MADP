/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PROBLEMTYPE_H_
#define _PROBLEMTYPE_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief ProblemType is a enumeration of different problems.
 * 
 */
namespace ProblemType 
{
    ///These are the standard problem types
    /**Note: to be able to specify them as an argument on the command 
     * line, you also need to add them to the argumentHandler (in
     * argumentHandler.cpp).
     */
    enum Problem_t {PARSE, DT, FF, FFF, FFG, Aloha, DTcreak}; //FFG, DT, etc.

    std::string SoftPrint(Problem_t t);
};


#endif /* !_PROBLEMTYPE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
