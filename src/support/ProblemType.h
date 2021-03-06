/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
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
    enum Problem_t {PARSE, DT, FF, FFF, FFG, FFGOpt, Aloha, AlohaOpt, DTcreak, TCP, Random}; //FFG, DT, etc.

    std::string SoftPrint(Problem_t t);
};


#endif /* !_PROBLEMTYPE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
