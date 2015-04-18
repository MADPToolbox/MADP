/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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


using namespace std;

#include "JESPtype.h"
#include "E.h"

std::string JESPtype::SoftPrint(JESP_t type)
{
    switch(type)
    {
    case JESPExhaustive:
        return("JESPExh");
        break;
    case JESPDP:
        return("JESPDP");
        break;
    }

    throw(E("JESPtype::SoftPrint invalid type"));

    return("INVALID_JESP_TYPE");
}
