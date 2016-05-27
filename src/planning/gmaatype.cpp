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


using namespace std;

#include "gmaatype.h"
#include "E.h"

std::string GMAAtype::SoftPrint(GMAA_t type)
{
    switch(type)
    {
    case MAAstar:
        return("MAAstar");
        break;
    case FSPC:
        return("FSPC");
        break;
    case kGMAA:
        return("kGMAA");
        break;
    case MAAstarClassic:
        return("MAAstarClassic");
        break;
    }

    throw(E("GMAAtype::SoftPrint invalid type"));

    return("INVALIDQMAATYPE");
}
