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

#include "ProblemType.h"

using namespace std;

namespace ProblemType 
{
    string SoftPrint(Problem_t t)
    {
        switch(t)
        {
            case(PARSE):return("Parsed problem");
            case(FF):   return("Fire Fighting");
            case(FFF):  return("Factored Fire Fighting");
            case(FFG):  return("Factored Fire Graph");
            case(DT):   return("Dec-Tiger");
            case(Aloha):   return("Aloha");
            case(DTcreak):   return("Dec-Tiger with creaks");
            default:    return("Unrecognized ProblemType !");
        }


    }
};

