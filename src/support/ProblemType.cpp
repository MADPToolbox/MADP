/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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

