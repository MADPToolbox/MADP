/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
