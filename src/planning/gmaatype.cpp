/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
