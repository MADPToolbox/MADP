/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#ifndef  _GMAATYPE_H_
#define  _GMAATYPE_H_ 1

#include <string>

namespace GMAAtype {

enum GMAA_t {MAAstar, kGMAA, FSPC, MAAstarClassic };

std::string SoftPrint(GMAA_t type);

}

#endif /* !_GMAATYPE_H_*/
