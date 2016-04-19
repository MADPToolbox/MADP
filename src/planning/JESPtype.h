/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#ifndef  _JESPTYPE_H_
#define  _JESPTYPE_H_ 1

#include <string>

namespace JESPtype {

enum JESP_t {JESPExhaustive, JESPDP};

std::string SoftPrint(JESP_t type);

}

#endif /* !_JESPTYPE_H_*/
