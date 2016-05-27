/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */


using namespace std;

#include "BGIP_SolverType.h"

std::string BGIP_SolverType::SoftPrint(BGIP_Solver_t type)
{
    return BGIP_SolverNames[type] ;
}


