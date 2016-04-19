/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#ifndef  _BG_SOLVER_TYPE_H_
#define  _BG_SOLVER_TYPE_H_ 1

#include <string>

namespace BGIP_SolverType {

    // ordering: first BGIP, then BGCG, then the random at the end
    enum BGIP_Solver_t {
        BFS, AM, CE, 
        MaxPlus,
        BnB,
        CGBG_MaxPlus,
        NDP,
        Random,
    };
    const std::string BGIP_SolverNames[] = {
        "BGIP-BFS",
        "BGIP-AM",
        "BGIP-CE",
        "BGIP-MP",
        "BGIP-BnB",
        "BGCG-MP",
        "BGCG-NDP",
        "BGIP-Random",
    };
    std::string SoftPrint(BGIP_Solver_t type);
    const size_t NUMBER_OF_BGIP_SOLVER_TYPES = 8;

}

#endif /* !_BG_SOLVER_TYPE_H_*/
