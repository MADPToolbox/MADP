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
