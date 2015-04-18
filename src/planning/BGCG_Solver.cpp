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

#include "BGCG_Solver.h"
#include "BayesianGameCollaborativeGraphical.h"

//Default constructor
BGCG_Solver::BGCG_Solver(const BGCG_constPtr &bg, size_t nrSolutions):
    BayesianGameIdenticalPayoffSolver(bg, nrSolutions),
    _m_bgcg(bg)
{
}
//Copy constructor.    
//BGCG_Solver::BGCG_Solver(const BGCG_Solver& o) 
//{
//}
//Destructor
BGCG_Solver::~BGCG_Solver()
{
}
//Copy assignment operator
BGCG_Solver& BGCG_Solver::operator= (const BGCG_Solver& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}

