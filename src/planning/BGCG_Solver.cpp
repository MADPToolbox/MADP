/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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

