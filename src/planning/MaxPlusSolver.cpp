/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "MaxPlusSolver.h"

using namespace std;

//Default constructor
MaxPlusSolver::MaxPlusSolver(
        size_t maxiter,
        string updateType,
        int verbosity,
        double damping,
        size_t nrSolutions,
        size_t nrRestarts)
    :
    _m_maxiter(maxiter),
    _m_updateType(updateType),
    _m_verbosity(verbosity),
    _m_damping(damping),
    _m_nrSolutions(nrSolutions),
    _m_nrRestarts(nrRestarts)
{
}

/*
//Copy constructor.    
MaxPlusSolver::MaxPlusSolver(const MaxPlusSolver& o) 
{
}
//Destructor
MaxPlusSolver::~MaxPlusSolver()
{
}
//Copy assignment operator
MaxPlusSolver& MaxPlusSolver::operator= (const MaxPlusSolver& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/
