/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
