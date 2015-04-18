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

#include "BGIP_SolverCreator_MP.h"

using namespace std;

/*
//Default constructor
BGIP_SolverCreator_MP::BGIP_SolverCreator_MP()
{
}
//Copy constructor.    
BGIP_SolverCreator_MP::BGIP_SolverCreator_MP(const BGIP_SolverCreator_MP& o) 
{
}
//Destructor
BGIP_SolverCreator_MP::~BGIP_SolverCreator_MP()
{
}
//Copy assignment operator
BGIP_SolverCreator_MP& BGIP_SolverCreator_MP::operator= (const BGIP_SolverCreator_MP& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/

#if 0
string BGIP_SolverCreator_MP::SoftPrint() const
{
    stringstream ss;
    ss << "BGIP_SolverCreator_MP object with _m_maxiter="<<_m_maxiter <<
        ", _m_updateType=" << _m_updateType <<
        ", _m_verbose="<<_m_verbose <<", _m_damping="<< _m_damping <<
        ", _m_nrSolutions="<<_m_nrSolutions<<", _m_nrRestarts="<<
        _m_nrRestarts<< endl;
    return (ss.str());
}
#endif
