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

#include "PartialJPDPValuePair.h"
#include "PartialJointPolicyDiscretePure.h"

using namespace std;

PartialJPDPValuePair::PartialJPDPValuePair(const PJPDP_sharedPtr &jp,
                                           double val) :
    PartialJointPolicyValuePair(val),
    _m_jpol(jp)
{
}                

//Copy constructor.    
PartialJPDPValuePair::PartialJPDPValuePair(const PartialJPDPValuePair& o) :
    PartialJointPolicyValuePair(o)
{
    _m_jpol = PJPDP_sharedPtr(o._m_jpol->Clone());
    _m_bgip_solver = o._m_bgip_solver;
    _m_bgcg_solver = o._m_bgcg_solver;    
    _m_bgip_solver_T = o._m_bgip_solver_T;
    _m_bgip_solver_TC = o._m_bgip_solver_TC;    
}

PartialJPDPValuePair::~PartialJPDPValuePair()
{
//    delete _m_jpol;
}

PartialJPDPValuePair* PartialJPDPValuePair::Clone() const
{
    return(new PartialJPDPValuePair(*this));
}

void PartialJPDPValuePair::CleanUpBGIPSolver()
{
}

string PartialJPDPValuePair::SoftPrint() const
{
    stringstream ss;
    ss << "PartialJPDPValuePair: val = "<< GetValue() <<", pol:"; 
    ss << _m_jpol->SoftPrint();
    return(ss.str());
}

string PartialJPDPValuePair::SoftPrintBrief() const
{ 
    stringstream ss;
    ss << "PartialJPDPValuePair(" << GetValue() << ","
       << _m_jpol->SoftPrintBrief() << "[d = " << _m_jpol->GetDepth()
       << "])";
    return(ss.str());
}
