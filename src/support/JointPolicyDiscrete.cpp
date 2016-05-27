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

#include "JointPolicyDiscrete.h"
#include "PolicyDiscrete.h"
using namespace std;

//Default constructor
JointPolicyDiscrete::JointPolicyDiscrete(const Interface_ProblemToPolicyDiscrete* iptpd,
                                         PolicyGlobals::PolicyDomainCategory idc )  :
    JointPolicy(iptpd->GetNrAgents())
    ,_m_indexDomCat( idc )
    ,_m_PTPD(iptpd),
    _m_PTPDshared()
{
}
//Default constructor
JointPolicyDiscrete::JointPolicyDiscrete(const I_PtPD_constPtr &iptpd,
                                         PolicyGlobals::PolicyDomainCategory idc )  :
    JointPolicy(iptpd->GetNrAgents())
    ,_m_indexDomCat( idc )
    ,_m_PTPD(0)
    ,_m_PTPDshared(iptpd)
{
}
//Copy  constructor.    
JointPolicyDiscrete::JointPolicyDiscrete(const JointPolicyDiscrete& o) 
    :
    JointPolicy(o)
    ,_m_indexDomCat(o._m_indexDomCat)
    ,_m_PTPD(o._m_PTPD)
    ,_m_PTPDshared(o._m_PTPDshared)
{
}

JointPolicyDiscrete&
JointPolicyDiscrete::operator= (const JointPolicyDiscrete& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyDiscrete::operator= (const JointPolicyDiscrete& o) called"<<endl;
#endif
    if (this == &o) return *this;   // Gracefully handle self assignment
    JointPolicy::operator= ( o );   //call parent
    _m_indexDomCat = o._m_indexDomCat;
    _m_PTPD = o._m_PTPD;
    _m_PTPDshared = o._m_PTPDshared;
    return *this;
}

void
JointPolicyDiscrete::SampleJointActionVector( 
        const vector<Index>& indivDomIndices , 
        vector<Index>& sampled_aIs  ) const
{
    sampled_aIs.resize(_m_nrAgents);
    for(Index agI=0; agI < _m_nrAgents; agI++)
    {
        PolicyDiscrete* p = GetIndividualPolicyDiscrete(agI);
        Index agI_domI = indivDomIndices.at(agI);
        sampled_aIs.at(agI) = p->SampleAction(agI_domI);
    }
    return;
}

Index JointPolicyDiscrete::SampleJointAction( 
        const vector<Index>& indivDomIndices  ) const
{
    vector<Index> sampled_aIs;
    SampleJointActionVector(indivDomIndices, sampled_aIs);
    Index ja = GetInterfacePTPDiscrete()->IndividualToJointActionIndices(sampled_aIs);
    return ja;

}
void
JointPolicyDiscrete::SampleJointActionVector( Index i, 
       vector<Index>& sampled_aIs ) const
{

    if( GetInterfacePTPDiscrete()->AreCachedJointToIndivIndices(_m_indexDomCat) )
    {
        const vector<Index>& indivDomIndices = GetInterfacePTPDiscrete()->
            JointToIndividualPolicyDomainIndicesRef (i, _m_indexDomCat);
        return( SampleJointActionVector( indivDomIndices, sampled_aIs) );
    }
    else
    {
        vector<Index> indivDomIndices = GetInterfacePTPDiscrete()->    
            JointToIndividualPolicyDomainIndices(i, _m_indexDomCat);
        return( SampleJointActionVector( indivDomIndices, sampled_aIs) );
    }
            
}
Index JointPolicyDiscrete::SampleJointAction( Index i ) const
{
    if( GetInterfacePTPDiscrete()->AreCachedJointToIndivIndices(_m_indexDomCat) )
    {
        const vector<Index>& indivDomIndices = GetInterfacePTPDiscrete()->
            JointToIndividualPolicyDomainIndicesRef (i, _m_indexDomCat);
        return( SampleJointAction( indivDomIndices) );
    }
    else
    {
        vector<Index> indivDomIndices = GetInterfacePTPDiscrete()->    
            JointToIndividualPolicyDomainIndices(i, _m_indexDomCat);
        return( SampleJointAction( indivDomIndices) );
    }
            
}
