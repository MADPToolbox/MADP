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

#include "PolicyPureVector.h"
#include "IndexTools.h"
#include <stdlib.h>

using namespace std;

#define DEBUG_PPV 0

using std::stringstream;

PolicyPureVector::PolicyPureVector(
    const Interface_ProblemToPolicyDiscretePure* pu,
    Index agentI,
    PolicyGlobals::PolicyDomainCategory idc,
    size_t depth
        ) :
     PolicyDiscretePure(pu,idc, agentI)
{
    Index nrDE = GetInterfacePTPDiscretePure()->
        GetNrPolicyDomainElements(agentI, GetPolicyDomainCategory(), depth);
    if(DEBUG_PPV)
        cout << "PolicyPureVector(): creating policy for agent "<<agentI
             << ", depth " << depth << " (nr domain elements "
             << nrDE << ")" << endl;

    _m_agentI = agentI;
    _m_domainToActionIndices = vector<Index>(nrDE, 0);
}

PolicyPureVector::PolicyPureVector(
    const I_PtPD_constPtr &pu,
    Index agentI,
    PolicyGlobals::PolicyDomainCategory idc,
    size_t depth
        ) :
     PolicyDiscretePure(pu,idc, agentI)
{
    Index nrDE = GetInterfacePTPDiscretePure()->
        GetNrPolicyDomainElements(agentI, GetPolicyDomainCategory(), depth);
    if(DEBUG_PPV)
        cout << "PolicyPureVector(): creating policy for agent "<<agentI
             << ", depth " << depth << " (nr domain elements "
             << nrDE << ")" << endl;

    _m_agentI = agentI;
    _m_domainToActionIndices = vector<Index>(nrDE, 0);
}

//Copy assignment constructor.    
PolicyPureVector::PolicyPureVector(const PolicyPureVector& o) :
    PolicyDiscretePure(o)
{
    if(DEBUG_PPV)    cout << " clone PolicyPureVector ";
    _m_agentI = o._m_agentI;
    _m_domainToActionIndices =vector<Index>(o._m_domainToActionIndices);
}

//Destructor
PolicyPureVector::~PolicyPureVector()
{
    _m_domainToActionIndices.clear();
}

PolicyPureVector& PolicyPureVector::operator= (const PolicyPureVector& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    PolicyDiscretePure::operator= ( o );
    _m_agentI = o._m_agentI;
    _m_domainToActionIndices = o._m_domainToActionIndices;

    return *this;
}
void PolicyPureVector::ZeroInitialization()
{
if(DEBUG_PPV)cout << ">>>PolicyPureVector::ZeroInitialization(): for agent " 
                              << _m_agentI << endl;
    vector<Index>::iterator it = _m_domainToActionIndices.begin();
    vector<Index>::iterator last = _m_domainToActionIndices.end();

    while(it != last)
    {
        *it = 0;
        it++;
    }
}

void PolicyPureVector::RandomInitialization()
{
    if(DEBUG_PPV)
        cout << ">>>PolicyPureVector::RandomInitialization(): for agent " 
            << _m_agentI << endl;

    Index nrA = GetInterfacePTPDiscretePure()->GetNrActions(_m_agentI);   
    vector<Index>::iterator it = _m_domainToActionIndices.begin();
    vector<Index>::iterator last = _m_domainToActionIndices.end();

    while(it != last)
    {
        int r = rand();
        if(DEBUG_PPV)
            cout << "rand() = " <<r<<endl;
        *it =  r% nrA;
        it++;
    }
}

bool PolicyPureVector::Increment()
{
    bool carry_over = true;
    size_t nrA = GetInterfacePTPDiscretePure()->GetNrActions(_m_agentI);
    size_t nrOH = GetInterfacePTPDiscretePure()->
        GetNrPolicyDomainElements(_m_agentI, GetPolicyDomainCategory(),
                                  GetDepth());

    //i is an integer index counting from nrObservationHistories-1 (=the last
    // observation history) to 0
    //(corresponding to the first (empty) observation sequence.
    Index i = nrOH - 1;
    while(carry_over)
    {
        _m_domainToActionIndices.at(i) = (_m_domainToActionIndices.at(i) + 1) % nrA;
        carry_over = (_m_domainToActionIndices.at(i) == 0);
        if(i > 0)
            i--;
        else
            break;
    }
    return(carry_over);
}

LIndex PolicyPureVector::GetIndex() const
{
    int nrO=GetInterfacePTPDiscretePure()->
        GetNrPolicyDomainElements(_m_agentI,
                                  GetPolicyDomainCategory(),
                                  GetDepth() ),
        nrA=GetInterfacePTPDiscretePure()->GetNrActions(_m_agentI);
    LIndex i=0;

    vector<LIndex> nrElems(2);
    nrElems[0]=1;
    nrElems[1]=nrA;

    vector<LIndex> indexVec(2);

    for(int o=0;o!=nrO;++o)
    {
        nrElems[0]*=nrO;
        indexVec[0]=i;
        indexVec[1]=_m_domainToActionIndices[o];
        i=IndexTools::IndividualToJointIndices(indexVec,nrElems);
    }
    return(i);
}

void PolicyPureVector::SetIndex(LIndex i)
{
    int nrO=GetInterfacePTPDiscretePure()->
        GetNrPolicyDomainElements(_m_agentI, 
                                  GetPolicyDomainCategory(),
                                  GetDepth()),
        nrA=GetInterfacePTPDiscretePure()->GetNrActions(_m_agentI);
    
    vector<LIndex> nrElems(2);
    nrElems[0]=static_cast<LIndex>(pow(static_cast<double>(nrA),nrO));
    nrElems[1]=nrA;
    vector<LIndex> indexVec(2);

    for(int o=nrO-1;o>=0;--o)
    {
        nrElems[0]/=nrO;
        indexVec=IndexTools::JointToIndividualIndices(i,nrElems);
        SetAction(o,CastLIndexToIndex(indexVec[1]));
        i=indexVec[0];
    }
}

void PolicyPureVector::SetDepth(size_t d)
{
    Policy::SetDepth(d);
    _m_domainToActionIndices.resize(
        GetInterfacePTPDiscretePure()->GetNrPolicyDomainElements(
            _m_agentI,
            GetPolicyDomainCategory(),
            d));
}

string PolicyPureVector::SoftPrint() const
{
    vector<Index>::const_iterator it = _m_domainToActionIndices.begin();
    vector<Index>::const_iterator last = _m_domainToActionIndices.end();
   
    //const ObservationHistoryTree* oht;
    Index dIndex = 0;

    stringstream ss;

    while(it != last)
    {
        ss << GetInterfacePTPDiscretePure()->SoftPrintPolicyDomainElement
            (_m_agentI, dIndex, GetPolicyDomainCategory() );
        ss << " --> ";
        ss << GetInterfacePTPDiscretePure()->SoftPrintAction(_m_agentI, *it);
        ss << endl;
                
        it++;
        dIndex++;
    }
    return(ss.str());
}

