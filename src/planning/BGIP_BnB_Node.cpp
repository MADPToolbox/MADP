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

#include "BGIP_BnB_Node.h"
#include "BayesianGameIdenticalPayoffInterface.h"
#include <float.h>
#include <numeric>

using namespace std;

BGIP_BnB_Node::BGIP_BnB_Node(size_t nrAgents) :
//    _m_parent(0),
#if 0
    _m_f(0),
#endif
    _m_g(0.0),
    _m_h(0.0),
    _m_depth(0),
    _m_specifiedActions(nrAgents, UNSPECIFIED_ACTION )
#if DYNAMIC_JT_INDEX_MAPPING
    ,    _m_jtIndexMapping(0) // we don't keep a jtIndexMapping per node
#endif
#if INCR_EXPAND
    ,
    _m_alreadyExpandedJA(_m_bgip->GetNrJointActions(), false)
#endif
{
#if MAINTAIN_FULL_POL        
    // initialize policies to UNSPECIFIED_ACTION, meaning unassigned
    for(Index i=0;i!=_m_bgip->GetNrAgents();++i)
    {
        _m_policy.push_back(std::vector<Index>(_m_bgip->GetNrTypes(i),
                                               UNSPECIFIED_ACTION));
    }
#endif
}

#if DYNAMIC_JT_INDEX_MAPPING
BGIP_BnB_Node::BGIP_BnB_Node(
        BayesianGameIdenticalPayoffInterface *bgip,
        Index maxdepth,
        const std::vector<Index> &jtIndexMapping) :
    _m_bgip(bgip),
    _m_parent(0),
    _m_f(0),
    _m_g(0.0),
    _m_h(0.0),
    _m_depth(0),
    _m_maxDepth(maxdepth),
    _m_specifiedActions( _m_bgip->GetNrAgents(), UNSPECIFIED_ACTION ),
    _m_jtIndexMapping(new vector<Index>(jtIndexMapping)) // make a copy of it
#if INCR_EXPAND
    ,
    _m_alreadyExpandedJA(_m_bgip->GetNrJointActions(), false)
#endif
{
    // initialize policies to UNSPECIFIED_ACTION, meaning unassigned
    for(Index i=0;i!=_m_bgip->GetNrAgents();++i)
    {
#if MAINTAIN_FULL_POL        
        _m_policy.push_back(std::vector<Index>(_m_bgip->GetNrTypes(i),
                                               UNSPECIFIED_ACTION));
#endif
    }
}
#endif

BGIP_BnB_Node::BGIP_BnB_Node(const BGIP_BnB_Node& n) :
    _m_parent(n._m_parent),
    _m_g(n._m_g),
    _m_h(n._m_h),
    _m_depth(n._m_depth),
    _m_specifiedActions(n._m_specifiedActions)
#if MAINTAIN_FULL_POL        
    , _m_policy(n._m_policy)
#endif
#if INCR_EXPAND
    , _m_alreadyExpandedJA(n._m_alreadyExpandedJA)
#endif
{
#if DYNAMIC_JT_INDEX_MAPPING
    if(n._m_jtIndexMapping)
        _m_jtIndexMapping=new vector<Index>(*n._m_jtIndexMapping);
    else
        _m_jtIndexMapping=0;
#endif
}

BGIP_BnB_Node::~BGIP_BnB_Node()
{
#if DYNAMIC_JT_INDEX_MAPPING
    delete _m_jtIndexMapping;
#endif
}

void BGIP_BnB_Node::UpdateF()
{
#if 0
    _m_f=_m_g+_m_h;
#endif
}

string BGIP_BnB_Node::SoftPrint() const 
{
    std::stringstream ss;
    ss << "BGIP_BnB_Node[" << this << "] depth(=nr. spec. jtypes)= " << _m_depth
       << " F " << GetF()
       << " G "
       << _m_g
       << " H "
       << _m_h
       << " nrAleft "
       //<< _m_nrActionsLeftToSpecify
       ;
#if INCR_EXPAND
    ss << " jaExp <";
    for(Index ja=0;ja!=_m_alreadyExpandedJA.size();++ja)
        if(_m_alreadyExpandedJA[ja])
            ss << ja << " ";
    ss << ">";
#endif
    return(ss.str());
}

string BGIP_BnB_Node::SoftPrint(
        const vector<Index> & jtIndexMapping
        ) const 
{
    std::stringstream ss;
    ss << "BGIP_BnB_Node[" << this << "] depth(=nr. spec. jtypes)= " << _m_depth
       << " F " << GetF()
       << " G "
       << _m_g
       << " H "
       << _m_h
       << " nrAleft "
       //<< _m_nrActionsLeftToSpecify
       ;
#if INCR_EXPAND
    ss << " jaExp <";
    for(Index ja=0;ja!=_m_alreadyExpandedJA.size();++ja)
        if(_m_alreadyExpandedJA[ja])
            ss << ja << " ";
    ss << ">";
#endif
#if DYNAMIC_JT_INDEX_MAPPING
    if(_m_jtIndexMapping)
        ss << "\n" << SoftPrintPartiallySpecifiedPolicy(*_m_jtIndexMapping);
    else
        ss << "\n" << SoftPrintPartiallySpecifiedPolicy(jtIndexMapping);
#endif
    return(ss.str());
}

    
string BGIP_BnB_Node::SoftPrintPartiallySpecifiedPolicy(
    BayesianGameIdenticalPayoffInterface *bgip,
    const vector<Index> & jtIndexMapping

        ) const
{
    std::stringstream ss;
#if 1
    for(Index jt_oI=0;jt_oI!=_m_depth;++jt_oI)
    {
        Index jt;
#if DYNAMIC_JT_INDEX_MAPPING
        if(_m_jtIndexMapping)
            jt=(*_m_jtIndexMapping)[jt_oI];
        else
#endif
            jt=jtIndexMapping[jt_oI];
        ss << "\tjt=" << jt << " [";
        const std::vector<Index> &indTypes=
            bgip->JointToIndividualTypeIndices( jt );
        for(Index i=0;i!=indTypes.size();++i)
        {
            ss << " (" << indTypes[i] << "->"
               << GetAction(i) << ")";
        }
        ss << "]" << "\n";
    }
#else
    for(Index jt=0;jt!=_m_depth;++jt)
    {
        ss << "jt " << _m_jtIndexMapping[jt] << " [";
        const std::vector<Index> &indTypes=
            bgip->JointToIndividualTypeIndices(
                _m_jtIndexMapping[jt]);
        for(Index i=0;i!=indTypes.size();++i)
        {
            ss << " (" << indTypes[i] << "->"
               << GetAction(i) << ")";
        }
        ss << "]" << "\t";
    }
#endif
    return(ss.str());
}


#if INCR_EXPAND

void BGIP_BnB_Node::SetAlreadyExpanded(Index ja)
{
    if(_m_alreadyExpandedJA.at(ja))
    {
        stringstream ss;
        ss << "BGIP_BnB_Node::SetAlreadyExpanded cannot expand joint action "
           << ja << " more than once: "
           << SoftPrint() << endl;
        throw(E(ss.str()));
    }
    _m_alreadyExpandedJA.at(ja)=true;
}

void BGIP_BnB_Node::ClearAlreadyExpanded()
{
    for(Index ja=0;ja!=_m_alreadyExpandedJA.size();++ja)
        _m_alreadyExpandedJA[ja]=false;
}
#endif

void BGIP_BnB_Node::UpdateG(double dG)
{
    _m_g+=dG;
    UpdateF();
#if 0
    if(_m_parent)
        _m_parent->UpdateG(dG);
#endif
}

void BGIP_BnB_Node::UpdateH(double dH)
{
    _m_h+=dH;
    UpdateF();
#if 0
    if(_m_parent)
        _m_parent->UpdateH(dH);
#endif
}

void BGIP_BnB_Node::SetH(double h)
{
    _m_h=h;
    UpdateF();
}

bool BGIP_BnB_Node::IsFullySpecifiedPolicy(Index maxDepth) const
{
#if 1
    //if(_m_nrActionsLeftToSpecify)
    if(_m_depth < maxDepth)
        return(false);
    else// if _m_depth == _m_maxDepth
        return(true);
#else
    for(Index i=0;i!=_m_policy.size();++i)
        for(Index a=0;a!=_m_policy[i].size();++a)
            if(_m_policy[i][a]==UNSPECIFIED_ACTION)
                return(false);

    return(true);
#endif
}

#if DYNAMIC_JT_INDEX_MAPPING
void BGIP_BnB_Node::GetImpliedJPol(
        std::vector< std::vector < Index > >& impliedJPol
        ) const
{
    if(_m_jtIndexMapping)
        GetImpliedJPol(*_m_jtIndexMapping, impliedJPol);
    else
        throw(E("BGIP_BnB_Node::GetImpliedJPol node does not maintain a JointType Index Mapping"));
}
#endif

void BGIP_BnB_Node::GetImpliedJPol(
    BayesianGameIdenticalPayoffInterface *bgip,
        const std::vector< Index >& jtIndexMapping, 
        std::vector< std::vector < Index > >& impliedJPol
        ) const
{
    Index depth = this->GetDepth();
    if(depth == 0)
        //depth 0 (root) doesn't specify any joint actions...
        return;

    Index jt_oI = GetOrderIndexForDepth(depth);
    Index jt_bgI = jtIndexMapping.at(jt_oI);
    const std::vector<Index> &indTypes = 
        bgip->JointToIndividualTypeIndices(jt_bgI);
    size_t nrAg = bgip->GetNrAgents();

#if 0
    std::cout << "GetImpliedJPol at depth="<<depth;
    std::cout << " (that specifies jt_bgI="<<jt_bgI<<"="<<SoftPrintVector(indTypes)<<")";
    std::cout << " the specified joint action is " << SoftPrintVector(_m_specifiedActions)<<endl;
#endif
    for(Index agI=0; agI < nrAg; agI++)
    {
        Index tI = indTypes.at(agI);
        Index acI =  _m_specifiedActions[agI];
        if(acI != UNSPECIFIED_ACTION)
            impliedJPol.at(agI).at(tI) = acI;
    }
    if(_m_parent == 0)
        throw E("depth>0, expected parent!");
    else
        _m_parent->GetImpliedJPol(bgip, jtIndexMapping, impliedJPol);

}
#if DYNAMIC_JT_INDEX_MAPPING
Index BGIP_BnB_Node::GetDepthFirstSpecified(Index agentI, Index typeI) const
{
    if(_m_jtIndexMapping)
    {
        Index jt_oI = 0;
        Index depth=0;
        bool typeFound=false;
        while(!typeFound)
        {
            //the actions for the 0-th joint type will be specified at
            //depth 1!!!
            depth = jt_oI + 1;
            Index jt_bgI = GetJTIndexMapping(jt_oI);
            const std::vector<Index> &indTypes=
                _m_bgip->JointToIndividualTypeIndices(jt_bgI);
            if(typeI==indTypes[agentI])
            {
                typeFound=true;
                return(depth);
            }
            jt_oI++;
        }
    }
    else
    {
        throw(E("BGIP_BnB_Node::GetDepthFirstSpecified() node does not maintain a JointType Index Mapping"));
    }

    // should never get here
    return(INT_MAX);
}
#endif
