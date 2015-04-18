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

/* Only include this header file once. */
#ifndef _BGIP_BNB_NODE_H_
#define _BGIP_BNB_NODE_H_ 1

/* the include directives */
#include "Globals.h"
#include "BnB_JointTypeOrdering.h"
#include <limits.h>
#include <sstream>
#include "boost/shared_ptr.hpp"

#define INCR_EXPAND 0
#define MAINTAIN_FULL_POL 0
#define DYNAMIC_JT_INDEX_MAPPING 0

class BayesianGameIdenticalPayoffInterface;

//How the fuck should we know that this:
using BGIP_BnB::UNSPECIFIED_ACTION;
//is specified in BnB_JointTypeOrdering ?!?! need to have less files where this stuff is scattered,

class BGIP_BnB_Node;
typedef boost::shared_ptr<BGIP_BnB_Node> BGIP_BnB_NodePtr;

/**\brief BGIP_BnB_Node represents a node in the search tree of
 * BGIP_SolverBranchAndBound.
 */
class BGIP_BnB_Node
{
private:

    /// The parent of this node in the search tree.
    BGIP_BnB_NodePtr _m_parent;

#if 0
    /// The heuristic value of this node: _m_f = _m_g + _m_h.
    double _m_f;
#endif

    /// The sum of the values for the joint types already specified.
    double _m_g;
    /// The sum of the heuristic values for joint types not yet specified.
    double _m_h;

    /// _m_depth is equal to the number of joint types for which a
    /// joint action is specified
    Index _m_depth;

    /**_m_specifiedActions[agentI] contains the index of the individual action
     * this node specifies for agentI (for the typeI implied by the joint type
     * of this node)
     */
    std::vector< Index > _m_specifiedActions;

#if DYNAMIC_JT_INDEX_MAPPING
    std::vector<Index> *_m_jtIndexMapping;
#endif

#if MAINTAIN_FULL_POL        
    std::vector<std::vector<Index > > _m_policy;
#endif
#if INCR_EXPAND
    std::vector<bool> _m_alreadyExpandedJA;
#endif

    void UpdateF();

public:

    BGIP_BnB_Node(size_t nrAgents);

#if 0
    BGIP_BnB_Node(BayesianGameIdenticalPayoffInterface *bgip,
                  Index maxDepth);
    BGIP_BnB_Node(BayesianGameIdenticalPayoffInterface *bgip,
                  Index maxDepth,
                  const std::vector<Index> &jtIndexMapping);
#endif

    BGIP_BnB_Node(const BGIP_BnB_Node& n);

    ~BGIP_BnB_Node();

    double GetF() const { return(_m_g+_m_h); }
    double GetG() const { return(_m_g); }
    double GetH() const { return(_m_h); }
    
#if MAINTAIN_FULL_POL        
    Index GetAction(Index agentI, Index indType) const 
    { 
            return(_m_policy[agentI][indType]); 
    }
#endif

    Index GetAction(Index agentI) const 
    {   return _m_specifiedActions[agentI]; }

    Index GetSpecifiedAction(Index agI, Index depth_tI) const
    { 
        size_t d = this->GetDepth();
        if( depth_tI == d )
            return _m_specifiedActions[agI];
        else if (depth_tI < d)
        {
            if(_m_parent == 0)
            {
                std::stringstream ss;
                ss << "asking parent value, but this node (at depth/that specifies "<<_m_depth<<" joint actions (for as many joint types) ) has no parent!";
                throw E(ss);
            }
            return _m_parent->GetSpecifiedAction(agI, depth_tI);
        }
        else
            throw E("Error asking node for action at a deeper level!");
    }

    void GetImpliedJPol(BayesianGameIdenticalPayoffInterface *bgip,
                        const std::vector< Index >& jtIndexMapping, 
                        std::vector< std::vector < Index > >& impliedJPol) const;

    void GetImpliedJPol(std::vector< std::vector < Index > >& impliedJPol) const;
#if INCR_EXPAND
    bool GetAlreadyExpanded(Index ja) const
        { return(_m_alreadyExpandedJA.at(ja)); }
#endif
    bool IsFullySpecifiedPolicy(Index maxDepth) const;
    BGIP_BnB_NodePtr GetParent() const { return(_m_parent); }

    void SetParent(const BGIP_BnB_NodePtr &parent) { _m_parent=parent; }
#if MAINTAIN_FULL_POL        
    void SetAction(Index agentI, Index indType, Index action)
    { 
        //if(_m_policy[agentI][indType]==INT_MAX)
            //_m_nrActionsLeftToSpecify--;

        _m_policy[agentI][indType]=action;
    }
#endif
    void SetAction(const std::vector<Index> &ja)
    {
        _m_specifiedActions = ja;
    }
    void SetAction(Index agentI, Index action)
    {
        _m_specifiedActions[agentI] = action;
    }
    void UpdateG(double dG);
    void UpdateH(double dH);
    void SetH(double h);

    /*
    size_t GetNrSpecified() const { return(_m_depth); }
    void SetNrSpecified(size_t nr) { _m_depth=nr; }
    */
    /**the depth of a node is the number of joint types specified
     * i.e., a node with depth d specifies the joint type with
     *      jt_oI = d - 1;
     */
    size_t GetDepth() const { return(_m_depth); }
    static Index GetOrderIndexForDepth(Index depth) 
    {
        if(depth > 0)
            return depth-1;
        throw E("no jt_oI for depth-0!");
    }
    void SetDepth(Index nr) { _m_depth=nr; }

#if DYNAMIC_JT_INDEX_MAPPING
    /** \brief Returns the i-th joint type for which a joint action
     * will be selected. */
    Index GetJTIndexMapping(Index i) const
        {
            if(_m_jtIndexMapping)
                return((*_m_jtIndexMapping)[i]);
            else
            {
                throw(E("BGIP_BnB_Node::GetJTIndexMapping() node does not maintain a JointType Index Mapping"));
                return(INT_MAX);
            }
        }

    void SetJTIndexMapping(Index jt_oI, Index jtI)
        {
            if(_m_jtIndexMapping)
            {
                if(jt_oI<=GetOrderIndexForDepth(GetDepth()))
                    throw(E("BGIP_BnB_Node::SetJTIndexMapping can only reshuffle the joint type index mapping for joint types that have not yet been defined at this depth"));
                else
                    (*_m_jtIndexMapping)[jt_oI]=jtI;
            }
            else
                throw(E("BGIP_BnB_Node::SetJTIndexMapping() node does not maintain a JointType Index Mapping"));
        }

    Index GetDepthFirstSpecified(Index agentI, Index typeI) const;
#endif

    void SetAlreadyExpanded(Index ja);
    void ClearAlreadyExpanded();

    std::string SoftPrint() const;
    std::string SoftPrint(
        const std::vector<Index> & jtIndexMapping
            ) const;
    std::string SoftPrintPartiallySpecifiedPolicy(
        BayesianGameIdenticalPayoffInterface *bgip,
        const std::vector<Index> & jtIndexMapping
            ) const;

};

namespace std{
    /**\brief Overload the less<Type> template for BGIP_BnB_Node* (we want less
     * to give an ordering according to values, not addresses...).*/
    template <> 
    struct less< BGIP_BnB_Node* > //struct, so operator() is public by def. 
    {
        bool operator()(const BGIP_BnB_Node* x,
                        const BGIP_BnB_Node* y) const
        { 
            return( x->GetF() < y->GetF() );
        }

    };
    template <> 
    struct less< BGIP_BnB_NodePtr > //struct, so operator() is public by def. 
    {
        bool operator()(const BGIP_BnB_NodePtr x,
                        const BGIP_BnB_NodePtr y) const
        { 
            return( x->GetF() < y->GetF() );
        }

    };
}



#endif /* !_BGIP_BNB_NODE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
