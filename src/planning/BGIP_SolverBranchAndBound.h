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
#ifndef _BGIP_SOLVERBRANCHANDBOUND_H_
#define _BGIP_SOLVERBRANCHANDBOUND_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BGIP_IncrementalSolverInterface_T.h"
#include "BGIP_BnB_Node.h"
#include "BnB_JointTypeOrdering.h"
#include <float.h>
#include <queue>
#include <algorithm>
#include <limits.h>
#include "JPPVValuePair.h"
#include "PartialJPDPValuePair.h"
#include "VectorTools.h"

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// compute _m_impliedJPol when a new node is selected:
#define CACHE_IMPLIED_JPOL 0

#define INITIALIZE_LB_TO_BESTFOUND 1


#define DEBUG_VALID_ACTIONS 0
#define CHECK_VALID_JA 0

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

using BGIP_BnB::UNSPECIFIED_ACTION;

/**\brief BGIP_SolverBranchAndBound is a class that performs
 * Branch-and-Bound search for identical payoff Bayesian Games.
 *
 * The template argument JP represents the joint policy class the
 * solver should return.
 */
template<class JP>
class BGIP_SolverBranchAndBound : public BGIP_IncrementalSolverInterface_T<JP>
{
private:

    /// Level of verboseness of the solver.
    int _m_verbosity;

    /// The number of solutions we already computed for the incremental solver.
    size_t _m_nrSolutionsComputed;

    /// Whether we already called Solve().
    bool _m_solved;

    /// Pointer to the BG we're solving.
    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> _m_bgip;

    /// The priority queue keeping track of all the open nodes in the search.
    std::priority_queue<BGIP_BnB_NodePtr> *_m_openQueue;

    ///the best node found so far...
    BGIP_BnB_NodePtr _m_bestNode;
    
    //_m_completeInformationValues[jt_bgI]
    std::vector<double> _m_completeInformationValues;
    
    /// The ordering of joint types used in the search.
    BGIP_BnB::BnB_JointTypeOrdering _m_jtOrdering;

    /**_m_jtIndexMapping[i] the i-th joint type for which a joint action will 
     * be selected,
     *      _m_jtIndexMapping[i] = j 
     * means that the j-th joint-type of the BG will be the i-th for which a ..
     *
     * so 
     * \li i is the order index: i_oI
     * \li j is the BG-index:   j_bgI
     */
    std::vector<Index> _m_jtIndexMapping;
    /**_m_depthFirstSpecified[agentI][typeI] specifies at what depth
     * typeI of agentI is first specified.
     * This is computed when the ordering is determined.
     */
    std::vector< std::vector< Index > > _m_depthFirstSpecified;

#if CACHE_IMPLIED_JPOL
    /**vector in which we cache the joint policy implied by a node:
     *      _m_impliedJPol[agI][typeI] = aI
     * if a node does specify the action for typeI of agI
     *      _m_impliedJPol[agI][typeI] = UNSPECIFIED_ACTION
     * if it doesn't specify that action.
     */
    std::vector< std::vector< Index > > _m_impliedJPol;
#endif
    ///the maximum depth of a search node (i.e., the depth at which it is fully 
    //expanded)
    Index _m_maxDepth;

    bool _m_expandAll;
    bool _m_reComputeH;
    bool _m_reComputeJTIndexMapping;

    double _m_CBGlowerBound;
    double _m_CBGupperBound;
    double _m_maxLowerBound;

    // keep some statistics
    size_t _m_nrNodesExpanded;
    size_t _m_nrNodesPruned;
    size_t _m_nrNodesFullySpecified;

    size_t _m_nrAgents;
    const std::vector<size_t> & _m_nrActions;
    size_t _m_nrJTs;

    std::vector<std::vector<Index> > _m_jaToIndCache;

    double ReSolve()
    {
        if(_m_verbosity>=1)
        {
            std::cout << "BGIP_SolverBranchAndBound starting BG Branch'n'Bound search now!"<<std::endl;
            LIndex nrJPols=0;
            try{
                nrJPols = this->GetBGIPI()->GetNrJointPolicies();
            } catch(E& e)
            { ; }
            std::cout << "BGIP_SolverBranchAndBound Solver Address: "
                      << this << ", nr BG JPols=" << nrJPols
                      << ", CBG_LB=" << _m_CBGlowerBound << std::endl;
        }

        _m_maxLowerBound=-DBL_MAX;
#if INITIALIZE_LB_TO_BESTFOUND
        if(this->GetNrFoundSolutions()>0)
        {
            _m_maxLowerBound=this->GetPayoff();
            if(_m_verbosity>=0)
                std::cout << "Initialized max lower bound to value of best solution (out of "
                          << this->GetNrFoundSolutions() << " already found): "
                          << _m_maxLowerBound << std::endl;
        }
#endif

        Index i=0;
        // keep on expanding the open queue until it's empty
        while(!_m_openQueue->empty())
        {
            PrintStatistics(i++);

            if(i % 100 == 0)
                this->CheckDeadline("BnB deadline exceeded");

            BGIP_BnB_NodePtr top=_m_openQueue->top();
            

            if(this->GetNrDesiredSolutions()==1 &&
               top->IsFullySpecifiedPolicy(_m_maxDepth) &&
               top->GetF() > (_m_CBGupperBound-PROB_PRECISION))
            {
                std::cout << "Hit CBG upperbound" << std::endl;
                ProcessFullySpecifiedNode(top);
                _m_nrNodesFullySpecified++;
                break;
            }
            if(top->GetF() < _m_maxLowerBound &&
               top->IsFullySpecifiedPolicy(_m_maxDepth)) // we still need to check
                                              // whether this is a fully
                                              // specified policy, there
                                              // can be multiple nodes
                                              // with the same
                                              // heuristic<LB, but not all
                                              // need to be complete
            {
                //We are done!!!
                //(max. upper bound < best found policy)
                
                //put the best policy on the solution
    //             ConvertNodeToPolicyAndAddToSolution(top, _m_maxLowerBound);
    //             PopAndDeleteNode(top);
                ProcessFullySpecifiedNode(top);
                _m_nrNodesFullySpecified++;
                //not really needed:
                //Prune(_m_maxLowerBound);
                break;
            }

            //for evaluation of the final policy, the expansion of nodes
            //and the computation of heuristic values  for the children we
            //will need to check whether actions are consistent. 
#if CACHE_IMPLIED_JPOL
            //(We cache the already specified actions in _m_impliedJPol)
            InitImpliedJPol();
#if DEBUG_VALID_ACTIONS 
            std::cout << "BGIP_SolverBranchAndBound implied jpol after initialization: " << 
                SoftPrintVector(_m_impliedJPol) << std::endl;
#endif                        
            top->GetImpliedJPol(_m_bgip, _m_jtIndexMapping, _m_impliedJPol);
#if DEBUG_VALID_ACTIONS 
            std::cout << "BGIP_SolverBranchAndBound implied jpol after GetImpliedJPol: " << 
                SoftPrintVector(_m_impliedJPol) << std::endl;
#endif                        
#endif // CACHE_IMPLIED_JPOL

            //else (if ! top->GetF() < _m_maxLowerBound )
            // check if node is fully specified
            if(top->IsFullySpecifiedPolicy(_m_maxDepth))
            {
                // updates _m_maxLowerBound and deletes the node
                ProcessFullySpecifiedNode(top);
                _m_nrNodesFullySpecified++;
            }
            else
            {
                if(_m_verbosity>3)
                    std::cout << "BGIP_SolverBranchAndBound EXPANDING top node "
                              <<std::endl;
                //top will be expanded
                //
#if INCR_EXPAND
                if(_m_expandAll)
                    ExpandAllExtensions(top);
                else
                    ExpandOneExtension(top);
#else                    
                ExpandAllExtensions(top);
#endif
            }
        }

        if(_m_verbosity>=1)
        {
            std::cout << "BGIP_SolverBranchAndBound Found "
                      << (this->
                          GetNrFoundSolutions())
                      << " solutions, after expanding "
                      << _m_nrNodesExpanded << " nodes ("
                      << GetPerJPRatio(_m_nrNodesExpanded) <<" pjp) - of which "
                      << _m_nrNodesFullySpecified << " full ("
                      << GetPerJPRatio(_m_nrNodesFullySpecified)
                      << " pjp) - and pruning "
                      << _m_nrNodesPruned << " of those"
                      << std::endl;
            std::cout << "BGIP_SolverBranchAndBound " << this
                      << " Open Queue size: " << _m_openQueue->size() << std::endl;
        }

        return(this->
               GetPayoff());
    };

    /// Computes the "complete information" heuristic values for all joint types.
    void ComputeCompleteInformationValues()
    {
        _m_completeInformationValues.resize(_m_bgip->GetNrJointTypes());
        for(Index i=0;i!=_m_completeInformationValues.size();++i)
        {
            if(i % 100 == 0)
                this->CheckDeadline("BnB deadline exceeded (in ComputeCompleteInformationValues())");

            double bestValue=-DBL_MAX;
            for(Index ja=0;ja!=_m_bgip->GetNrJointActions();++ja)
            {
                double c=GetContribution(i,ja);
                bestValue=std::max(c,bestValue);
            }
            _m_completeInformationValues[i]=bestValue;
        }
        
        if(_m_verbosity>2)
            std::cout << "BGIP_SolverBranchAndBound Complete information values: "
                      << SoftPrintVector(_m_completeInformationValues)
                      << std::endl;
    };

    std::vector<double> ComputeMinContributionValues() const
    {
        std::vector<double> minValues(_m_bgip->GetNrJointTypes());
        for(Index i=0;i!=minValues.size();++i)
        {
            double minValue=DBL_MAX;
            for(Index ja=0;ja!=_m_bgip->GetNrJointActions();++ja)
            {
                double c=GetContribution(i,ja);
                minValue=std::min(c,minValue);
            }
            minValues[i]=minValue;
        }
        
        if(_m_verbosity>2)
            std::cout << "BGIP_SolverBranchAndBound Minimum contribution values: "
                      << SoftPrintVector(minValues)
                      << std::endl;
        return(minValues);
    };

    /// Sorts a std::vector in ascending order.
    void SortAscending(std::vector<double> &values) const{
    std::sort(values.begin(), values.end());
};

    /// Sorts a std::vector in descending order.
    void SortDescending(std::vector<double> &values) const{
    SortAscending(values);
    
    // invert the sorting
    std::vector<double> valuesDescending(values.size());
    for(Index i=0;i!=values.size();++i)
        valuesDescending[values.size()-1-i]=values[i];
    values=valuesDescending;
};
        
    void ReOrderJointTypes(std::vector<Index>& jtIndexMapping){
    std::vector<Index> identityJTIndexMapping(_m_completeInformationValues.size());
    for(Index i=0;i!=identityJTIndexMapping.size();++i)
        identityJTIndexMapping[i]=i;

    switch(_m_jtOrdering)
    {
        // no reordering at all
    case BGIP_BnB::IdentityMapping:
    {
        jtIndexMapping=identityJTIndexMapping;
        break;
        // reorder to have joint types with the highest
        // heuristic complete information values first
    }
    case BGIP_BnB::MaxContribution:
    case BGIP_BnB::ConsistentMaxContribution:
    {
        std::vector<double> CIvalues=_m_completeInformationValues;
        const std::vector<double> & origCIvalues=_m_completeInformationValues;
        SortDescending(CIvalues);

        ReOrderJointTypes(identityJTIndexMapping,
                          origCIvalues,
                          CIvalues,
                          jtIndexMapping);
        break;
    }
    // reorder to have joint types with the worst
    // heuristic complete information values first
    case BGIP_BnB::MinContribution:
    case BGIP_BnB::ConsistentMinContribution:
    {
        std::vector<double> minValues=ComputeMinContributionValues();
        std::vector<double> origMinValues=minValues;
        // sorts in ascending order, no need to invert now
        SortAscending(minValues);

        if(_m_verbosity>2)
            std::cout << "BGIP_SolverBranchAndBound Sorted Minimum contribution values: "
                      << SoftPrintVector(minValues)
                      << std::endl;
        ReOrderJointTypes(identityJTIndexMapping,
                          origMinValues,
                          minValues,
                          jtIndexMapping);
        break;
    }
    case BGIP_BnB::MaxContributionDifference:
    case BGIP_BnB::ConsistentMaxContributionDifference:
    {
        std::vector<double> maxCIvalues=_m_completeInformationValues;
        std::vector<double> minValues=ComputeMinContributionValues();
        std::vector<double> origDiffValues(minValues.size());
        for(Index i=0;i!=origDiffValues.size();++i)
            origDiffValues[i]=maxCIvalues[i] - minValues[i];
        std::vector<double> diffValues=origDiffValues;

        SortDescending(diffValues);
                
        if(_m_verbosity>2)
            std::cout << "BGIP_SolverBranchAndBound Sorted Max difference contribution values: "
                      << SoftPrintVector(diffValues)
                      << std::endl;

        ReOrderJointTypes(identityJTIndexMapping,
                          origDiffValues,
                          diffValues,
                          jtIndexMapping);
                                               
        break;
    }
    case BGIP_BnB::DescendingProbability:
    {
        std::vector<double> prob(_m_bgip->GetNrJointTypes());
        for(Index i=0;i!=prob.size();++i)
            prob[i]=_m_bgip->GetProbability(i);
        std::vector<double> origProb=prob;

        SortDescending(prob);

        if(_m_verbosity>2)
            std::cout << "BGIP_SolverBranchAndBound Sorted probability values: "
                      << SoftPrintVector(prob)
                      << std::endl;

        ReOrderJointTypes(identityJTIndexMapping,
                          origProb,
                          prob,
                          jtIndexMapping);

        break;
    }
    case BGIP_BnB::BasisTypes:
    {
        jtIndexMapping=std::vector<Index>(_m_completeInformationValues.size(),
                                          INT_MAX);

        // Reorder to have the basis types (basis observations
        // in the PBIP paper) are in front. This means that
        // only they will be used.

        for(Index i=0;i!=jtIndexMapping.size();++i)
            jtIndexMapping[i]=i;

        size_t largestNrIndividualTypes=0;
        for(Index aI=0;aI!=_m_bgip->GetNrAgents();++aI)
            largestNrIndividualTypes=
                std::max(largestNrIndividualTypes,
                         _m_bgip->GetNrTypes(aI));

        Index j=0;
        std::vector<Index> indTypes(_m_bgip->GetNrAgents());
        for(Index i=0;i!=largestNrIndividualTypes;++i)
        {
            for(Index aI=0;aI!=_m_bgip->GetNrAgents();++aI)
            {
                // this is required for agents with different type sizes
                if(_m_bgip->GetNrTypes(aI) < largestNrIndividualTypes &&
                    i>=_m_bgip->GetNrTypes(aI))
                    indTypes[aI]=0;
                else
                    indTypes[aI]=i;
            }

            Index jt=_m_bgip->IndividualToJointTypeIndices(indTypes);
            // swap so that these basis types are in front
            jtIndexMapping[j]=jt;
            jtIndexMapping[jt]=j;

            j++;
        }
        if(_m_verbosity>2)
            std::cout << "BGIP_SolverBranchAndBound Shortest possible joint type space: "
                      << SoftPrintVector(jtIndexMapping)
                      << std::endl;
        break;
    }
    default:
        throw(E("BGIP_SolverBranchAndBound::ReOrderJointTypes unhandled BnB_JointTypeOrdering"));
    }

    if(_m_verbosity>2)
    {
        std::cout << "BGIP_SolverBranchAndBound Sorted JT indices: "
                  << SoftPrintVector(jtIndexMapping)
                  << std::endl << "I.e.: < ";
        for(Index i = 0; i < jtIndexMapping.size(); i++)
        {
            std::cout <<
                SoftPrintVector( _m_bgip->JointToIndividualTypeIndices( 
                                     jtIndexMapping[i] ) )
                      << ", ";
        }
        std::cout << std::endl;
    }

    //compute _m_depthFirstSpecified
    //the total number of actions we need to specify
    size_t nrAcsToSpecify = 0;
    //initialize all to UNSPECIFIED_ACTION
    for(Index agI=0; agI < _m_nrAgents; agI++)
    {
        size_t nrTypes_i =  _m_bgip->GetNrTypes(agI);
        nrAcsToSpecify += nrTypes_i;
        _m_depthFirstSpecified[agI] = 
            std::vector< Index> ( nrTypes_i , UNSPECIFIED_ACTION);
    }


    //std::cout << "Computing _m_depthFirstSpecified!"<<std::endl;
    //for(Index jt_oI=0; jt_oI < jtIndexMapping.size(); jt_oI++)
    Index jt_oI = 0;
    Index depth=0;
    while(nrAcsToSpecify > 0)
    {
        //the actions for the 0-th joint type will be specified at
        //depth 1!!!
        depth = jt_oI + 1;
        Index jt_bgI = jtIndexMapping[jt_oI];
        const std::vector<Index> &indTypes=
            _m_bgip->JointToIndividualTypeIndices(jt_bgI);
        bool uselessJointType = true;
        for(Index agI=0; agI < _m_nrAgents; agI++)
        {
            Index typeI = indTypes[agI];
            if(_m_depthFirstSpecified[agI][typeI] == UNSPECIFIED_ACTION)
            {
                //this type was not yet specified, so we record it
                uselessJointType = false;
                _m_depthFirstSpecified[agI][typeI] = depth;
                if(_m_verbosity>=1)
                    std::cout << "BGIP_SolverBranchAndBound typeI"
                              <<typeI<<" of agI"
                              <<agI<<" first specified at depth="<<depth<<std::endl;
                nrAcsToSpecify--;
            }
        }
        if(uselessJointType)
        {
            //std::cout << "The "<<jt_oI<<"-th joint type (that specifies the joint type for which an action is selected at depth "<<depth<<" of the search tree) is useless! (erasing it...)"<<std::endl;
            jtIndexMapping.erase(jtIndexMapping.begin() + jt_oI);
            //put it on the back (otherwise heuristics will not be computed correctly)
            jtIndexMapping.push_back(jt_bgI);

        }
        else
            jt_oI++;
    }
    if(_m_verbosity>=1)
        std::cout << "BGIP_SolverBranchAndBound Done computing _m_depthFirstSpecified! - max-depth="
                  <<depth<<std::endl;
    _m_maxDepth=depth;
};

    /// Get the contribution of a (joint type,joint action) pair.
    double GetContribution(Index jtI, Index jaI) const
        {
            return(_m_bgip->GetProbability(jtI)*_m_bgip->GetUtility(jtI,jaI));
        }

    /**\brief Orders the joint type mapping origJTIndexMapping (with
     * values origValues) according to orderValues.
     */
    void ReOrderJointTypes(
            const std::vector<Index> &origJTIndexMapping,
            const std::vector<double> &origValues,
            const std::vector<double> &orderedValues,
            std::vector<Index>& jtIndexMapping ) const{
    if((origJTIndexMapping.size() != origValues.size()) ||
       (origValues.size()!=orderedValues.size()))
        throw(E("BGIP_SolverBranchAndBound::ReOrderJointTypes() std::vectors should all have the same length"));

    size_t nrJTypes=origJTIndexMapping.size();
    jtIndexMapping = std::vector<Index>(nrJTypes, UNSPECIFIED_ACTION);

    //we search where jt_bgI ended up
    for(Index jt_bgI=0;jt_bgI!=nrJTypes;++jt_bgI)
    {
        //the original value of the jt_bgI-th joint type (of the BG)
        double origValue=origValues[jt_bgI];
        for(Index jt_oI=0;jt_oI!=nrJTypes;++jt_oI)
        {
            if(orderedValues[jt_oI]==origValue &&
               //make sure that jt_oI is not specified yet (in
               //case of equal vals)
               jtIndexMapping[jt_oI]==UNSPECIFIED_ACTION)
            {
                jtIndexMapping[jt_oI]=origJTIndexMapping[jt_bgI];
                break;
            }
        }
    }
};

    ///returns std::vector with all joint action indices that are valid.
    void 
    ComputeValidJointActionExtensions(BGIP_BnB_NodePtr node, 
                                      std::vector<Index>& valid_JAs){
    // a std::vector containing all valid joint actions
    //std::vector<Index> valid_JAs;
    valid_JAs.clear();
    // the joint type we are currently considering
    // *node* specifies jt_oI =  node->GetDepth() - 1
    // so its child specifies jt_oI =  node->GetDepth()
    Index jt_oI = node->GetDepth();
    Index jt_bgI=GetJTIndexMapping(jt_oI, node);

#if DEBUG_VALID_ACTIONS            
    std::cout << "\tComputing valid JAs for the "<<jt_oI<<"th joint type, corr. to jt_bgI"<<jt_bgI<<std::endl;
#endif
    ComputeValidJointActions(jt_bgI, valid_JAs, node);
#if CHECK_VALID_JA        
    std::vector<Index> control_JAs;
    ComputeValidJointActionsOld(node, jt_bgI, control_JAs);
    std::sort( valid_JAs.begin(), valid_JAs.end());
    std::sort( control_JAs.begin(), control_JAs.end());
    if(! VectorTools::Equal(valid_JAs, control_JAs) )
    {
        std::cout <<"BGIP_SolverBranchAndBound WARNING!!! valid joint actions not equal!!!"<<std::endl
                  << "valid_JAs="<< SoftPrintVector(valid_JAs) << std::endl
                  << "control_JAs="<< SoftPrintVector(control_JAs) << std::endl;
    }
    else
        std::cout << "BGIP_SolverBranchAndBound Valid joint actions equal: "<< 
            SoftPrintVector(valid_JAs) << std::endl;
#endif
    
};
   
    /**computes the valid joint actions in a slow manner
     * but always works correctly.
     */
    void 
    ComputeValidJointActionsOld(
            BGIP_BnB_NodePtr node, 
            Index jt_bgI, 
            std::vector<Index>& valid_JAs ){
    const std::vector<Index> &indTypes=
        _m_bgip->JointToIndividualTypeIndices(jt_bgI);
    // we loop over all joint actions and check whether 
    // they are valid
    for(Index ja=0;ja!=_m_jaToIndCache.size();++ja)
    {
        const std::vector<Index> &aI=_m_jaToIndCache[ja];

        // assume this joint action is consistent until we
        // find a counterexample
        bool consistent=true;
        
        // only check each individual has been specified, and
        // if so, it should be consistent
        for(Index agI=0;agI!=_m_nrAgents;++agI)
        {
            //Index acI=node->GetAction(agI,indTypes[agI]);
            Index acI;
            Index tI = indTypes[agI];
#if CACHE_IMPLIED_JPOL
            acI = _m_impliedJPol[agI][tI];
#else          
            Index depth_tI = GetDepthFirstSpecified(agI,tI,node);
            acI = node->GetSpecifiedAction(agI, depth_tI);
#endif
            if(acI!=UNSPECIFIED_ACTION && 
               aI[agI] != acI)
            {
                consistent=false;
                break;
            }
        }                
        if(consistent)
        {
            //std::cout << SoftPrintVector(aI) << std::endl;
            valid_JAs.push_back(ja);
        }
    }
}
;

    /**the following computes the valid joint actions for jt_bgI.
     *
     * when CACHE_IMPLIED_JPOL is true, it simply uses the cached 
     * implied policy:
     *      valid_action_set[agI] = _m_impliedJPol[agI][tI];
     * to determine the valid actions for each agent.
     */
    void 
    ComputeValidJointActions(
            Index jt_bgI, 
            std::vector<Index>& valid_JAs,
            BGIP_BnB_NodePtr node){
    const std::vector<Index> &indTypes=
        _m_bgip->JointToIndividualTypeIndices(jt_bgI);

#if DEBUG_VALID_ACTIONS            
    std::cout << "\tComputing valid JAs for indTypes=" <<
        SoftPrintVector(indTypes) << std::endl;
#endif

    //  valid_action_set[agI] stores the valid action for agent i
    std::vector< Index > valid_action_set(_m_nrAgents, UNSPECIFIED_ACTION);
    for(Index agI=0; agI < _m_nrAgents; agI++)
    {
        Index tI = indTypes[agI];
#if CACHE_IMPLIED_JPOL
        valid_action_set.at(agI) = _m_impliedJPol.at(agI).at(tI);
#else            
        //should be pre-computed with the ordering:
        Index depth_tI = GetDepthFirstSpecified(agI,tI,node);
        Index depth_node = node->GetDepth(); //this node's depth
#if DEBUG_VALID_ACTIONS            
        std::cout << "\tagent "<<agI<<"'s type ( tI="<<tI
                  <<" ) is first specified at depth_tI "<< depth_tI << std::endl;
        std::cout << "\tthis node's depth_node is " << depth_node << std::endl;
#endif
        if( depth_tI <= depth_node )
            //when depth_tI == depth_node, then *this* node specifies the 
            //action for tI. This means that the valid joint actions that 
            //can be assigned to children nodes of this node, are 
            //constrained!
            valid_action_set.at(agI) = node->GetSpecifiedAction(agI, depth_tI);
        else
        {
            ;
            //we do nothing because
            //  valid_action_set[agI] = {1,...,GetNrActions(agI) }
            //is encoded by valid_action_set[agI] = UNSPECIFIED_ACTION
            //(which is set already)
        }
#endif
#if DEBUG_VALID_ACTIONS            
        std::cout <<"\tvalid action for agI "<<agI<<" is " << 
            valid_action_set.at(agI) << std::endl;
#endif
    }
    std::vector<Index> curr_JA(_m_nrAgents);
    GenerateJointActions(valid_action_set,curr_JA, valid_JAs, 0);
#if DEBUG_VALID_ACTIONS            
    std::cout <<"Generated the following valid_JAs:"<<SoftPrintVector(valid_JAs)<<endl;
#endif
    //return(valid_JAs);
};

    void GenerateJointActions(
              const std::vector<Index>& valid_action_set, //input arg
              //maintains the choices made earlier in the tree
              std::vector<Index>& curr_JA, 
              //the output set where the valid jais are put in
              std::vector<Index>& valid_JAs, 
              //the agent we now will select actions for 
              //(i.e., the depth in the search tree) :
              Index agentI 
        )
{
    //for all valid actions of this agent
    if(valid_action_set[agentI] == UNSPECIFIED_ACTION)
    {
#if DEBUG_VALID_ACTIONS            
        std::cout << "Processing agent "<<agentI<<" of which the action is not yet specified (going to loop through its " << _m_nrActions.at(agentI) <<" actions)"<<endl;
#endif
        for(Index acI=0; acI<_m_nrActions.at(agentI); acI++)
        {
            GenerateJointActions_ProcessAction(valid_action_set,curr_JA,
                                               valid_JAs, agentI, acI);
        }
        if (_m_nrActions.at(agentI) == 0)
        {
            std::stringstream ss;
            ss<<"BGIP_SolverBranchAndBound<JP>::GenerateJointActions - Error agent "<<agentI<< " has 0 actions!";
            throw E(ss);
            //or may be usefull for some strange cases...?
            //in any case, the following doesn't work yet, the computation of joint <-> individual joint action
            //indices does not work if the nr actions is 0 for 1 agent it seems...(?)
            //
            //Index acI = 0;
            //GenerateJointActions_ProcessAction(valid_action_set,curr_JA,
                                               //valid_JAs, agentI, acI);
        }
#if DEBUG_VALID_ACTIONS            
        std::cout << "Done processing agent "<<agentI<<", resulting valid_JAs:"<<SoftPrintVector(valid_JAs)<<endl;
#endif
    }
    else
    {
        Index acI = valid_action_set[agentI];
#if DEBUG_VALID_ACTIONS            
        std::cout << "Processing agent "<<agentI<<" of which the action already is specified acI="<<acI
            <<endl;
#endif
        GenerateJointActions_ProcessAction(valid_action_set,curr_JA,
                                           valid_JAs, agentI, acI);
#if DEBUG_VALID_ACTIONS            
        std::cout << "Done processing agent "<<agentI<<", resulting valid_JAs:"<<SoftPrintVector(valid_JAs)<<endl;
#endif
    }
    return;
};

    void GenerateJointActions_ProcessAction(
              const std::vector<Index>& valid_action_set, //input arg
              std::vector<Index>& curr_JA, 
              std::vector<Index>& valid_JAs, 
              Index agentI,
              Index actionI
        ){
#if DEBUG_VALID_ACTIONS            
        std::cout<< "GenerateJointActions_ProcessAction: agent "<<agentI << " actionI "<<actionI<<"...";
#endif
    curr_JA[agentI] = actionI;
    //if this is last agent then agentI == _m_nrAgents - 1
    if(agentI < _m_nrAgents - 1)
    {
        //do recursive call
#if DEBUG_VALID_ACTIONS            
        std::cout << "Not the last agent, going deeper in recursion..."<< std::endl;
#endif
        GenerateJointActions(valid_action_set,curr_JA, valid_JAs, agentI+1);
    }
    else
    {
#if DEBUG_VALID_ACTIONS            
        std::cout << "Last agent, so the joint action is fully specified"<< std::endl;
#endif
        //this is the last agent, so the joint action is fully specified!
        Index jaI = _m_bgip->IndividualToJointActionIndices(curr_JA);
        valid_JAs.push_back(jaI);
#if DEBUG_VALID_ACTIONS            
        std::cout << "resulting valid_JAs:"<<SoftPrintVector(valid_JAs)<<endl;
#endif
    }
    return;
}
;

    void ReComputeH(BGIP_BnB_NodePtr node)
{
    size_t nrJT = _m_bgip->GetNrJointTypes();
    //size_t _m_nrAgents = _m_bgip->GetNrAgents();
    double h=0;
    for(Index jt_oI = node->GetDepth(); jt_oI < nrJT; ++jt_oI)
    {
        Index jt_bgI = GetJTIndexMapping(jt_oI, node);    
        double bestValue=-DBL_MAX;
        std::vector< Index > valid_JAs;
        //the next call assumes that the induced policy for *node*
        //has been set!
        ComputeValidJointActions(jt_bgI, valid_JAs, node);
        for(Index val_jaI=0; val_jaI < valid_JAs.size(); val_jaI++)
        {
            Index ja = valid_JAs[val_jaI];
            double c=_m_bgip->GetUtility(jt_bgI,ja);
            bestValue=std::max(c,bestValue);
        }
        h+=_m_bgip->GetProbability(jt_bgI)*bestValue;
    }
    if(_m_verbosity>2)
        std::cout << "\tBGIP_SolverBranchAndBoundUpdated H from " << node->GetH()
                  << " to " << h << std::endl;
    node->SetH(h);

    if(_m_reComputeJTIndexMapping)
    {
    switch(_m_jtOrdering)
    {
#if DYNAMIC_JT_INDEX_MAPPING
    case BGIP_BnB::ConsistentMaxContribution:
    {
        throw(E("BGIP_SolverBranchAndBound::ReComputeH unhandled ConsistentMaxContribution nyi"));
//         std::vector<double> CIvalues=_m_completeInformationValues;
//         const std::vector<double> & origCIvalues=_m_completeInformationValues;
//         SortDescending(CIvalues);

//         ReOrderJointTypes(origCIvalues,CIvalues,jtIndexMapping);
        break;
    }
    // reorder to have joint types with the worst
    // heuristic complete information values first
    case BGIP_BnB::ConsistentMinContribution:
    {
        throw(E("BGIP_SolverBranchAndBound::ReComputeH unhandled ConsistentMinContribution nyi"));
//         std::vector<double> minValues=ComputeMinContributionValues();
//         std::vector<double> origMinValues=minValues;
//         // sorts in ascending order, no need to invert now
//         SortAscending(minValues);

//         if(_m_verbosity>2)
//             std::cout << "Sorted Minimum contribution values: "
//                       << SoftPrintVector(minValues)
//                       << std::endl;
//         ReOrderJointTypes(origMinValues,minValues,jtIndexMapping);
        break;
    }
    case BGIP_BnB::ConsistentMaxContributionDifference:
    {
//         std::vector<vector < Index > > impliedJpol=InitImpliedJPol();
//         node->GetImpliedJPol(impliedJpol);
        size_t nrJTtoReOrder=_m_bgip->GetNrJointTypes() - node->GetDepth();
        std::vector<double> origDiffValues(nrJTtoReOrder);
        std::vector<Index> jtIndexMappingPartial(nrJTtoReOrder);
        std::vector<bool> jaAlreadySpecified(_m_bgip->GetNrJointActions(), false);

        std::vector< std::vector< Index > > impliedJPol(_m_nrAgents);
        for(Index agI=0; agI < _m_nrAgents; agI++)
        {
            impliedJPol[agI] = std::vector< Index >(
                _m_bgip->GetNrTypes(agI), UNSPECIFIED_ACTION );
        }
        node->GetImpliedJPol(impliedJPol);

        for(Index i=0;i!=origDiffValues.size();++i)
        {
            Index jt_oI=node->GetDepth() + i;
            Index jtI=node->GetJTIndexMapping(jt_oI);
            jtIndexMappingPartial.at(i)=jtI;
            double minValue=DBL_MAX;
            double maxValue=-DBL_MAX;
            const std::vector<Index> &indTypes=
                _m_bgip->JointToIndividualTypeIndices(jtI);
            
            for(Index ja=0;ja!=_m_bgip->GetNrJointActions();++ja)
            {
                const std::vector<Index> &indActions=
                    _m_bgip->JointToIndividualActionIndices(ja);

                bool jaAlreadySpecified=true;
                for(Index agI=0;agI!=_m_nrAgents;++agI)
                {
                    if(impliedJPol[agI][indTypes[agI]]!=indActions[agI])
                        jaAlreadySpecified=false;
                }
                if(!jaAlreadySpecified)
                {
                    double c=GetContribution(jtI,ja);
                    minValue=std::min(c,minValue);
                    maxValue=std::max(c,maxValue);
                }
            }
            origDiffValues[i]=maxValue-minValue;
        }

        std::vector<double> diffValues=origDiffValues;

        SortDescending(diffValues);
        
        if(_m_verbosity>2)
        {
            std::cout << "BGIP_SolverBranchAndBound Original Max difference contribution values: "
                      << SoftPrintVector(origDiffValues)
                      << std::endl;
            std::cout << "BGIP_SolverBranchAndBound Sorted Max difference contribution values: "
                      << SoftPrintVector(diffValues)
                      << std::endl;
        }
        std::vector<Index> jtIndexMapping;
        ReOrderJointTypes(jtIndexMappingPartial,
                          origDiffValues,
                          diffValues,
                          jtIndexMapping);
        for(Index i=0;i!=jtIndexMapping.size();++i)
        {
            Index jt_oI=node->GetDepth() + i;
            node->SetJTIndexMapping(jt_oI, jtIndexMapping[i]);
        }
        break;
    }
#endif
    default:
        throw(E("BGIP_SolverBranchAndBound::ReComputeH unhandled BnB_JointTypeOrdering"));
    }

    }
};            

    double ComputeValueOfFullySpecifiedPolicy(BGIP_BnB_NodePtr node)
    {
        std::vector<Index> aIs(_m_bgip->GetNrAgents());
        
        // For the already specified joint types, G contains their value
        double value=node->GetG();
        // We loop over the remaining types to get their contribution
        for(Index jt_oI = node->GetDepth();
            jt_oI <  _m_bgip->GetNrJointTypes();
            ++jt_oI)
        {
            Index jt_bgI = GetJTIndexMapping(jt_oI, node);
            const std::vector<Index> &indTypes=
                _m_bgip->JointToIndividualTypeIndices(jt_bgI);
            for(Index agI=0;agI!=indTypes.size();++agI)
            {
                //aIs[agI]=node->GetAction(agI,indTypes[agI]);
                Index acI;
                Index tI = indTypes[agI];
#if MAINTAIN_FULL_POL        
                acI = node->GetAction(agI,tI);
#else
#if 0 //CACHE_IMPLIED_JPOL
          // NO, this is not correct, because
          //_m_impliedJPol is not related to the particular node we are
          //computing this for
                acI = _m_impliedJPol[agI][tI];
#else          
                Index depth_tI = GetDepthFirstSpecified(agI,tI,node);
                acI = node->GetSpecifiedAction(agI, depth_tI);
#endif
#endif                    
                aIs[agI] = acI;
            }

            Index ja = _m_bgip->IndividualToJointActionIndices(aIs);
            
            value += GetContribution(jt_bgI, ja);
        }
        return(value);
    };

    void ConvertNodeToPolicyAndAddToSolution(BGIP_BnB_NodePtr node, double value)
    {
        boost::shared_ptr<JP> jpol =  boost::dynamic_pointer_cast<JP>( this->GetNewJpol() );
        for(Index agI=0;agI!=_m_nrAgents;++agI)
            for(Index tI=0;tI!=_m_bgip->GetNrTypes(agI);++tI)
            {
                Index acI;
#if MAINTAIN_FULL_POL        
                acI = node->GetAction(agI,tI);
#else
#if 0 //CACHE_IMPLIED_JPOL
          // NO, this is not correct, because
          //_m_impliedJPol is not related to the particular node we are
          //computing this for
                acI = _m_impliedJPol[agI][tI];
#else          
                Index depth_tI = GetDepthFirstSpecified(agI,tI,node);
                acI = node->GetSpecifiedAction(agI, depth_tI);
#endif
#endif
                jpol->SetAction(agI,tI,acI);
            }
        this->
            AddSolution(*jpol, value);
        if(_m_verbosity>=1)
            std::cout << "BGIP_SolverBranchAndBound added solution " << jpol
                      << " with value " << value << std::endl;
    };

    void ProcessFullySpecifiedNode(BGIP_BnB_NodePtr node)
    {
        if(!node->IsFullySpecifiedPolicy(_m_maxDepth))
            throw(E("ProcessFullySpecifiedNode called with a node which is not fully specified"));

        // compute the true value of this jpol
        double value=ComputeValueOfFullySpecifiedPolicy(node);
        if(_m_verbosity>3)
            std::cout << "BGIP_SolverBranchAndBound Found fully specified policy with value "
                      << value << ":" << std::endl;
        
        ConvertNodeToPolicyAndAddToSolution(node,value);

        bool lowerBoundImproved=false;
        // update max lowerbound if possible
        if(value > _m_maxLowerBound)
        {
            _m_maxLowerBound=value;
            lowerBoundImproved=true;
            if(_m_verbosity>=1)
                std::cout << "BGIP_SolverBranchAndBound Updated max lower bound to "
                          << _m_maxLowerBound << std::endl; 
        }

        if(this->GetNrDesiredSolutions()>1)
        {
            // if we want more than 1 solution, keep track of it in the
            // closed queue.
    //         if(lowerBoundImproved)
    //         {
    //         }
    //         else
    //         {
                Pop(node);
    //            _m_closedQueueFullySpecifiedNodes->push(node);
    //        }

        }
        else 
        {
            if(lowerBoundImproved)
            {
                //save the new best node
                _m_bestNode = node;
                //and remove it from the top of the priority queue
                Pop(node); 
            
                //", now pruning..." << std::endl;
                // now we could prune the queue, but that's slow
                // because of the need to reconstruct a new
                // priority queue, so instead we rely on
                // on-the-fly pruning
#if 0 // slow
                Prune(_m_maxLowerBound);
#endif
            }
            else // we only 
            {
                //remove the node from the prior.Q and delete it
                PopAndDeleteNode(node);
            }
        }
    };

    void ExpandAllExtensions(BGIP_BnB_NodePtr node)
    {
        if(_m_verbosity>3)
            std::cout << "BGIP_SolverBranchAndBound Expanding the following node: "
                      << node->SoftPrint(_m_jtIndexMapping) << std::endl;
        // find the valid extensions of this node
        // no by value for std::vectors!
        //std::vector<Index> JAs=
        //ComputeValidJointActionExtensions(node);
        std::vector<Index> JAs;
        ComputeValidJointActionExtensions(node, JAs);
        if(_m_verbosity>3)
            std::cout << "BGIP_SolverBranchAndBound found " << JAs.size()
                      << " joint action extensions" << std::endl;
        
        Pop(node);

        for(Index i=0;i!=JAs.size();++i)
        {
            Expand(node,JAs[i]);
        }

    //    _m_closedQueueIntermediateNodes->push(node);
    };

#if INCR_EXPAND &&  MAINTAIN_FULL_POL        
    //this should be improved IF we want incremental expansion
    // 1) if we expand 1-by-1, we want to first expand the joint action that gives the higest 
    //    contribution (i.e., we want to order the joint actions by contribution)
    //
    // 2) after expanding a child, the value of the parent should be updated I guess, right?
    //   (otherwise it will immediately be selected for expansion again...)
    // 
    void ExpandOneExtension(BGIP_BnB_NodePtr node)
    {
        // the joint type we are currently considering
        Index currentJT=_m_jtIndexMapping[node->GetDepth()];
        const std::vector<Index> &currentTs=
            _m_bgip->JointToIndividualTypeIndices(currentJT);
        
        if(_m_verbosity>3)
            std::cout << "BGIP_SolverBranchAndBound Expanding node once:"
                      << std::endl
                      << node->SoftPrint() << std::endl;
        bool foundExpansion=false;
        // we loop over all joint actions
        Index ja;
        for(ja=0;ja!=_m_jaToIndCache.size();++ja)
        {
            const std::vector<Index> &aI=_m_jaToIndCache[ja];
            
            // assume this joint action is consistent until we
            // find a counterexample
            bool consistent=true;
            
            for(Index i=0;i!=currentTs.size();++i)
            {
                //Index a=node->GetAction(i,currentTs[i]);
                Index depth_tI = GetDepthFirstSpecified(i,currentTs[i]);
                Index a = node->GetSpecifiedAction(agI, depth_tI);
                if(a!=UNSPECIFIED_ACTION && 
                   aI[i] != a)
                {
                    consistent=false;
                    break;
                }
                
            }
            // if the joint is consistent AND we did not expand it
            // before, this is the one we want to expand
            if(consistent &&
               !node->GetAlreadyExpanded(ja))
            {
                foundExpansion=true;
                break;
            }
        }
        
        if(foundExpansion)
            Expand(node, ja);
        else
        {
#if 1
            if(_m_verbosity>4)
                std::cout << "BGIP_SolverBranchAndBound Removing fully expanded node from queue:"
                          << std::endl
                          << node->SoftPrint() << std::endl;
            // node was already fully expanded, so remove it
            PopAndDeleteNode(node);
#else
            Pop(node);
#endif
        }
    };
#endif

    void Expand(BGIP_BnB_NodePtr node, Index JA)
    {
        const std::vector<Index> &ja=_m_jaToIndCache[JA];
        
        // Create a new node, copy from its parent
        BGIP_BnB_NodePtr nodeExtend=BGIP_BnB_NodePtr(new BGIP_BnB_Node(*node));

        nodeExtend->SetDepth( node->GetDepth() + 1 );
#if INCR_EXPAND
        nodeExtend->ClearAlreadyExpanded();
#endif
        nodeExtend->SetParent(node);
        _m_nrNodesExpanded++;
        
        //jt_oI of *nodeExtended* is *node*->GetDepth
        Index jt_oI = node->GetDepth();
        Index jt_bgI = _m_jtIndexMapping[jt_oI];
        
        // update the new nodes joint policy
        const std::vector<Index> &indTypes=
            _m_bgip->JointToIndividualTypeIndices(jt_bgI);
        for(Index agI=0;agI!=indTypes.size();++agI)
        {
#if MAINTAIN_FULL_POL        
            nodeExtend->SetAction(agI,indTypes[agI],ja[agI]);
#else
            nodeExtend->SetAction(agI,ja[agI]);
#endif

#if CACHE_IMPLIED_JPOL
            //update the implied policy:
            //(this is used by ReComputeH below!)
            _m_impliedJPol[agI][ indTypes[agI] ] = ja[agI];
#endif
        }
        
        // update the G and H values, which automatically updates F:
        if(_m_reComputeH)
            ReComputeH(nodeExtend);
        else
        {
            // first we substract the heuristic CI value for the
            // current joint type
            nodeExtend->UpdateH(-_m_completeInformationValues[jt_bgI]);
        }
        // then we add the real value to G
        nodeExtend->UpdateG(GetContribution(jt_bgI,JA));
                
        if(_m_verbosity>4)
            std::cout << "BGIP_SolverBranchAndBound Adding node: "
                      << nodeExtend->SoftPrint() << std::endl;
        
#if INCR_EXPAND
        // keep track of already expanded nodes
        node->SetAlreadyExpanded(JA);
#endif

        if(node->GetF() < _m_CBGlowerBound)
            _m_nrNodesPruned++; // do nothing, by not putting it on the
                                // queue it will be deleted
        else
            // add the node to the queue
            _m_openQueue->push(nodeExtend);
    };

    void Prune(double value)
    {
        // this Prune routine is quite slow...

        std::priority_queue<BGIP_BnB_NodePtr> * newQueue = 
            new std::priority_queue<BGIP_BnB_NodePtr>();
        
        int nrPruned=0;
        while(!_m_openQueue->empty())
        {
            BGIP_BnB_NodePtr node = _m_openQueue->top();
            std::cout << "BGIP_SolverBranchAndBound Queue pruning: " << node->GetF() << " " << value << std::endl;
            if(node->GetF() > value)
            {
                Pop(node);
                newQueue->push(node);
            }
            else
            {
                PopAndDeleteNode(node);
    //            _m_closedList.push_back(node);
                nrPruned++;
                _m_nrNodesPruned++;
            }
    //        _m_openQueue->pop();    
        }
        delete _m_openQueue; //delete old queue
        _m_openQueue = newQueue; //point to new queue
        
        if(_m_verbosity>=0)
            std::cout << "BGIP_SolverBranchAndBound Pruned " << nrPruned << " nodes" << std::endl;
    };

    void Pop(BGIP_BnB_NodePtr node)
    {
        if(_m_openQueue->top()!=node)
            throw(E("Node to be popped is not the one specified"));

        _m_openQueue->pop();
    };

    void PopAndDeleteNode(BGIP_BnB_NodePtr node)
    {
        Pop(node);
        //Frans 20110922: deleting is not necessary due to shared pointed (?) 
    //    node.reset();
    //    delete node;
    //    _m_closedList.push_back(node);
    };

    /** \brief Returns the i-th joint type for which a joint action
     * will be selected. */
    Index GetJTIndexMapping(Index i, BGIP_BnB_NodePtr node) const
        {
#if DYNAMIC_JT_INDEX_MAPPING
            if(_m_reComputeJTIndexMapping)
                return(node->GetJTIndexMapping(i));
            else
#endif
                return(_m_jtIndexMapping[i]);
        }

    
    Index GetDepthFirstSpecified(Index agentI, Index typeI, 
                                 BGIP_BnB_NodePtr node) const 
        {
#if DYNAMIC_JT_INDEX_MAPPING
            if(_m_reComputeJTIndexMapping)
                return(node->GetDepthFirstSpecified(agentI,typeI));
            else
#endif
                return(_m_depthFirstSpecified[agentI][typeI]);
        }

#if CACHE_IMPLIED_JPOL
    void InitImpliedJPol()
    {
        std::vector<std::vector<Index> >::iterator it1 = _m_impliedJPol.begin();
        std::vector<std::vector<Index> >::iterator last1 = _m_impliedJPol.end();
        while(it1 != last1)
        {
            std::vector<Index>::iterator it2 = (*it1).begin();
            std::vector<Index>::iterator last2 = (*it1).end();
            while(it2 != last2)
            {
                *it2 = UNSPECIFIED_ACTION;
                it2++;
            }
            it1++;
        }
    }
;
#endif
    double GetPerJPRatio(size_t s) const
    {
        double nrJPols=0;
        try{
            nrJPols = CastLIndexToDouble(
                _m_bgip->GetNrJointPolicies()
                );
        } catch(E& e)
        { 
            nrJPols = std::numeric_limits<double>::max();
        }
        return (double)s / nrJPols;
    };
    void PrintStatistics(Index i) const
    {
#define PRINTNUM 100000
        Index printNum = PRINTNUM;
        if(_m_verbosity>1)
        {
            // limit the second argument of pow(), otherwise we get SIGFPE
            printNum /= pow(10,std::min(_m_verbosity-1,5));
        }
        if(_m_verbosity>=0 && i % printNum == 0 && i > printNum) 
        {
            //do not do the printing when not outputting it!
            std::stringstream stats;
            stats << i << "-th node selected for expansion:   "<<_m_openQueue->size() 
                  << " nodes in queue ( "
                  << GetPerJPRatio(_m_openQueue->size())
                  <<" per joint policy), pruned "
                  << _m_nrNodesPruned
                  << ", full policies "
                  << _m_nrNodesFullySpecified
                  << "( " << GetPerJPRatio(_m_nrNodesFullySpecified)
                  << " pjp )";
            std::cout << "BGIP_SolverBranchAndBound " << stats.str() << std::endl;
        }
    };

protected:
    
public:
    // Constructor, destructor and copy assignment.
    // (default) Constructor
    //BGIP_SolverBranchAndBound();
    /**Constructor. Directly Associates a problem with the planner
     * Information regarding the problem is used to construct a joint policy
     * of the proper shape.*/
    BGIP_SolverBranchAndBound(const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
                              int verbose=0, 
                              size_t nrDesiredSolutions=1,
                              bool expandAll=false,
                              BGIP_BnB::BnB_JointTypeOrdering jtOrdering=BGIP_BnB::IdentityMapping,
                              bool reComputeHeur=false
        )
    :
        BGIP_IncrementalSolverInterface_T<JP>(bg,nrDesiredSolutions),
        _m_verbosity(verbose),
        _m_nrSolutionsComputed(0),
        _m_solved(false),
        _m_bgip(bg),
        _m_openQueue(new std::priority_queue<BGIP_BnB_NodePtr>()),
    //    _m_bestNode(0),
        _m_jtOrdering(jtOrdering),
        _m_jtIndexMapping(0),
        _m_depthFirstSpecified(bg->GetNrAgents() ),
#if CACHE_IMPLIED_JPOL
        _m_impliedJPol(bg->GetNrAgents() ),
#endif
        _m_expandAll(expandAll),
        _m_reComputeH(reComputeHeur), 
        _m_reComputeJTIndexMapping(false), 
        _m_CBGlowerBound(-DBL_MAX),
        _m_CBGupperBound(DBL_MAX),
        _m_maxLowerBound(-DBL_MAX),
        _m_nrNodesExpanded(0),
        _m_nrNodesPruned(0),
        _m_nrNodesFullySpecified(0),
        _m_nrAgents(bg->GetNrAgents() ),
        _m_nrActions( bg->GetNrActions() ),
        _m_nrJTs(bg->GetNrJointTypes() )
    {
        // set up a cache for quick joint action conversions
        for(Index ja=0;ja!=_m_bgip->GetNrJointActions();++ja)
            _m_jaToIndCache.push_back(
                _m_bgip->JointToIndividualActionIndices(ja));
        switch(jtOrdering)
        {
        case BGIP_BnB::ConsistentMaxContribution:
        case BGIP_BnB::ConsistentMinContribution:
        case BGIP_BnB::ConsistentMaxContributionDifference:
            _m_reComputeJTIndexMapping=true;
            break;
        default:
            _m_reComputeJTIndexMapping=false;
            break;
        }

#if CACHE_IMPLIED_JPOL
        for(Index agI=0; agI < _m_nrAgents; agI++)
        {
            _m_impliedJPol[agI] = std::vector< Index >(
                _m_bgip->GetNrTypes(agI), UNSPECIFIED_ACTION );
        }
#endif
    };

    ~BGIP_SolverBranchAndBound()
    {delete _m_openQueue;};

    double Solve()
    {
        _m_solved=true;
        this->InitDeadline();
        if(_m_verbosity>=1)
            std::cout << "BGIP_SolverBranchAndBound::Solve() called."<<std::endl;
        // Compute upper bound heuristic values
        ComputeCompleteInformationValues();
        
        // Reorder the joint types
        ReOrderJointTypes(_m_jtIndexMapping); 

        if(_m_verbosity>=1)
            std::cout
                << "BGIP_SolverBranchAndBound Resulting order of joint types: "
                << SoftPrintVector(_m_jtIndexMapping) << std::endl;

        // Initialize root node
        BGIP_BnB_NodePtr root;
        // If we want to update the joint type index mapping during the
        // search, we have to maintain it for each node. Otherwise we can
        // just use the copy in this class.
#if DYNAMIC_JT_INDEX_MAPPING
        if(_m_reComputeJTIndexMapping)
            root = new BGIP_BnB_Node(_m_bgip, _m_maxDepth,
                                     _m_jtIndexMapping);
        else
#endif
            root = BGIP_BnB_NodePtr(new BGIP_BnB_Node(_m_bgip->GetNrAgents()));
        // G is initialized to 0,
        // H is also initialized to 0, so we now *add* the contributions for each joint type
        for(Index jt_bgI=0;jt_bgI!=_m_bgip->GetNrJointTypes();++jt_bgI)
            root->UpdateH(_m_completeInformationValues.at(jt_bgI));

        // push the root node on the queue of open nodes
        _m_openQueue->push(root);

        if(_m_verbosity>3)
            std::cout << "BGIP_SolverBranchAndBound "
                      << root->SoftPrint() << std::endl;

        return(ReSolve());
    };

    size_t GetNrNodesExpanded() const { return(_m_nrNodesExpanded); }

    bool IsExactSolver() const { return(true); }

    bool GetNextJointPolicyAndValueSpecific(boost::shared_ptr<JointPolicyDiscretePure> &jpol, double &value)
    {
        // If we never solved this BG, do that now
        if(!_m_solved)
            Solve();
        // even if we have more solutions stored in the solution queue, we
        // don't know whether they are optimal, so we need to resolve to
        // make sure
        else
            ReSolve();

        bool foundSolution=false;

        // this requires some magic to make sure we return from
        // the correct pool, which depends on the JP type
        if(!this->IsEmptyJPPV())
        {                
            const boost::shared_ptr<JPPVValuePair> jppv=
                this->GetNextSolutionJPPV();
            //if((jpol=boost::dynamic_pointer_cast<JP>(jppv->GetJPPV())))
            if((jpol=boost::static_pointer_cast<JointPolicyDiscretePure>(jppv->GetJPPV())))
            {
                foundSolution=true;
                value=jppv->GetValue();
                this->PopNextSolutionJPPV();
            }
        }
        else if(!this->IsEmptyPJPDP())
        {
            boost::shared_ptr<PartialJPDPValuePair> jppv=
                this->GetNextSolutionPJPDP();
            //if((jpol=boost::dynamic_pointer_cast<JP>(jppv->GetJPol())))
            if((jpol=boost::static_pointer_cast<JointPolicyDiscretePure>(jppv->GetJPol())))
            {
                foundSolution=true;
                value=jppv->GetValue();
                this->PopNextSolutionPJPDP();
            }
        }
        else
        {
            jpol.reset();
            value=0;
        }
        if(_m_verbosity>=0)
            std::cout << "BGIP_SolverBranchAndBound found jpol "
                      << jpol << " value " << value << " and " 
                      << this->GetNrFoundSolutions()
                      << " solutions left" 
                      << std::endl;
        _m_nrSolutionsComputed++;

        return(foundSolution);
    } ;

    void SetCBGlowerBound(double lowerbound)
        { _m_CBGlowerBound=lowerbound; }

    void SetCBGupperBound(double upperbound){ _m_CBGupperBound=upperbound; }

    //bool AllSolutionsHaveBeenReturned() const;
};

#endif /* !_BGIP_SOLVERBRANCHANDBOUND_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
