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
#ifndef _QBGPLANNER_TREEINCPRUNEBNB_H_
#define _QBGPLANNER_TREEINCPRUNEBNB_H_ 1

/* the include directives */
#include <iostream>
#include <map>
#include "Globals.h"

#include "MonahanBGPlanner.h"
#include "AlphaVectorPlanning.h"
#include "Timing.h"

class Scope;

//#if USE_POMDPSOLVE_LIBRARY
//// we should not include any of Tony Cassandra's header files in one
//// of our headers, otherwise the macros in that code (such as Equal())
//// mess up our functions
//namespace pomdpsolve {

    //struct PomdpSolveParamStruct;
    //struct AlphaListType;
    //typedef struct AlphaListType *AlphaList;


//}
//#endif


/**QBGPlanner_TreeIncPruneBnB computes vector-based QBG functions
 * using tree-based incremental pruning.  */
class QBGPlanner_TreeIncPruneBnB : public MonahanBGPlanner
{
private:    
    
    /// Compute a backup stage.
    virtual QFunctionsDiscrete
    BackupStage(const QFunctionsDiscrete &Qs, size_t maxNrAlphas=0);

    void Initialize();

    std::vector<AlphaVector>* GetGaoa(Index a,
                       const std::vector<Index> &os,
                       const std::vector<Index> &as) const;
    std::vector<AlphaVector>* GetGaoa(Index a, Index o, Index aPrime) const;

    //GaobetaVectorSet *_m_Gaoa;
    //MonahanBGPlanner.h does: typedef boost::multi_array<VectorSet*,3> GaobetaVectorSet;
    //We use pointers here, because we will often pass these Alpha vectors around - however, we need to store them somewhere...
    boost::multi_array< std::vector<AlphaVector>, 2> * _m_Gao_storage;
    boost::multi_array< std::vector<AlphaVector>, 3> * _m_Gaoa_storage;
    boost::multi_array< std::vector<AlphaVector>*, 3> * _m_Gaoa;

    /// The sets H that represent the heuristic function h.  
    //_m_heuristic_for_depth[depth] is a std::vector<AlphaVector>* 
    std::vector< std::vector<AlphaVector> > _m_heuristic_for_depth;

    //some statistics:
    LIndex _m_nr_bottom_hits;
    LIndex _m_nr_backtracks;
    LIndex _m_nodes_visited;
    std::vector<LIndex> _m_union_nodes_at_level;
    std::vector<LIndex> _m_cs_nodes_at_level;

    ///This initializes H using Gao vectors for ja
    void InitializeHeuristicsForJA(Index ja);

    //std::map<std::string,VectorSet> _m_vectorSetCache;

    //VectorSet //<- no longer needed, our result is now constructed in the set L
    void UnionNode(Index a, 
                        Index depth, 
                        const std::vector<std::vector<Index> > &jpol,
                        //VectorSet & L, // <- the lower bound vectors
                        //VectorSet & G  //The implied g-function
                        std::vector<AlphaVector> & L, // <- the lower bound vectors
                        std::vector<AlphaVector> & G  //The implied g-function
                        );

    //VectorSet //<- no longer needed, our result is now constructed in the set L
    void CrossSumNode(Index a, 
                        Index depth, 
                        const std::vector<std::vector<Index> > &jpol,
                        //VectorSet & L, // <- the lower bound vectors
                        //VectorSet G    //The implied g-function:
                        std::vector<AlphaVector> & L, // <- the lower bound vectors
                        std::vector<AlphaVector> & G    //The implied g-function:
                        );

    bool IsDominatedNode(
                        Index depth, 
                        //const VectorSet & L,
                        //const VectorSet & G
                        std::vector<AlphaVector> & L, // <- the lower bound vectors
                        std::vector<AlphaVector> & G  //The implied g-function
            );

    
    std::vector<Scope> GetValidJointActions(Index depth,
                                    const std::vector<std::vector<Index> > &jpol) const;
    std::vector<Scope> GetValidActions(Index depth,
                                    const std::vector<std::vector<Index> > &jpol) const;

    Index GetJointObservationsFromDepth(Index depth) const;
    std::vector<Index> GetObservationsFromDepth(Index depth) const;

    std::string SoftPrintJpol(const std::vector<std::vector<Index> > &jpol) const;

/*
    std::string ComputeKey(Index a,
                           Index depth,
                           const std::vector<std::vector<Index> > &jpol,
                           std::string extraIdentifier=std::string("")) const;
    bool CheckCache_CrossSum(Index a,
                            Index depth,
                            const std::vector<std::vector<Index> > &jpol,
                            VectorSet &G);

    bool CheckCache_Union(Index a,
                          Index depth,
                          const std::vector<std::vector<Index> > &jpol,
                          VectorSet &G);

    void AddToCache_CrossSum(Index a,
                             Index depth,
                             const std::vector<std::vector<Index> > &jpol,
                             const VectorSet &G);
    void AddToCache_Union(Index a,
                          Index depth,
                          const std::vector<std::vector<Index> > &jpol,
                          const VectorSet &G);
*/
    Index _m_maxDepth;

    bool _m_pruneAfterUnion, _m_pruneAfterCrossSum, _m_useVectorCache;

    static const Index UNSPECIFIED_ACTION=INDEX_MAX;

    Timing _m_timing;

protected:

    //Check Domination routintes probably should be moved to a higher class (AlphaVectorPlanning?)
    //
    bool 
    CheckDomination(
                            const AlphaVector & v,
                            const std::vector<AlphaVector> & vSet
            );
    bool 
    CheckDominationPointWise(
                            const AlphaVector & v,
                            const std::vector<AlphaVector> & vSet
            );
    bool 
    CheckDominationLP(
                            const AlphaVector & v,
                            const std::vector<AlphaVector> & vSet
            );
    bool 
    CheckNotDominatedLP_POMDPSolve(
                            const AlphaVector & v,
                            const std::vector<AlphaVector> & vSet
            );

#if USE_POMDPSOLVE_LIBRARY
    bool 
    CheckNotDominatedLP_POMDPSolve(double* alpha, pomdpsolve::AlphaList& list);
#endif

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QBGPlanner_TreeIncPruneBnB(const PlanningUnitDecPOMDPDiscrete* pu);
    QBGPlanner_TreeIncPruneBnB(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    ~QBGPlanner_TreeIncPruneBnB();

    QFunctionsDiscrete GetQFunctions(size_t horizon)
        { return(_m_qFunction[horizon-1]); }

    ValueFunctionPOMDPDiscrete GetValueFunction(size_t horizon);

    std::string SoftPrintBrief() const;
/*
    void SetPruneAfterUnion(bool doPrune) { _m_pruneAfterUnion=doPrune; }
    void SetPruneAfterCrossSum(bool doPrune) { _m_pruneAfterCrossSum=doPrune; }
    void SetUseVectorCache(bool keepCache) { _m_useVectorCache=keepCache; }
*/
    
    std::string SoftPrintStats() const;
};

#endif /* !_QBGPLANNER_TREEINCPRUNEBNB_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
