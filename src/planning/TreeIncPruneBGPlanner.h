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
#ifndef _TREEINCPRUNEBGPLANNER_H_
#define _TREEINCPRUNEBGPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include <map>
#include "Globals.h"

#include "MonahanBGPlanner.h"
#include "AlphaVectorPlanning.h"
#include "Timing.h"

class Scope;

/**TreeIncPruneBGPlanner computes vector-based QBG functions
 * using tree-based incremental pruning.  */
class TreeIncPruneBGPlanner : public MonahanBGPlanner
{
private:    
    
    /// Compute a backup stage.
    virtual QFunctionsDiscrete
    BackupStage(const QFunctionsDiscrete &Qs, size_t maxNrAlphas=0);

    void Initialize();

    VectorSet* GetGaoa(Index a,
                       const std::vector<Index> &os,
                       const std::vector<Index> &as) const;
    VectorSet* GetGaoa(Index a, Index o, Index aPrime) const;

    GaobetaVectorSet *_m_Gaoa;

    std::map<std::string,VectorSet> _m_vectorSetCache;

    //some statistics:
    LIndex _m_nr_bottom_hits;
    LIndex _m_nr_backtracks;
    LIndex _m_nodes_visited;
    std::vector<LIndex> _m_union_nodes_at_level;
    std::vector<LIndex> _m_cs_nodes_at_level;


    VectorSet UnionNode(Index a, Index depth, 
                        const std::vector<std::vector<Index> > &jpol);
    VectorSet CrossSumNode(Index a, Index depth, 
                           const std::vector<std::vector<Index> > &jpol);
    
    std::vector<Scope> GetValidJointActions(Index depth,
                                            const std::vector<std::vector<Index> > &jpol) const;
    std::vector<Scope> GetValidActions(Index depth,
                                       const std::vector<std::vector<Index> > &jpol) const;

    std::vector<Index> GetObservationsFromDepth(Index depth) const;

    std::string SoftPrintJpol(const std::vector<std::vector<Index> > &jpol) const;

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

    VectorSet PruneAfterUnion(Index depth, const VectorSet &G);
    VectorSet PruneAfterCrossSum(Index depth, const VectorSet &G);

    Index _m_maxDepth;

    bool _m_pruneAfterUnion, _m_pruneAfterCrossSum, _m_useVectorCache;

    size_t _m_cacheHit, _m_cacheMiss;
    std::vector<std::vector<size_t> > _m_pruneAfterUnionStats;
    std::vector<std::vector<size_t> > _m_pruneAfterCrossSumStats;

    static const Index UNSPECIFIED_ACTION=INDEX_MAX;

    void SaveStats() const;

    Timing _m_timing;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    TreeIncPruneBGPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    TreeIncPruneBGPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    ~TreeIncPruneBGPlanner();

    QFunctionsDiscrete GetQFunctions(size_t horizon)
        { return(_m_qFunction[horizon-1]); }

    ValueFunctionPOMDPDiscrete GetValueFunction(size_t horizon);

    std::string SoftPrintBrief() const;
    std::string SoftPrintStats() const;

    void SetPruneAfterUnion(bool doPrune) { _m_pruneAfterUnion=doPrune; }
    void SetPruneAfterCrossSum(bool doPrune) { _m_pruneAfterCrossSum=doPrune; }
    void SetUseVectorCache(bool keepCache) { _m_useVectorCache=keepCache; }
    
};

#endif /* !_TREEINCPRUNEBGPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
