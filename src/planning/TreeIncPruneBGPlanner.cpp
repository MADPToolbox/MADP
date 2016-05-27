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

#include <fstream>
#include "TreeIncPruneBGPlanner.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BeliefValue.h"
#include "Belief.h"

#include "BayesianGameIdenticalPayoff.h"
#include "JointPolicyPureVector.h"

#include "Scope.h"

#include "directories.h"

using namespace std;

#define DEBUG_TreeIncPruneBGPlanner 0
#define DEBUG_TreeIncPruneBGPlannerVerbose 0

#define DEBUG_TreeIncPruneBGPlanner_KeepPruneStats 0

#define DEBUG_TreeIncPruneBGPlanner_TimeQPOMDP 0

//apparently 
//"class static objects must also be declared outside any function or class just like normal globals."
//( http://forums.devshed.com/c-programming-42/linker-errors-undefined-reference-to-static-member-data-193010.html )
//otherwise, we get problems linking the debug executables.
const Index TreeIncPruneBGPlanner::UNSPECIFIED_ACTION;


//Default constructor
TreeIncPruneBGPlanner::TreeIncPruneBGPlanner(const PlanningUnitDecPOMDPDiscrete* pu) :
    MonahanBGPlanner(pu),
    _m_Gaoa(0),
    _m_pruneAfterUnion(true),
    _m_pruneAfterCrossSum(true),
    _m_useVectorCache(true),
    _m_cacheHit(0),
    _m_cacheMiss(0),
    _m_pruneAfterUnionStats(),
    _m_pruneAfterCrossSumStats()
{
}

TreeIncPruneBGPlanner::TreeIncPruneBGPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    MonahanBGPlanner(pu),
    _m_Gaoa(0),
    _m_pruneAfterUnion(true),
    _m_pruneAfterCrossSum(true),
    _m_useVectorCache(true),
    _m_cacheHit(0),
    _m_cacheMiss(0),
    _m_pruneAfterUnionStats(),
    _m_pruneAfterCrossSumStats()
{
}

//Destructor
TreeIncPruneBGPlanner::~TreeIncPruneBGPlanner()
{
    delete _m_Gaoa;
}

void TreeIncPruneBGPlanner::Initialize()
{
    AlphaVectorPlanning::Initialize();

    _m_Gaoa=new GaobetaVectorSet(boost::extents[GetPU()->GetNrJointActions()]
                                 [GetPU()->GetNrJointObservations()]
                                 [GetPU()->GetNrJointActions()]);

    _m_maxDepth=GetPU()->GetNrJointObservations();
    _m_vectorSetCache.clear();

    MonahanPlanner::_m_initialized=true;
}

VectorSet* TreeIncPruneBGPlanner::GetGaoa(Index a,
                                          const vector<Index> &os,
                                          const vector<Index> &as) const
{
    Index aPrime=GetPU()->IndividualToJointActionIndices(as);
    Index jo=GetPU()->IndividualToJointObservationIndices(os);
    return(GetGaoa(a,jo,aPrime));
}

VectorSet* TreeIncPruneBGPlanner::GetGaoa(Index a, Index o, Index aPrime) const
{
    if((*_m_Gaoa)[a][o][aPrime]==0)
        throw(E("TreeIncPruneBGPlanner::GetGaoa set should have been computed"));

    if(aPrime==UNSPECIFIED_ACTION)
        throw(E("TreeIncPruneBGPlanner::GetGaoa the next time step action should be specified"));

    return((*_m_Gaoa)[a][o][aPrime]);
}

QFunctionsDiscrete
TreeIncPruneBGPlanner::BackupStage(const QFunctionsDiscrete &Qs, size_t maxNrAlphas)
{
    _m_nr_bottom_hits = 0;
    _m_nr_backtracks = 0;
    _m_nodes_visited = 0;
    _m_union_nodes_at_level = vector<LIndex>(_m_maxDepth, 0);
    _m_cs_nodes_at_level = vector<LIndex>(_m_maxDepth, 0);
    
    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Qs);

//    vector<Index> jointActionOrder=GetJointActionOrder();

    int nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations();

    cout << "TreeIncPruneBGPlanner::BackupStage computing all Gaoa vectors"; cout.flush();
    *_m_Gaoa=ComputeAllGaoa(V);

#if DEBUG_TreeIncPruneBGPlanner_TimeQPOMDP
    {
        GaoVectorSet Gao_storage=BackProject(V);
        stringstream ss;
        ss << "ComputePOMDPheur_ts" << GetTimeStep();
        _m_timing.Start(ss.str());
        cout << "Computing heuristic..." << endl;
        for(GaoVectorSetIndex ja=0;ja!=nrA;ja++)
        {
            std::vector< std::vector<AlphaVector> > heuristic_for_depth;
            heuristic_for_depth.resize(_m_maxDepth);
            //use the Gao to define the heuristic sets H for the different depths
            for(Index invT=1; invT <= _m_maxDepth; invT++)
            {
                Index T = _m_maxDepth - invT; //t = h - \ttg
                cout << "starting depth " << T <<"..."; cout.flush();
                //figure out what joint observation is assigned at depth T
                Index jo = T;
                std::vector<AlphaVector> Gao = VectorSetToValueFunction(*Gao_storage[ja][jo]);
                std::vector<AlphaVector> H_T;
                if(T < _m_maxDepth - 1)
                {
                    //this is not the last joint observation
                    const std::vector<AlphaVector>  & H_T_next = heuristic_for_depth.at(T+1);
                    CrossSum(Gao, H_T_next, H_T);
                }
                else
                    H_T = Gao;
                heuristic_for_depth.at(T) = Prune(H_T);
                size_t nrVecs = heuristic_for_depth.at(T).size();
                cout << "done. (got " << nrVecs << " vectors)" << endl;
                //TODO: bail out when we cannot compute the POMDP solution.
                //_m_heuristic_mindepth = T; 
            }
        }
        _m_timing.Stop(ss.str());
        _m_timing.PrintSummary(); cout << endl;
    }
#endif


    AlphaVector alpha(nrS);
    
    QFunctionsDiscrete Q(nrA);

    _m_cacheMiss=0;
    _m_cacheHit=0;
    _m_pruneAfterUnionStats.clear();
    _m_pruneAfterCrossSumStats.clear();

    // Initialize a joint policy with all individual observations
    // mapping to UNSPECIFIED_ACTION
    vector<vector<Index> > jpol;
    for(Index k=0;k!=GetPU()->GetNrAgents();++k)
    {
        vector<Index> pol(GetPU()->GetNrObservations(k),UNSPECIFIED_ACTION);
        jpol.push_back(pol);
    }

    // We loop over all joint actions in the current time step
    for(GaoVectorSetIndex a=0;a!=nrA;a++)
    {
//#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
        cout << "\n\n--------------------------------------\n";
        cout << "-proceeding with a="<<a<<"-------------------\n";
        cout << "--------------------------------------\n\n";
//#endif
        // we don't start with CrossSum as the reward is already in Gaoa
        VectorSet Ga=UnionNode(a,0,jpol);
        // since we were supposed to start the tree with a CrossSum,
        // if we enable pruneAfterCrossSum (but not pruneAfterUnion),
        // we have to toss in an extra Prune here to be consistent.
// TODO: check if it's not better to start with CrossSumNode, even if
// we crosssum with an empty set
        if(_m_pruneAfterCrossSum && !_m_pruneAfterUnion)
            Ga=PruneAfterCrossSum(0, Ga);

        // we did not do any pruning in the tree at all, so in order
        // to still get a parsimonious representation we have to prune
        // the whole thing here
        if(!_m_pruneAfterCrossSum && !_m_pruneAfterUnion)
            Ga=PruneAfterUnion(0, Ga);

        AlphaVector::BGPolicyIndex betaI=-1; // NEED TO FIGURE OUT BETAI DOWN THE TREE SOMEHOW
        Q[a]=VectorSetToValueFunction(Ga,a,betaI);
        cout << SoftPrintStats();
    }

    // cleanup
    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
            for(GaoVectorSetIndex aPrime=0;aPrime!=nrA;aPrime++) //for each next joint a
                delete (*_m_Gaoa)[a][o][aPrime];

    _m_vectorSetCache.clear();

    cout << SoftPrintStats() << endl;
    SaveStats();

    return(Q);
}

VectorSet 
TreeIncPruneBGPlanner::UnionNode(Index a,
                                 Index depth,
                                 const vector<vector<Index> > &jpol)
{
    _m_union_nodes_at_level.at(depth)++;
    _m_nodes_visited++;

    VectorSet G;
    if(CheckCache_Union(a,depth,jpol,G))
    {
        // G has now been filled by the cache
    }
    else
    {
        vector<Index> os=GetObservationsFromDepth(depth);

#if DEBUG_TreeIncPruneBGPlannerVerbose
        cout << "UnionNode:    a=" << a << ",d=" << depth << "(obs="
             << SoftPrintVector(os) << "),jpol="
             << SoftPrintJpol(jpol) << endl;
#endif
        
        vector<Scope> jas=GetValidJointActions(depth,jpol);
#if 0 // it's easier to just go until the max depth
        // if there is only a single joint action possible, we reached a leaf node
        if(jas.size()==1)
            G=*GetGaoa(a,os,jas[0]);
#else
        if(depth==_m_maxDepth-1)
        {
            if(jas.size()>1)
                throw(E("TreeIncPruneBGPlanner::UnionNode should only a single joint action possible"));
            G=*GetGaoa(a,os,jas[0]);
            _m_nr_bottom_hits++;
#endif
        }
        else
        {
            for(Index i=0;i!=jas.size();++i)
            {
                // extend the current jpol with one of the valid joint
                // actions
                vector<vector<Index> > jpolNew=jpol;
                for(Index k=0;k!=GetPU()->GetNrAgents();++k)
                {
                    if(jpolNew[k][os[k]]==UNSPECIFIED_ACTION || jpolNew[k][os[k]]==jas[i][k])
                        jpolNew[k][os[k]]=jas[i][k];
                    else
                        throw(E("TreeIncPruneBGPlanner::UnionNode observation has already been specified (or differently)"));
                }
#if DEBUG_TreeIncPruneBGPlanner
                cout << SoftPrintJpol(jpol) << " + " << SoftPrintVector(jas[i])
                     << " = " << SoftPrintJpol(jpolNew) << endl;
#endif
                VectorSet G1=CrossSumNode(a, depth, jpolNew);
                
                // for the first joint action we don't need to the union
                if(i==0)
                    G=G1;
                else
                {
                    G=Union(G,G1);
                    // we might consider skipping this Prune()
                    if(_m_pruneAfterUnion)
                        G=PruneAfterUnion(depth, G);
                }
            }
        }

        AddToCache_Union(a, depth, jpol, G);
    }
    return(G);
}

VectorSet
TreeIncPruneBGPlanner::CrossSumNode(Index a,
                                    Index depth,
                                    const vector<vector<Index> > &jpol)
{
    _m_cs_nodes_at_level.at(depth)++;
    _m_nodes_visited++;

    VectorSet G;
    if(CheckCache_CrossSum(a,depth,jpol,G))
    {
        // G has now been filled by the cache
    }
    else
    {
        vector<Index> os=GetObservationsFromDepth(depth);

#if DEBUG_TreeIncPruneBGPlannerVerbose
        cout << "CrossSumNode: a=" << a << ",d=" << depth << "(obs="
             << SoftPrintVector(os) << "),jpol="
             << SoftPrintJpol(jpol) << endl;
#endif

        // figure out which a,o,aPrime we need to get
        
        vector<Index> as(jpol.size());
        for(Index k=0;k!=as.size();++k)
            as[k]=jpol[k][os[k]];
        VectorSet *Gaoa=GetGaoa(a,os,as);

        G=CrossSum(*Gaoa,
                   UnionNode(a,
                             depth+1, // we up the depth
                             jpol));
        if(_m_pruneAfterCrossSum)
            G=PruneAfterCrossSum(depth, G);
        AddToCache_CrossSum(a, depth, jpol, G);
    }

    return(G);
}

ValueFunctionPOMDPDiscrete TreeIncPruneBGPlanner::GetValueFunction(size_t horizon)
{
    return(QFunctionsToValueFunction(_m_qFunction[horizon-1]));
}

vector<Scope> TreeIncPruneBGPlanner::GetValidActions(
    Index depth,
    const vector<vector<Index> > &jpol) const
{
    size_t nrAgents=jpol.size();

    vector<Index> Os=GetObservationsFromDepth(depth);
    vector<Scope> as;
    for(Index k=0;k!=nrAgents;++k)
    {
        Scope aI;
        if(jpol[k][Os[k]]==UNSPECIFIED_ACTION)
            // we didn't specify this obs yet, so all actions are valid
            for(Index a=0;a!=GetPU()->GetNrActions(k);++a)
                aI.push_back(a);
        else // we already assigned this obs, so we can only use that
             // particular action
            aI.push_back(jpol[k][Os[k]]);
        as.push_back(aI);
    }

    return(as);
}

vector<Scope> TreeIncPruneBGPlanner::GetValidJointActions(
    Index depth,
    const vector<vector<Index> > &jpol) const
{
    size_t nrAgents=jpol.size();

    vector<Scope> as=GetValidActions(depth,jpol);

    // now construct the set of joint actions
    vector<Scope> jas;
    switch(nrAgents)
    {
    case 2:
    {
#if DEBUG_TreeIncPruneBGPlanner    
        cout << "GetValidJointActions for depth " << depth
             << " jpol " << SoftPrintJpol(jpol) << ":";
#endif

        for(Index a0=0;a0!=as[0].size();++a0)
            for(Index a1=0;a1!=as[1].size();++a1)
            {
                vector<Index> asI(nrAgents);
                asI[0]=as[0][a0];
                asI[1]=as[1][a1];
                jas.push_back(asI);
#if DEBUG_TreeIncPruneBGPlanner    
                cout << SoftPrintVector(asI);
#endif
            }
#if DEBUG_TreeIncPruneBGPlanner    
        cout << endl;
#endif
        break;
    }
    case 3:
    {
        for(Index a0=0;a0!=as[0].size();++a0)
            for(Index a1=0;a1!=as[1].size();++a1)
                for(Index a2=0;a2!=as[2].size();++a2)
                {
                    vector<Index> asI(nrAgents);
                    asI[0]=as[0][a0];
                    asI[1]=as[1][a1];
                    asI[2]=as[2][a2];
                    jas.push_back(asI);
                }
        break;
    }
    case 4:
    {
        for(Index a0=0;a0!=as[0].size();++a0)
            for(Index a1=0;a1!=as[1].size();++a1)
                for(Index a2=0;a2!=as[2].size();++a2)
                    for(Index a3=0;a3!=as[3].size();++a3)
                    {
                        vector<Index> asI(nrAgents);
                        asI[0]=as[0][a0];
                        asI[1]=as[1][a1];
                        asI[2]=as[2][a2];
                        asI[3]=as[3][a3];
                        jas.push_back(asI);
                    }
        break;
    }
    default:
        throw(E("TreeIncPruneBGPlanner::GetValidJointActions this number of agents"));
    }

    return(jas);
}

vector<Index> TreeIncPruneBGPlanner::GetObservationsFromDepth(Index depth) const
{
    // here we just enumerate all joint observations in their original
    // order, so the jo index is equal to the depth
    Index jo=depth;
    vector<Index> os=GetPU()->JointToIndividualObservationIndices(jo);
#if DEBUG_TreeIncPruneBGPlanner
    cout << "Depth " << depth << " Obs " << SoftPrintVector(os) << endl;
#endif
    return(os);
}

string TreeIncPruneBGPlanner::SoftPrintJpol(
    const vector<vector<Index> > &jpol) const
{
    stringstream ss;
    for(Index k=0;k!=jpol.size();++k)
    {
        ss << "[";
        for(Index o=0;o!=jpol[k].size();++o)
        {
            if(jpol[k][o]==UNSPECIFIED_ACTION)
                ss << "U";
            else
                ss << jpol[k][o];
        }
        ss << "]";
    }
    
    return(ss.str());
}

string 
TreeIncPruneBGPlanner::ComputeKey(Index a,
                                  Index depth, 
                                  const vector<vector<Index> > &jpol,
                                  string extraIdentifier) const
{
    stringstream ss;

    ss << "a" << a << "d" << depth << extraIdentifier; //<< SoftPrintJpol(jpol);
        
    // generate 'still possible action key', e.g.:
    //      ( <S,S>, <S,S>, <*,l>, <r,l> )
    // to form total key:
    //   (a, depth, <S,S>, <S,S>, <*,l>, <r,l> )
    for(Index d=depth;d!=_m_maxDepth;++d)
    {
        vector<Scope> asD=GetValidActions(d, jpol);
        ss << "[";
        for(Index k=0;k!=GetPU()->GetNrAgents();++k)
        {
            if(asD[k].size()==GetPU()->GetNrActions(k))
                ss << "*";
            else if(asD[k].size()==1)
                ss << asD[k][0];
            else
                throw(E("ComputeKey: strange number of valid actions found"));
        }
        ss << "]";
    }

#if 0
    // we start by creating the full action set for each agent
    vector<Scope> as;
    for(Index k=0;k!=jpol.size();++k)
    {
        Scope aI;
        for(Index a=0;a!=GetPU()->GetNrActions(k);++a)
            aI.push_back(a);
        as.push_back(aI);
    }

    // now, for all joint observations that are still unspecified, we
    // figure which actions are valid, taking the intersection of them
    for(Index d=depth;d!=_m_maxDepth;++d)
    {
        vector<Scope> asD=GetValidActions(d, jpol);
#if 0
        for(Index k=0;k!=as.size();++k)
            cout << as[k].SoftPrint();
        cout << endl;
        for(Index k=0;k!=as.size();++k)
            cout << asD[k].SoftPrint();
        cout << endl;
#endif        
        for(Index k=0;k!=jpol.size();++k)
            as[k]=Scope::Intersection(as[k],asD[k]);
    }

    //ss << "D" << d;
    for(Index k=0;k!=as.size();++k)
        ss << as[k].SoftPrint();
#endif

//     {
//         ss << "[";
//         for(Index a=0;a!=as[k].size();++a)
//             ss << as[k][a];
//         ss << "]";
//     }

    return(ss.str());
}

bool TreeIncPruneBGPlanner::CheckCache_CrossSum(Index a,
                                                Index depth,
                                                const vector<vector<Index> > &jpol,
                                                VectorSet &G)
{
    if(!_m_useVectorCache)
        return(false);

    string key=ComputeKey(a,depth,jpol,"CrossSum");

    map<string,VectorSet>::iterator it;

    // check if the key is already in the cache
    it=_m_vectorSetCache.find(key);
    if(it==_m_vectorSetCache.end())
    {
#if DEBUG_TreeIncPruneBGPlanner
        cout << "CheckCache: CrossSum node key " << key << " not found" << endl;
#endif
        _m_cacheMiss++;
        return(false);
    }
    else
    {
#if DEBUG_TreeIncPruneBGPlannerVerbose
        cout << "CheckCache: CrossSum node key " << key << " found (jpol "
             << SoftPrintJpol(jpol) << ")" << endl;
#endif
        G=it->second;
    }
    _m_cacheHit++;
    return(true);
}


bool TreeIncPruneBGPlanner::CheckCache_Union(Index a,
                                             Index depth,
                                             const vector<vector<Index> > &jpol,
                                             VectorSet &G)
{
    if(!_m_useVectorCache)
        return(false);

    string key=ComputeKey(a,depth,jpol,"Union");

    map<string,VectorSet>::iterator it;

    // check if the key is already in the cache
    it=_m_vectorSetCache.find(key);
    if(it==_m_vectorSetCache.end())
    {
#if DEBUG_TreeIncPruneBGPlanner
        cout << "CheckCache: Union node " << key << " not found" << endl;
#endif
        _m_cacheMiss++;
        return(false);
    }
    else
    {
#if DEBUG_TreeIncPruneBGPlannerVerbose
        cout << "CheckCache: Union node " << key << " found (jpol "
             << SoftPrintJpol(jpol) << ")" << endl;
#endif
        G=it->second;

#if 0
        VectorSet G1=CrossSumNode(a, depth, jpol);
        if(!AlphaVectorPlanning::EqualVS(G,G1))
        {
            stringstream ss;
            ss << "TreeIncPruneBGPlanner::CheckCache sets are not equal: "
               << SoftPrint(G) << " vs " << SoftPrint(G1) << endl;
            throw(E(ss));
        }
#endif
    }
    _m_cacheHit++;
    return(true);
}

void TreeIncPruneBGPlanner::AddToCache_CrossSum(Index a,
                                                Index depth,
                                                const vector<vector<Index> > &jpol,
                                                const VectorSet &G)
{
    if(!_m_useVectorCache)
        return;

    string key=ComputeKey(a,depth,jpol,"CrossSum");

#if DEBUG_TreeIncPruneBGPlannerVerbose
    cout << "Adding CrossSum node to cache with key " << key 
#endif
#if 0
         << " vectorset " << SoftPrint(G) 
#endif
#if DEBUG_TreeIncPruneBGPlannerVerbose
         << " (jpol " << SoftPrintJpol(jpol) << ")" << endl;
#endif
    _m_vectorSetCache[key]=G;//.insert(std::pair<key,G>);
}

void TreeIncPruneBGPlanner::AddToCache_Union(Index a,
                                                   Index depth,
                                                   const vector<vector<Index> > &jpol,
                                                   const VectorSet &G)
{
    if(!_m_useVectorCache)
        return;

    string key=ComputeKey(a,depth,jpol,"Union");

#if DEBUG_TreeIncPruneBGPlannerVerbose
    cout << "Adding Union Node to cache with key " << key 
#endif
#if 0
         << " vectorset " << SoftPrint(G)
#endif
#if DEBUG_TreeIncPruneBGPlannerVerbose
         << " (jpol " << SoftPrintJpol(jpol) << ")" << endl;
#endif
    _m_vectorSetCache[key]=G;//.insert(std::pair<key,G>);
}

string TreeIncPruneBGPlanner::SoftPrintBrief() const
{
    stringstream ss;
    ss << "TreeIncPruneU" << _m_pruneAfterUnion
       << "CS" << _m_pruneAfterCrossSum
       << "VC" << _m_useVectorCache
       << "BG";
    return(ss.str());
}

VectorSet TreeIncPruneBGPlanner::PruneAfterUnion(Index depth,
                                                 const VectorSet &G)
{
    VectorSet G1=Prune(G);
#if DEBUG_TreeIncPruneBGPlanner_KeepPruneStats
    vector<size_t> pruneStat(3);
    pruneStat[0]=depth;
    pruneStat[1]=G.size1();
    pruneStat[2]=G1.size1();
    _m_pruneAfterUnionStats.push_back(pruneStat);
#endif
    return(G1);
}

VectorSet TreeIncPruneBGPlanner::PruneAfterCrossSum(Index depth,
                                                    const VectorSet &G)
{
    VectorSet G1=Prune(G);
#if DEBUG_TreeIncPruneBGPlanner_KeepPruneStats
    vector<size_t> pruneStat(3);
    pruneStat[0]=depth;
    pruneStat[1]=G.size1();
    pruneStat[2]=G1.size1();
    _m_pruneAfterCrossSumStats.push_back(pruneStat);
#endif
    return(G1);
}

string TreeIncPruneBGPlanner::SoftPrintStats() const
{
    stringstream ss;
    ss << "Cache hits " << _m_cacheHit << " misses " << _m_cacheMiss
       << " ratio " << static_cast<double>(_m_cacheHit)/(_m_cacheHit+_m_cacheMiss) << endl;
#if 0     
    ss << "Prune after Union stats (size before, size after):";
    for(Index k=0; k!=_m_pruneAfterUnionStats.size();++k)
        ss << " (" << _m_pruneAfterUnionStats[k][1] << ","
           << _m_pruneAfterUnionStats[k][2] << ")";
    ss << endl;
    ss << "Prune after CrossSum stats (size before, size after):";
    for(Index k=0; k!=_m_pruneAfterCrossSumStats.size();++k)
        ss << " (" << _m_pruneAfterCrossSumStats[k][1] << ","
           << _m_pruneAfterCrossSumStats[k][2] << ")";
    ss << endl;
#endif

    double nr_jp = 1;
    for(Index agI=0; agI < GetPU()->GetNrAgents(); agI++)
    {
        double nrAcsI = (double) GetPU()->GetNrActions(agI);
        double nrObsI = (double) GetPU()->GetNrObservations(agI);
        double pols_agI = (size_t) pow(nrAcsI, nrObsI);
        nr_jp *= pols_agI;
    }
    size_t nrJA = GetPU()->GetNrJointActions();
    ss << "nr joint BG pols:   " << nr_jp << ", nr JA=" << nrJA;
    ss << "-> number of expected bottom leafs = " << nr_jp * nrJA <<  endl;
    ss << "_m_nr_bottom_hits:  " << _m_nr_bottom_hits << endl;
    //ss << "_m_nr_backtracks:   " << _m_nr_backtracks << endl;
    ss << "_m_nodes_visited:   " << _m_nodes_visited << endl;
    ss << "depth  union nodes  cs nodes" << endl;
    for(Index d=0; d < _m_maxDepth; d++)
    {
#if 0 // the following doesn't compile with arbitrary length integers
        char s[50];
        sprintf(s, "%2d     %6u     %6u\n", d, _m_union_nodes_at_level.at(d), _m_cs_nodes_at_level.at(d) );
        ss << s;
#else
        ss <<  d << "        " 
           << _m_union_nodes_at_level.at(d) << "        " 
           << _m_cs_nodes_at_level.at(d) << endl;
#endif
    }
    
    return(ss.str());
}

void TreeIncPruneBGPlanner::SaveStats() const
{
    stringstream ss;
    ss << directories::MADPGetResultsDir("GMAA",*GetPU()->GetDPOMDPD())
       << "/calculateQheuristic" << SoftPrintBrief() << "_h"
       << GetPU()->GetHorizon();
    if(GetPU()->GetDiscount()!=1)
        ss << "_g" <<GetPU()->GetDiscount();
    ss <<  "_t" << GetTimeStep();
    string baseFilename=ss.str();

    string filename=baseFilename + string("_generalStats");
    {
        double nr_jp = 1;
        for(Index agI=0; agI < GetPU()->GetNrAgents(); agI++)
        {
            double nrAcsI = (double) GetPU()->GetNrActions(agI);
            double nrObsI = (double) GetPU()->GetNrObservations(agI);
            double pols_agI = (size_t) pow(nrAcsI, nrObsI);
            nr_jp *= pols_agI;
        }
        size_t nrJA = GetPU()->GetNrJointActions();

        ofstream fp(filename.c_str());
        if(!fp)
        {
            cerr << "TreeIncPruneBGPlanner::SaveStats: failed to open file "
                 << ss.str() << endl;            
        }
        fp << "#  Cache hits, cache misses, nr joint BG pols, nrJA, nr expected bottom leafs, "
           << "nr bottom hits, nr nodes visited" << endl
           << _m_cacheHit << " " << _m_cacheMiss
           << " " << nr_jp << " " << nrJA << " " << nr_jp * nrJA
           << " " << _m_nr_bottom_hits << " " << _m_nodes_visited << endl;
    }
#if DEBUG_TreeIncPruneBGPlanner_KeepPruneStats
    // this usually uses too much disk space (and time to write the files...)
    filename=baseFilename + string("_pruneAfterUnion");
    {
        ofstream fp(filename.c_str());
        if(!fp)
        {
            cerr << "TreeIncPruneBGPlanner::SaveStats: failed to open file "
                 << ss.str() << endl;            
        }
        fp << "# Prune after Union stats (depth, size before, size after):" << endl;
        for(Index k=0; k!=_m_pruneAfterUnionStats.size();++k)
            fp << _m_pruneAfterUnionStats[k][0] << " "
               << _m_pruneAfterUnionStats[k][1] << " "
               << _m_pruneAfterUnionStats[k][2] << endl;
    }

    filename=baseFilename + string("_pruneAfterCrossSum");
    {
        ofstream fp(filename);
        if(!fp)
        {
            cerr << "TreeIncPruneBGPlanner::SaveStats: failed to open file "
                 << ss.str() << endl;            
        }
        fp << "# Prune after CrossSum stats (depth, size before, size after):" << endl;
        for(Index k=0; k!=_m_pruneAfterCrossSumStats.size();++k)
            fp << _m_pruneAfterCrossSumStats[k][0] << " "
               << _m_pruneAfterCrossSumStats[k][1] << " "
               << _m_pruneAfterCrossSumStats[k][2] << endl;
    }
#endif
}        
