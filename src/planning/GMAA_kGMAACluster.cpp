/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <float.h>
#include <numeric>
#include "Globals.h"
#include "GMAA_kGMAACluster.h"
#include "PartialJPDPValuePair.h"
#include "PolicyPoolPartialJPolValPair.h"
#include "JointPolicyPureVectorForClusteredBG.h"
#include "BayesianGameWithClusterInfo.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "BGIP_SolverCreatorInterface_T.h"

#define DEBUG_GMAA_EM 0

using namespace std;

GMAA_kGMAACluster::GMAA_kGMAACluster(
    const PlanningUnitMADPDiscreteParameters &params,
    //const BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * bgs,
    const BGIP_SolverCreatorInterface * bgs,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p,
    size_t nrPoliciesToProcess,
    BayesianGameWithClusterInfo::BGClusterAlgorithm clusterAlg
    ) :
//    PlanningUnitDecPOMDPDiscrete(params, horizon, p), //virtual base must be called directly
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(params, horizon, p),
    /// HACK, this should be BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG>*
    _m_newBGIP_Solver(dynamic_cast<const BGIP_IncrementalSolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG>* >(bgs)),
    _m_clusteredBGsizes(horizon,vector<int>(0,0)),
    _m_clusterStatsFilename(""),
    _m_clusterAlg(clusterAlg),
    //_m_thresholdJB(0),
    //_m_thresholdPjaoh(0),
    _m_dummyBG(new BayesianGameWithClusterInfo(this))
{
    if(!_m_newBGIP_Solver)
        throw E("GMAA_kGMAACluster needs a const BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG>* ");

    _m_nrPoliciesToProcess=nrPoliciesToProcess;
}

GMAA_kGMAACluster::~GMAA_kGMAACluster()
{
    ResetPlanner();
}

void GMAA_kGMAACluster::ResetPlanner()
{
    std::vector< JointPolicyPureVectorForClusteredBG* >::iterator it = 
        _m_jpolCache.begin();
    std::vector< JointPolicyPureVectorForClusteredBG* >::iterator last = 
        _m_jpolCache.end();
    while(it != last)
    {
        delete *it;
        it++;
    }
    _m_jpolCache.clear();

    _m_clusteredBGsizes=vector<vector<int> >(GetHorizon(),vector<int>(0,0));

}

bool GMAA_kGMAACluster::ConstructAndValuateNextPolicies(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi,
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
        bool &cleanUpPPI)
{
    cleanUpPPI=true;
    //cout << "ConstructAndValuateNextPoliciesExactBG ";
    double pastReward_prevTs = 0.0;
    BGwCI_sharedPtr bg_ts;
    boost::shared_ptr<PartialJointPolicyDiscretePure> pP = ppi->GetJPol();

    Index ts = pP->GetDepth(); //bg_ts->GetStage();

    boost::shared_ptr<JointPolicyPureVectorForClusteredBG> pJPolBG;
    if(ts == 0)
    {
        //first stage, build BG from scratch.
        //also here we need to store the past policy
        boost::shared_ptr<PartialJointPolicyPureVector> pPcopy =
            boost::shared_ptr<PartialJointPolicyPureVector>(new PartialJointPolicyPureVector(*pP));
        pP = pPcopy;

        bg_ts = BGwCI_sharedPtr(new BayesianGameWithClusterInfo(this, _m_qHeuristic, pP, _m_clusterAlg));
        //bg_ts->SetThresholdJB(_m_thresholdJB);
        //bg_ts->SetThresholdPjaoh(_m_thresholdPjaoh);
        //cout << "constucted BG for stage 0 "<< endl;
        //cout << bg_ts->SoftPrint() << endl;
    }
    else
    {

        try{
            pJPolBG = 
                boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(pP);
            /*
            cout << "Got policy from policy pool:"<<endl;
            cout << pJPolBG->SoftPrint() << endl;
            const BayesianGameWithClusterInfo* bgtemp = pJPolBG->GetBG();
            if(bgtemp != 0)
            {
                cout << "for the following BG:" <<endl; 
                cout << bgtemp->SoftPrint()<<endl;
            }
            */
        }catch(exception& e){
            cerr << "GMAA_kGMAACluster::ConstructAndValuateNextPolicies - dynamic cast failed! Exception:" << e.what() << endl;
        }
        if(pJPolBG == 0)
        {
            throw E("GMAA_kGMAACluster::ConstructAndValuateNextPolicies - dynamic cast failed!"); 
        }

        //when this function returns, ppi and the policy pointed to will be 
        //deleted by GeneralizedMAAStarPlannerForDecPOMDPDiscrete.
        //However, we need to preserve the past (bg) policy to reconstruct the
        //final policy!
        //As a solution, we make an actual copy here.
        boost::shared_ptr<JointPolicyPureVectorForClusteredBG> pPcopy = 
            boost::shared_ptr<JointPolicyPureVectorForClusteredBG>(
                new JointPolicyPureVectorForClusteredBG(*pJPolBG));
//        _m_jpolCache.push_back(pPcopy);
        pJPolBG = pPcopy;

        pastReward_prevTs = pJPolBG->GetPastReward();
        BGwCI_constPtr pBG = pJPolBG->GetBG();
        BGwCI_sharedPtr newBG =
            BayesianGameWithClusterInfo::ConstructExtendedBGWCI(pBG,*pJPolBG, _m_qHeuristic);
        bg_ts = newBG->Cluster();
        _m_clusteredBGsizes.at(bg_ts->GetStage()).push_back(bg_ts->GetNrJointTypes());
        if(_m_clusterStatsFilename!="")
            SaveClusterStats(_m_clusterStatsFilename);

    }
    
    if(_m_verboseness >= 2) {
        cout << "About to solve the following BG:"<< endl;
        cout << bg_ts->SoftPrint() << endl;
    }

    //size_t depth = jpolPrevTs->GetDepth(); // = depth = ts(jpolPrevTs) + 1
    //size_t ts = depth; //jpol = jpol^ts-1, we construct BG for ts == depth
    bool is_last_ts = (ts ==  GetHorizon() - 1);

    //the policy for the Bayesian game 
//     JointPolicyPureVectorForClusteredBG jpolBG = 
//         JointPolicyPureVectorForClusteredBG(*bg_ts, pJPolBG);

//     jpolBG.SetDepth(ts+1);


    //solve the Bayesian game
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver> bgips =
        boost::shared_ptr<BayesianGameIdenticalPayoffSolver>(
                _m_newBGIP_Solver->operator()(bg_ts)
                );
//     BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVectorForClusteredBG> * bgips =
//         (*_m_newBGIP_Solver)(bg_ts);
    bgips->Solve();
//    BGIPSolution & solution = bgips->GetSolution();
    
    LIndex nrJPols=0;
    try {
        nrJPols = bg_ts->GetNrJointPolicies();
    }
    catch(EOverflow& e){ 
    }
    if(_m_verboseness >= 1) 
        cout << "starting on solution of BG for t="<<ts<<" with nrJPols="
             <<nrJPols<<endl;

    stringstream ss;
    char ts_str[5];
    sprintf(ts_str, "%03d",ts);
    ss << "GMAA_kGMAACluster::Next"
       << BayesianGameWithClusterInfo::SoftPrint(_m_clusterAlg)
       << "_ts" << ts_str;
    StartTimer(ss.str());

    //some variables used when this is the last time-step
    // a copy, which is only used for the last stage
    JointPolicyPureVectorForClusteredBG bestLBjpolBG = 
        JointPolicyPureVectorForClusteredBG(bg_ts);

    //for each solution in BGIPSolution  
    bg_ts->ComputeAllImmediateRewards();
    for(Index solI=0; solI < _m_nrPoliciesToProcess; solI++)
    {
        if(bgips->IsEmptyPJPDP())
        {
            cerr << "Warning, BGIP_Solver only returned "<<solI<<
                " usable joint policies"<<endl;
            break;
        }
        const boost::shared_ptr<PartialJPDPValuePair> jpvp =
            bgips->GetNextSolutionPJPDP();
        boost::shared_ptr<PartialJointPolicyDiscretePure> bgpol =
            boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(jpvp->GetJPol());
        double val = jpvp->GetValue();

        bgips->PopNextSolutionPJPDP();
        boost::shared_ptr<JointPolicyPureVectorForClusteredBG> jpolBGcopy = 
            boost::shared_ptr<JointPolicyPureVectorForClusteredBG>(
                new JointPolicyPureVectorForClusteredBG(*boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(bgpol)));
        jpolBGcopy->SetDepth(ts+1);

        //compute expected immediate reward for this stage
        double immR = bg_ts->ComputeDiscountedImmediateRewardForJPol(bgpol);
        double newPastreward = pastReward_prevTs + immR;
        jpolBGcopy->SetPastReward(newPastreward);
        
        //push this policy and value on the priority queue
        ////if last stage, if so, we want to return the 
        ////*EXACT* past reward, newPastreward.
        if(is_last_ts)
            poolOfNextPolicies->Insert( NewPPI(jpolBGcopy,newPastreward) );
        else
            poolOfNextPolicies->Insert( NewPPI(jpolBGcopy,
                                        val + pastReward_prevTs) );
//        delete jpvp;
    }
    //empty the imm reward cache
    bg_ts->ClearAllImmediateRewards();


    //NOTE: we can not delete this BG now... we should create some 
    //administration to see if a BG can be removed at some point
    //delete(bg_ts);
    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);
}

void GMAA_kGMAACluster::SelectPoliciesToProcessFurther(
    const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies, bool are_LBs, double bestLB)
{
    SelectKBestPoliciesToProcessFurther(poolOfNextPolicies, are_LBs, 
                                        bestLB, _m_nrPoliciesToProcess);
}

PartialPolicyPoolItemInterface_sharedPtr
GMAA_kGMAACluster::NewPPI(const boost::shared_ptr<PartialJointPolicyDiscretePure> &jp, double v) const
{
    //the index variant can not store a JointPolicyPureVectorForClusteredBG!
    //PartialPolicyPoolItemInterface* ppi=new JPPVIndexValPair(jp,v);
    //delete jp;//We can't delete the joint policy!
    PartialPolicyPoolItemInterface_sharedPtr ppi=
        PartialPolicyPoolItemInterface_sharedPtr( new PartialJPDPValuePair(jp,v) );
    return (ppi); 
}
        

boost::shared_ptr<PartialJointPolicyDiscretePure>
GMAA_kGMAACluster::NewJPol() const
{ 
    boost::shared_ptr<PartialJointPolicyDiscretePure> jp=
        boost::shared_ptr<PartialJointPolicyDiscretePure>(
            new JointPolicyPureVectorForClusteredBG(_m_dummyBG));
    jp->SetDepth(0);
    return jp; 
}


std::string GMAA_kGMAACluster::SoftPrintClusteringStats() const
{
    stringstream ss;

    ss << "Number of joint types after clustering in each BG." << endl;
    ss << "timestep compressionRatio averageAfterClustering sizeWithoutClustering <all entries>" << endl;

#if 0
    ss << "CBGsize";
    for(unsigned int i=0;i!=_m_clusteredBGsizes.size();++i)
    {
        double average=accumulate(_m_clusteredBGsizes[i].begin(),
                                  _m_clusteredBGsizes[i].end(),0.0) /
            _m_clusteredBGsizes[i].size();
        if(_m_clusteredBGsizes[i].size()>0)
            ss << " " << average;
    }
    ss << endl;

    ss << "CBGratio";
#endif

    for(unsigned int i=0;i!=_m_clusteredBGsizes.size();++i)
    {
        if(_m_clusteredBGsizes[i].size()>0)
        {
            double average=accumulate(_m_clusteredBGsizes[i].begin(),
                                      _m_clusteredBGsizes[i].end(),0.0) /
                _m_clusteredBGsizes[i].size();

            LIndex originalBGsize=1;
            for(Index aI=0;aI!=GetNrAgents();++aI)
                originalBGsize*=pow(//pow needs correct arguments, otherwise I get 
                    //call of overloaded 'pow(size_t, unsigned int&)' is ambiguous
                    ((double)GetNrObservations(aI)),
                    ((double)i));
            
#if 1
            ss << i
               << " " << CastLIndexToDouble(originalBGsize)/average
               << " " << average
               << " " << originalBGsize
               << " " << SoftPrintVector(_m_clusteredBGsizes[i]) << endl;
#else
            ss << " " << static_cast<double>(originalBGsize)/average;
#endif
        }
    }
    return(ss.str());
}

void GMAA_kGMAACluster::SaveClusterStats(string filename) const
{
    ofstream fp(filename.c_str());
    if(!fp)
        cerr << "GMAA_kGMAACluster::SaveClusterStats failed to open file "
             << filename << endl;

    fp << SoftPrintClusteringStats() << endl;
}

