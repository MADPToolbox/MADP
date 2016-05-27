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

#include <float.h>
#include <numeric>
#include "Globals.h"
#include "GMAA_MAAstarCluster.h"
#include "PartialJPDPValuePair.h"
#include "PolicyPoolPartialJPolValPair.h"
#include "JointPolicyPureVectorForClusteredBG.h"
#include "BGIP_SolverCreator_BFS.h"
#include "boost/make_shared.hpp"

#define DEBUG_GMAA_EM 0

// To test the clustering performance, keep a policy pool of size one
// with a random solution.
#define GMAA_MAAstarCluster_TestClustering 0

#if GMAA_MAAstarCluster_TestClustering
#include "BGIP_SolverRandom.h"
#endif

using namespace std;

GMAA_MAAstarCluster::GMAA_MAAstarCluster(
    const PlanningUnitMADPDiscreteParameters &params,
    //const BGIP_IncrementalSolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * bgsc,
    const BGIP_SolverCreatorInterface * bgsc,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p,
    int verboseness
    ) :
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(params, horizon, p, verboseness),
    _m_newBGIP_Solver(
            dynamic_cast<const BGIP_IncrementalSolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG>* >(bgsc)
            ),
    _m_clusteredBGsizes(horizon,vector<int>(0,0)),
    _m_clusterStatsFilename(""),
    _m_dummyBG(new BayesianGameWithClusterInfo(this))
{
    // check if we were passed a BGsolver
    if(_m_newBGIP_Solver)
    {
        if(!_m_newBGIP_Solver->IsExactSolver())
            throw(E("GMAA_MAAstarCluster requires an exact BG solver"));
    }
    //else // otherwise instantiate one
    //{
        //_m_newBGIP_Solver= new BGIP_SolverCreator_BFS<JointPolicyPureVectorForClusteredBG>();
    //}
};

GMAA_MAAstarCluster::GMAA_MAAstarCluster(
    const BGIP_IncrementalSolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * bgsc,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p) :
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(horizon, p),
    _m_newBGIP_Solver(bgsc),
    _m_clusteredBGsizes(horizon,vector<int>(0,0)),
    _m_clusterStatsFilename(""),
    _m_dummyBG(new BayesianGameWithClusterInfo(this))
{
    // check if we were passed a BGsolver
    if(_m_newBGIP_Solver)
    {
        if(!_m_newBGIP_Solver->IsExactSolver())
            throw(E("GMAA_MAAstarCluster requires an exact BG solver"));
    }
    else // otherwise instantiate one
    {
        _m_newBGIP_Solver=
            new BGIP_SolverCreator_BFS<JointPolicyPureVectorForClusteredBG>();
    }
}

GMAA_MAAstarCluster::~GMAA_MAAstarCluster()
{
    ResetPlanner();
}

void GMAA_MAAstarCluster::ResetPlanner()
{
    _m_clusteredBGsizes=vector<vector<int> >(GetHorizon(),vector<int>(0,0));
}

bool GMAA_MAAstarCluster::ConstructAndValuateNextPolicies(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi,
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
        bool &cleanUpPPI)
{
    PJPDP_sharedPtr pP = ppi->GetJPol();
    Index ts = pP->GetDepth();
    bool is_last_ts = (ts ==  GetHorizon() - 1);

    BGwCI_constPtr bg_ts;
    boost::shared_ptr<BGIP_IncrementalSolverInterface_T<JointPolicyPureVectorForClusteredBG> > bgips;
    //boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVectorForClusteredBG> > bgipsNonIncremental;
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver> bgipsNonIncremental;

    JPPVfCBG_sharedPtr pJPolBG;

    if(ppi->GetBGIPSolverPointer()==0)
    {
        if(_m_verboseness >= 8) 
            cout << "GMAA_MAAstarCluster: I have not expanded this ppi before..." << endl;

        _m_bgCounter++;
        if(ts == 0)
        {
            //first stage, build BG from scratch.
            //also here we need to store the past policy
            PJPPV_sharedPtr pPcopy = boost::make_shared<PartialJointPolicyPureVector>(*pP);
            pP = pPcopy;

// Apparently make_shared works differently on Ubuntu 11.10, so call shared_ptr directly
//            bg_ts = boost::make_shared<BayesianGameWithClusterInfo>(this, _m_qHeuristic, pP);
            bg_ts = boost::shared_ptr<BayesianGameWithClusterInfo>(new BayesianGameWithClusterInfo(this, _m_qHeuristic, pP));
        }
        else
        {
            pJPolBG = 
                boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(pP);
            if(pJPolBG == 0)
                throw E("GMAA_MAAstarCluster::ConstructAndValuateNextPolicies - dynamic cast failed!"); 
            
            //when this function returns, ppi and the policy pointed to will be 
            //deleted by GeneralizedMAAStarPlannerForDecPOMDPDiscrete.
            //However, we need to preserve the past (bg) policy to reconstruct the
            //final policy!
            //As a solution, we make an actual copy here.
            JPPVfCBG_sharedPtr pPcopy = boost::make_shared<JointPolicyPureVectorForClusteredBG>(*pJPolBG);
            pJPolBG = pPcopy;

            BGwCI_constPtr pBG = pJPolBG->GetBG();
            BGwCI_sharedPtr newBG = 
                BayesianGameWithClusterInfo::ConstructExtendedBGWCI(pBG,*pJPolBG, _m_qHeuristic);
            
            if(_m_bgBaseFilename!="")
            {
                stringstream ssBefore;
                ssBefore << _m_bgBaseFilename << "beforeCluster"
                         << setw(6) << setfill('0')
                         << _m_bgCounter << "_ts"
                         << setw(3) << setfill('0') << ts;
                BayesianGameIdenticalPayoff::Save(*newBG,ssBefore.str());
            }

            {
                stringstream ss;
                ss << "GMAA_MAAstarCluster::BGCluster_ts" 
                   << setw(3) << setfill('0') << ts;
                StartTimer(ss.str());
                // do the clustering
                bg_ts = newBG->Cluster();
                StopTimer(ss.str());
            }

            _m_clusteredBGsizes.at(
                bg_ts->GetStage()).push_back(bg_ts->GetNrJointTypes());
            if(_m_clusterStatsFilename!="")
                SaveClusterStats(_m_clusterStatsFilename);
        
            if(_m_bgBaseFilename!="")
            {
                stringstream ssAfter;
                ssAfter << _m_bgBaseFilename << "afterCluster"
                         << setw(6) << setfill('0')
                         << _m_bgCounter << "_ts"
                         << setw(3) << setfill('0') << ts;
                BayesianGameIdenticalPayoff::Save(*bg_ts,ssAfter.str());
            }
        }

        // keep track of every bg_ts we instantiate, so we can clean
        // them up after solving
//        _m_pointersToAllBGTS.push_back(bg_ts);

        // create the BGIPSolver
        // _m_newBGIP_Solver is pointer to a BGSolverCreator, here this functor is called:
        bgipsNonIncremental =
            boost::shared_ptr<BayesianGameIdenticalPayoffSolver >(
                    _m_newBGIP_Solver->operator()(bg_ts) );

        //check if it can be converted to an incremental solver:
        bgips = 
            boost::dynamic_pointer_cast<BGIP_IncrementalSolverInterface_T<JointPolicyPureVectorForClusteredBG> > (bgipsNonIncremental);

        if(bgips==0)
            throw(E("GMAA_MAAstarCluster requires BGIP_IncrementalSolverInterface_T solvers"));
        if(is_last_ts)
        {
            //For the last stage we only want 1 solution.
            bgips->SetNrDesiredSolutions(1);
        }
        else if(bgips->GetNrDesiredSolutions()!=INT_MAX)
            throw(E("GMAA_MAAstar requires a BGIP solver to return all solutions (when requested)"));

        // associate the resulting incremental BGIPSolver with this
        // PPI, so that we can reuse it
        //ppi->SetBGIPSolver_T_Pointer(bgips);
        ppi->SetBGIPSolverPointer( boost::static_pointer_cast<BayesianGameIdenticalPayoffSolver>(bgips) );
    }
    else // we already have the solver:
    {
        if(_m_verboseness >= 8) 
            cout << "GMAA_MAAstarCluster: ppi was expanded before, getting the previous BGIPSolver..." << endl;

        //bgipsNonIncremental=ppi->GetBGIPSolver_T_PointerCluster();
        bgipsNonIncremental = ppi->GetBGIPSolverPointer();
        bgips = boost::dynamic_pointer_cast<BGIP_IncrementalSolverInterface_T<JointPolicyPureVectorForClusteredBG> >(bgipsNonIncremental);
        if(bgips == 0)
            throw(E("GMAA_MAAstarCluster requires BGIP_IncrementalSolverInterface_T solvers"));
        bg_ts = boost::dynamic_pointer_cast<const BayesianGameWithClusterInfo>(bgips->GetBGIPI());
        if(bg_ts == 0)
            throw(E("GMAA_MAAstarCluster BG is not of the correct type"));
    }

    if(_m_verboseness >= 2) {
        cout << "About to solve the following BG:"<< endl;
        cout << bg_ts->SoftPrint() << endl;
    }

    double pastReward_prevTs = pP->GetPastReward();

    stringstream ss;
    ss << "GMAA_MAAstarCluster::NextExact_ts" << setw(3) << setfill('0') << ts;
    StartTimer(ss.str());
    
    // Compute bounds on the value solving this CBG can result in
    SetCBGbounds(ppi,bgipsNonIncremental);

    // a copy, which is only used for the last stage
    JointPolicyPureVectorForClusteredBG bestLBjpolBG = 
        JointPolicyPureVectorForClusteredBG(bg_ts);

    // Ask the BGIPSolver for the next solution, which might not exist
    // due to the bounds on the value of the solution
    double val;
    boost::shared_ptr<JointPolicyDiscretePure> bgpol_jpdp;
    /// Calling the BGIPSolver
    bool foundCBGsolution = bgips->GetNextJointPolicyAndValue(bgpol_jpdp, val);

    if(foundCBGsolution)
    {
        JPPVfCBG_sharedPtr bgpol = boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(bgpol_jpdp);
        if(bgpol==0)
            throw(E("GMAA_MAAstarCluster: did not get a valid policy from the incremental BGIP solver"));
        double immR = bg_ts->ComputeDiscountedImmediateRewardForJPol(bgpol);
        
        JPPVfCBG_sharedPtr jpolBGcopy = boost::make_shared<JointPolicyPureVectorForClusteredBG>(*boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(bgpol));
        jpolBGcopy->SetDepth(ts+1);

        double discountToThePowerT = pow( GetDiscount(), (double)(ts) );
        double discounted_F = discountToThePowerT * val;

        //compute expected immediate reward for this stage
        double newPastreward = pastReward_prevTs + immR;
        jpolBGcopy->SetPastReward(newPastreward);
        
        //push this policy and value on the priority queue
        ////if last stage, if so, we want to return the 
        ////*EXACT* past reward, newPastreward.
        double newValue;
        if(is_last_ts)
            newValue=newPastreward;
        else
            newValue=pastReward_prevTs + discounted_F;

        poolOfNextPolicies->Insert( NewPPI(jpolBGcopy,newValue) );

        if(ts>0)
        {
            double oldValue=ppi->GetValue();
            // now we update the current PPI with the value of the next sibling
            // other option is the value of this child
//        ppi->SetValue(bgips->GetPayoff());
            ppi->SetValue(newValue);//-(1e-10));

            if(GetDiscount()==1.0 &&
               oldValue < ppi->GetValue() &&
               !EqualReward(oldValue, ppi->GetValue()))
            {
                stringstream ss;
                ss << "GMAA_MAAstarCluster value of parent can only go down when being updated: old value="
                   << oldValue << " new value=" << ppi->GetValue()
                   << " old-new=" << oldValue-ppi->GetValue();
                throw(E(ss));
            }
        }
    }
    else // we did NOT get a valid solution from this CBG
    {
        if(_m_verboseness >= 1)
            cout << "GMAA_MAAstar no valid solution returned by the Incremental BGIP Solver"<< endl;

        // as we didn't any solution for the CBG, we know this branch
        // of the tree can never contain the optimal policy, so we set
        // its value very low
        ppi->SetValue(-DBL_MAX);
    }

    // if we returned all solutions from this BG, it has to be deleted
    // from the policy pool
    if(bgips->AllSolutionsHaveBeenReturned())
    {
        if(_m_verboseness >= 4)
            cout << ">>>AllSolutionsHaveBeenReturned, removing " << pP->SoftPrint()
                 << "<<<"<<endl;
        cleanUpPPI=true;
    }
    else
        cleanUpPPI=false;

    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);
}

void GMAA_MAAstarCluster::SelectPoliciesToProcessFurther(
    const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
    bool are_LBs, double bestLB)
{
    //for MAA*, all policies are processed further when they aren't full
    //policies (i.e. lower bounds)
    //so, unless these are full policies...
    if(are_LBs)
    {
        while(!poolOfNextPolicies->Empty())
            poolOfNextPolicies->Pop();
    }
    //we can return immediately...
    return;
}

PartialPolicyPoolItemInterface_sharedPtr
GMAA_MAAstarCluster::NewPPI(const PJPDP_sharedPtr &jp, double v) const
{
    //the index variant can not store a JointPolicyPureVectorForClusteredBG!
    //PartialPolicyPoolItemInterface* ppi=new JPPVIndexValPair(jp,v);
    //delete jp;//We can't delete the joint policy!
    PartialPolicyPoolItemInterface_sharedPtr ppi=boost::make_shared<PartialJPDPValuePair>(jp,v);
    return (ppi);
}
        

PJPDP_sharedPtr
GMAA_MAAstarCluster::NewJPol() const
{ 
    return boost::make_shared<JointPolicyPureVectorForClusteredBG>(_m_dummyBG); 
}


std::string GMAA_MAAstarCluster::SoftPrintClusteringStats() const
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

void GMAA_MAAstarCluster::SaveClusterStats(const string &filename) const
{
    ofstream fp(filename.c_str());
    if(!fp)
        cerr << "GMAA_MAAstarCluster::SaveClusterStats failed to open file "
             << filename << endl;

    fp << SoftPrintClusteringStats() << endl;
}

