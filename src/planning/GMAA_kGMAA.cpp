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

#include "GMAA_kGMAA.h"
#include "JPPVValuePair.h"
#include "PartialJPDPValuePair.h"
#include "PartialPolicyPoolInterface.h"
#include "PolicyPoolPartialJPolValPair.h"
#include "BayesianGameForDecPOMDPStage.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "BGIP_SolverCreatorInterface_T.h"
#include "PartialJointPolicyPureVector.h"

using namespace std;

GMAA_kGMAA::GMAA_kGMAA(
    const PlanningUnitMADPDiscreteParameters &params,
    //const BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * bgsc,
    const BGIP_SolverCreatorInterface * bgsc,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p,
    size_t nrPoliciesToProcess) :
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(params, horizon, p),
    _m_newBGIP_Solver(bgsc)
{
    _m_nrPoliciesToProcess=nrPoliciesToProcess;
}

void GMAA_kGMAA::ResetPlanner()
{
    // nothing to do here
}

//This function will construct and sove a BG for the next timestep ts
bool GMAA_kGMAA::ConstructAndValuateNextPolicies(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi, 
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
        bool &cleanUpPPI)
{
    cleanUpPPI=true;
    PJPDP_sharedPtr jpolPrevTs = ppi->GetJPol();//jpol^ts-1
    size_t ts = jpolPrevTs->GetDepth();     // = depth = ts(jpolPrevTs) + 1
    bool is_last_ts = (ts ==  GetHorizon() - 1);

    vector<Index> firstOHtsI(GetNrAgents());
    for(Index agI=0; agI < GetNrAgents(); agI++)
        firstOHtsI.at(agI) = CastLIndexToIndex(GetFirstObservationHistoryIndex(agI, ts));
    // Construct the bayesian game for this timestep - 
    boost::shared_ptr<BayesianGameForDecPOMDPStage> bg_ts=
        boost::shared_ptr<BayesianGameForDecPOMDPStage>(
        new BayesianGameForDecPOMDPStage(
            this,
            _m_qHeuristic,
            jpolPrevTs
            ));

    _m_bgCounter++;
    if(_m_bgBaseFilename!="")
    {
        stringstream ss;
        ss << _m_bgBaseFilename << _m_bgCounter;
        BayesianGameIdenticalPayoff::Save(*bg_ts,ss.str());
    }

    double prevPastReward = jpolPrevTs->GetPastReward();
    const vector<size_t>& nrOHts = bg_ts->GetNrTypes(); 
    //size_t nrJOHts = bg_ts->GetNrJointTypes(); 

    //The set of Indicies of the policies added to poolOfNextPolicies
    //(to avoid adding duplicates).
    set<Index> poolOfNextPoliciesIndices;

    //the policy for the Bayesian game 
    JointPolicyPureVector jpolBG = JointPolicyPureVector(bg_ts,
            PolicyGlobals::TYPE_INDEX);

    stringstream ss;
    ss << "GMAA::kGMAA_ts" << ts;
    StartTimer(ss.str());
    
    //solve the Bayesian game
    //boost::shared_ptr<
    //BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> > bgips=
        //boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> >((*_m_newBGIP_Solver)(bg_ts));
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver> bgips =
        boost::shared_ptr<BayesianGameIdenticalPayoffSolver>(
                _m_newBGIP_Solver->operator()(bg_ts)
                );

    SetCBGbounds(ppi,bgips);
    bgips->Solve();

    //for each solution in BGIPSolution  
    bg_ts->ComputeAllImmediateRewards();
    for(Index solI=0; solI < _m_nrPoliciesToProcess; solI++)
    {
        if(bgips->IsEmptyJPPV())
        {
            cerr << "Warning, BGIP_Solver only returned "<<solI<<
                " usable joint policies"<<endl;
            break;
        }
        const boost::shared_ptr<JPPVValuePair> jpvp = bgips->GetNextSolutionJPPV();
        JPPV_sharedPtr bgpol = jpvp->GetJPPV();

        bgips->PopNextSolutionJPPV();
        PJPDP_sharedPtr jpolTs = 
            ConstructExtendedJointPolicy(*jpolPrevTs,
                                    *bgpol, nrOHts, firstOHtsI);

        double val = jpvp->GetValue();
        double discountToThePowerT = pow( GetDiscount(), (double)(ts) );
        double discounted_F = discountToThePowerT * val;

        //compute expected immediate reward for this stage
        double immR = bg_ts->ComputeDiscountedImmediateRewardForJPol(bgpol);
        double newPastreward = prevPastReward + immR;
        jpolTs->SetPastReward(newPastreward);
        
        ////if last stage, if so, we want to return the 
        ////*EXACT* past reward, newPastreward.
        double newValue;
        if(is_last_ts)
            newValue = newPastreward;
        else
            newValue = prevPastReward + discounted_F;

        //push this policy and value on the priority queue
        poolOfNextPolicies->Insert( NewPPI(jpolTs,newValue) );

        /*//push this policy and value on the priority queue
        ////if last stage, if so, we want to return the 
        ///[>EXACT* past reward, newPastreward.
        if(is_last_ts)
            poolOfNextPolicies->Insert( NewPPI(jpolTs,newPastreward) );
        else // VAL SHOULD BE DISCOUNTED?
            poolOfNextPolicies->Insert( NewPPI(jpolTs,
                                        val + prevPastReward) );*/

    }
    //empty the imm reward cache
    bg_ts->ClearAllImmediateRewards();

    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);
}

void GMAA_kGMAA::SelectPoliciesToProcessFurther(
    const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies, bool are_LBs, double bestLB)
{
    SelectKBestPoliciesToProcessFurther(poolOfNextPolicies, are_LBs, 
                                        bestLB, _m_nrPoliciesToProcess);
}                  

PartialPolicyPoolInterface_sharedPtr
GMAA_kGMAA::NewPP()
{return PartialPolicyPoolInterface_sharedPtr(new PolicyPoolPartialJPolValPair);}

PartialPolicyPoolItemInterface_sharedPtr 
GMAA_kGMAA::NewPPI(const PJPDP_sharedPtr &jp, double v) const
{
    //return (new JPPVValuePair(jp,v));
    PartialPolicyPoolItemInterface_sharedPtr ppi=
        PartialPolicyPoolItemInterface_sharedPtr(new PartialJPDPValuePair(jp,v));
    return (ppi);
}



