/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <vector>
#include <float.h>
#include "boost/pointer_cast.hpp"
#include "GMAA_MAAstar.h"
#include "JPPVValuePair.h"
#include "BGIP_SolverCreator_BFS.h"

#include "BayesianGameForDecPOMDPStage.h"
#include "BGIP_IncrementalSolverInterface.h"

using namespace std;

GMAA_MAAstar::GMAA_MAAstar(
    const PlanningUnitMADPDiscreteParameters &params,
    const BGIP_SolverCreatorInterface * bgsc,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p,
    int verboseness
    ) :
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(params, horizon, p, verboseness),
    _m_newBGIP_Solver(bgsc)
{
    if(!_m_newBGIP_Solver->IsExactSolver())
        throw(E("GMAA_MAAstar requires an exact BG solver"));
}

/*
GMAA_MAAstar::GMAA_MAAstar(
    const BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * bgsc,
    size_t horizon, 
    DecPOMDPDiscreteInterface* p) :
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete(horizon, p),
    _m_newBGIP_Solver(bgsc)
{
    // check if we were passed a BGsolver
    if(_m_newBGIP_Solver)
    {
        if(!_m_newBGIP_Solver->IsExactSolver())
            throw(E("GMAA_MAAstar requires an exact BG solver"));
    }
    else // otherwise instantiate one
    {
        _m_newBGIP_Solver=new BGIP_SolverCreator_BFS<JointPolicyPureVector>();
    }
}
*/

void GMAA_MAAstar::ResetPlanner()
{
}

bool GMAA_MAAstar::ConstructAndValuateNextPolicies(
        const PartialPolicyPoolItemInterface_sharedPtr &ppi,
        const PartialPolicyPoolInterface_sharedPtr &poolOfNextPolicies,
        bool &cleanUpPPI) 
{
    PJPDP_sharedPtr jpolPrevTs = ppi->GetJPol();//jpol^ts-1
    size_t ts = jpolPrevTs->GetDepth();     // = depth = ts(jpolPrevTs) + 1
    bool is_last_ts = (ts ==  GetHorizon() - 1);

    //the BG pointer
    boost::shared_ptr<const BayesianGameForDecPOMDPStage> bg_ts;
    //the solver pointers
    //boost::shared_ptr<BGIP_IncrementalSolverInterface_T<JointPolicyPureVector> > bgipIncSolver;
    //boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> > bgips;
    boost::shared_ptr<BGIP_IncrementalSolverInterface> bgipIncSolver;
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver> bgips;
    // Check whether we already considered this
    // PartialJointPolicyDiscretePure before, in which case the
    // BayesianGameForDecPOMDPStage and the BGIPSolver have already
    // been generated
    if(ppi->GetBGIPSolverPointer()==0)
    {
        if(_m_verboseness >= 8) 
            cout << "GMAA_MAAstar: I have not expanded this ppi before..." << endl;

        // Construct the bayesian game for this timestep - 
        bg_ts= boost::shared_ptr<BayesianGameForDecPOMDPStage> (
            new BayesianGameForDecPOMDPStage(
                this,
                _m_qHeuristic,
                jpolPrevTs
                ));
        // not sure this pays off in terms of time vs space...
//        bg_ts->ComputeAllImmediateRewards();

        // save the BG to disk in case the user requested it
        _m_bgCounter++;
        if(_m_bgBaseFilename!="")
        {
            stringstream ss;
            ss << _m_bgBaseFilename << _m_bgCounter;
            BayesianGameIdenticalPayoff::Save(*bg_ts,ss.str());
        }

        // keep track of every bg_ts we instantiate, so we can clean
        // them up after solving
//        _m_pointersToAllBGTS.push_back(bg_ts);

        //create the Bayesian game solver
        bgips =
            boost::shared_ptr<
            BayesianGameIdenticalPayoffSolver/*_T<JointPolicyPureVector>*/
            >( (*_m_newBGIP_Solver)(bg_ts) );
        bgipIncSolver = boost::dynamic_pointer_cast<
            BGIP_IncrementalSolverInterface/*_T<JointPolicyPureVector>*/ > (bgips);
        if(bgipIncSolver==0)
            throw(E("GMAA_MAAstar requires BGIP_IncrementalSolverInterface solvers"));

        if(is_last_ts)
        {
            //For the last stage we only want 1 solution.
            bgipIncSolver->SetNrDesiredSolutions(1);
            if(_m_verboseness >= 4)
                std::cout << "Last stage - set nr solutions to 1"<<endl;
        }
        else if(bgipIncSolver->GetNrDesiredSolutions()!=INT_MAX)
            throw(E("GMAA_MAAstar requires a BGIP solver to return all solutions (when requested)"));

        // associate the resulting incremental BGIPSolver with this
        // PPI, so that we can reuse it
        ppi->SetBGIPSolverPointer(bgipIncSolver);
    }
    else
    {
        if(_m_verboseness >= 8) 
            cout << "GMAA_MAAstar: ppi was expanded before, getting the previous BGIPSolver..." << endl;
        bgips = ppi->GetBGIPSolverPointer();           
        bgipIncSolver= boost::dynamic_pointer_cast<
            BGIP_IncrementalSolverInterface/*_T<JointPolicyPureVector> */
            > (bgips);
        if(bgipIncSolver==0)
            throw(E("GMAA_MAAstar requires BGIP_IncrementalSolverInterface solvers"));
        bg_ts = boost::dynamic_pointer_cast<const BayesianGameForDecPOMDPStage>(bgipIncSolver->GetBGIPI());
        if(bg_ts==0)
            throw(E("GMAA_MAAstar BG is not of the correct type"));
    }

    stringstream ss;
    ss << "GMAA_MAAstar::NextExact_ts" << ts;
    StartTimer(ss.str());

    // Compute bounds on the value solving this CBG can result in
    this->GeneralizedMAAStarPlannerForDecPOMDPDiscrete::SetCBGbounds(ppi,bgips);

    // Ask the BGIPSolver for the next solution, which might not exist
    // due to the bounds on the value of the solution
    double val;
    boost::shared_ptr<JointPolicyDiscretePure> jpdp;
    /// Calling the BGIPSolver
    bool foundCBGsolution = bgipIncSolver->GetNextJointPolicyAndValue(jpdp, val);

    if(foundCBGsolution)
    {
        JPPV_sharedPtr bgpol = boost::dynamic_pointer_cast<JointPolicyPureVector>(jpdp);
        if(bgpol==0)
            throw(E("GMAA_MAAstar we did not get a valid policy from the incremental BGIP solver"));

        //Frans 20110908: what kind of construction is this: 
        PJPDP_sharedPtr jpolTs;
        {
            vector<Index> firstOHtsI(GetNrAgents());
            for(Index agI=0; agI < GetNrAgents(); agI++)
                firstOHtsI.at(agI) = 
                    CastLIndexToIndex(GetFirstObservationHistoryIndex(agI, ts));

            const vector<size_t>& nrOHts = bg_ts->GetNrTypes(); 
            jpolTs = ConstructExtendedJointPolicy(*jpolPrevTs,
                                                  *bgpol,
                                                  nrOHts,
                                                  firstOHtsI);
        }
        //??? why the indented block?!
        
        double discountToThePowerT = pow( GetDiscount(), (double)(ts) );
        double discounted_F = discountToThePowerT * val;
        
        //compute expected immediate reward for this stage
        double immR = bg_ts->ComputeDiscountedImmediateRewardForJPol(bgpol);
        double prevPastReward = jpolPrevTs->GetPastReward();
        double newPastreward = jpolPrevTs->GetPastReward() + immR;
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

#if DEBUG_GMAA4
        cout <<"v = pastReward_prevTs + g^t * f = "
             << newValue <<" = "
             << jpolPrevTs->GetPastReward() <<" + "
             << discounted_F << "   "
             << "(g^t * f ="<<discountToThePowerT << " * " << val << ")"
             << endl;
#endif

        if(ts>0)
        {
            double oldValue=ppi->GetValue();
            // now we update the current PPI with the value of the next sibling
            // other option is the value of this child
//          ppi->SetValue(bgipIncSolver->GetPayoff());
            ppi->SetValue(newValue);

            if(GetDiscount()==1.0 &&
               oldValue < ppi->GetValue() &&
               !EqualReward(oldValue, ppi->GetValue()))
            {
                stringstream ss;
                ss << "GMAA_MAAstar value of parent can only go down when being updated: old value="
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
    if(bgipIncSolver->AllSolutionsHaveBeenReturned())
    {
        if(_m_verboseness >= 4)
            cout << "AllSolutionsHaveBeenReturned, removing " << jpolPrevTs->SoftPrint()
                 << endl;
        cleanUpPPI=true;
    }
    else
        cleanUpPPI=false;

    StopTimer(ss.str());
    //if we created a BG for the last time step t=h-1 - we have a lowerbound
    return(is_last_ts);
}

void GMAA_MAAstar::SelectPoliciesToProcessFurther(
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
GMAA_MAAstar::NewPPI(const PJPDP_sharedPtr &jp,
                     double v) const
{
    //NOTE: this conversion to an index does save a lot of space, so
    //if we can enable this again would be better.
    //PartialPolicyPoolItemInterface* ppi=new JPPVIndexValPair(jp,v);
    //delete jp;
    PartialPolicyPoolItemInterface_sharedPtr ppi=
        PartialPolicyPoolItemInterface_sharedPtr(new PartialJPDPValuePair(jp,v));
    return (ppi);
}
