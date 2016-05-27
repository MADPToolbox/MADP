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
#include <limits.h>
#include "GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete.h"

#include "FactoredQFunctionStateJAOHInterface.h"

#include "PartialJointPolicyPureVector.h"
#include "PartialJPDPValuePair.h"
#include "PolicyPoolPartialJPolValPair.h"

//needed for some auxil. functions that should be removed:
#include "BayesianGameIdenticalPayoff.h"
#include "JointObservationHistoryTree.h"
#include "JointBeliefInterface.h"
#include "JointActionObservationHistoryTree.h"


using namespace std;

//Default constructor
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete(
        PlanningUnitMADPDiscreteParameters params,
        size_t horizon, 
        FactoredDecPOMDPDiscreteInterface* p
        , int verboseness
        , double slack) 
:
    PlanningUnitFactoredDecPOMDPDiscrete(params, horizon, p)
    , GeneralizedMAAStarPlanner(verboseness, slack)
{
    _m_qHeuristic=0;
    _m_useSparseBeliefs=GetParams().GetUseSparseJointBeliefs();
}

GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete(
        size_t horizon, 
        FactoredDecPOMDPDiscreteInterface* p
        , double slack) 
:
    PlanningUnitFactoredDecPOMDPDiscrete(horizon, p),
    GeneralizedMAAStarPlanner(0, slack)
{
    cout << "GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete slack="<<slack<<endl;
    _m_qHeuristic=0;
    _m_useSparseBeliefs=GetParams().GetUseSparseJointBeliefs();
}

/*
//Destructor
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
~GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete()
{
}
*/
//Copy assignment operator
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete& 
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::operator= (
        const GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    //
    throw("GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::operator= not implemented");

    return *this;
}


//this function extends a previous policy jpolPrevTs for ts-1 with the behavior specified by the policy of the BayesianGame for time step ts (jpolBG).
PJPDP_sharedPtr 
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
ConstructExtendedJointPolicy(
        const PartialJointPolicyDiscretePure& jpolPrevTs, 
        const JointPolicyDiscretePure& jpolBG, 
        const vector<size_t>& nrOHts, 
        const vector<Index>& firstOHtsI)
{
    //check policy types
    if(jpolPrevTs.GetPolicyDomainCategory() != PolicyGlobals::OHIST_INDEX)
        throw E("GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::ConstructExtendedJointPolicy --- jpolPrevTs.GetPolicyDomainCategory() != PolicyGlobals::OHIST_INDEX ");
    if(jpolBG.GetPolicyDomainCategory() != PolicyGlobals::TYPE_INDEX)
        throw E("GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::ConstructExtendedJointPolicy --- jpolPrevTs.GetPolicyDomainCategory() != PolicyGlobals::TYPE_INDEX ");
    //construct a policy for the DecPOMDP: 
    //a copy of jpolPrevTs with extended to this time step (ts) by 
    //jpolBG
    boost::shared_ptr<PartialJointPolicyPureVector> jpolTs = 
        boost::shared_ptr<PartialJointPolicyPureVector>(
            new PartialJointPolicyPureVector( 
                dynamic_cast<const PartialJointPolicyPureVector&>( jpolPrevTs )
                ));
    jpolTs->SetDepth( jpolTs->GetDepth()+1 );
    for(Index agentI=0; agentI < GetNrAgents(); agentI++)
    {
        for(Index type = 0; type < nrOHts[agentI]; type++)
        {
            Index ohI = type + firstOHtsI[agentI];
            jpolTs->SetAction(agentI, ohI, 
                    jpolBG.GetActionIndex(agentI, type) );
        }         
    }
    return(jpolTs);
}


PJPDP_sharedPtr
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
NewJPol() const
{
    return PJPDP_sharedPtr(
        new PartialJointPolicyPureVector(this, OHIST_INDEX, 0.0, 0));
}

PartialPolicyPoolItemInterface_sharedPtr
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
NewPPI(const PJPDP_sharedPtr &jp, double v) const
{
    ///we know that jp is a PartialJointPolicyPureVector (see NewJPol above)
    //this can be used to have a more compact representation by doing:
    //  PartialJointPolicyPureVector* p = 
    //      static_cast<PartialJointPolicyPureVector*>(jp);
    //  ppi = new PartialJPPVIndexValPair(jp,v);

    PartialPolicyPoolItemInterface_sharedPtr ppi = 
        PartialPolicyPoolItemInterface_sharedPtr( new PartialJPDPValuePair(jp,v) );
    //delete jp; // <- only if policy stored as index
    return (ppi);
}

PartialPolicyPoolInterface_sharedPtr
GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete::
NewPP() const
{
    return PartialPolicyPoolInterface_sharedPtr(new PolicyPoolPartialJPolValPair);
}
