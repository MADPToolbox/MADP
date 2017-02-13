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

#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "FactoredStateAOHDistribution.h"
#include "PartialJointPolicyDiscretePure.h"
#include "FSAOHDist_NECOF.h"
using namespace std;

//Default constructor
PlanningUnitFactoredDecPOMDPDiscrete::PlanningUnitFactoredDecPOMDPDiscrete(
    size_t horizon,
    FactoredDecPOMDPDiscreteInterface* p,
    const PlanningUnitMADPDiscreteParameters *params
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon,p, params) 
    ,_m_fDecPOMDP(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitFactoredDecPOMDPDiscrete(PlanningUnitMADPDiscreteParameters params, size_t horizon, FactoredDecPOMDPDiscreteInterface* p  called" << endl;
}

/*
PlanningUnitFactoredDecPOMDPDiscrete::PlanningUnitFactoredDecPOMDPDiscrete(
    size_t horizon,
    FactoredDecPOMDPDiscreteInterface* p
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon,p)
    ,_m_fDecPOMDP(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitFactoredDecPOMDPDiscrete(size_t horizon, FactoredDecPOMDPDiscreteInterface* p  called" << endl;
}
 * */

/*
//Copy assignment constructor.    
PlanningUnitFactoredDecPOMDPDiscrete::PlanningUnitFactoredDecPOMDPDiscrete(const PlanningUnitFactoredDecPOMDPDiscrete& o) 
{
}
//Destructor
PlanningUnitFactoredDecPOMDPDiscrete::~PlanningUnitFactoredDecPOMDPDiscrete()
{
}
*/
void PlanningUnitFactoredDecPOMDPDiscrete::SetProblem(FactoredDecPOMDPDiscreteInterface* p)
{
    if(p == GetFDPOMDPD())
        return;

    _m_fDecPOMDP = p;
    //set (and initialize) the problem at PlanningUnitDecPOMDPDiscrete level:
    DecPOMDPDiscreteInterface* p2 = static_cast<DecPOMDPDiscreteInterface*>(p);
    PlanningUnitDecPOMDPDiscrete::SetProblem(p2);
}


FactoredStateAOHDistribution* PlanningUnitFactoredDecPOMDPDiscrete::
GetNewFactoredStateAOHDistribution() const
{
    return new FSAOHDist_NECOF(this);
}

void PlanningUnitFactoredDecPOMDPDiscrete::
ComputeFSAOHDist(
        FactoredStateAOHDistribution* fsaoh, 
        const PartialJointPolicyDiscretePure& pJPol) const
{
    Index depth = pJPol.GetDepth();
    //cout << "computing factored FSAOHDist for policy of depth"<< depth<<endl;
    fsaoh->InitializeFromISD( GetFDPOMDPD()->GetFactoredISD() );
    fsaoh->SanityCheck();
    Index lastStage;
    if(depth > 0)
        //depth 1 specifies action for t=0 -> compute FSAOH for t=1
        lastStage = depth; 
    else
        return;

    //cerr<<"starting updates from t=0 to t<"<<lastStage<<endl;
    for(Index t=0 ; t < lastStage; t++)
        fsaoh->Update(pJPol);
}
