/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "FactoredStateAOHDistribution.h"
#include "PartialJointPolicyDiscretePure.h"
#include "FSAOHDist_NECOF.h"
using namespace std;

//Default constructor
PlanningUnitFactoredDecPOMDPDiscrete::PlanningUnitFactoredDecPOMDPDiscrete(
    const PlanningUnitMADPDiscreteParameters &params,
    size_t horizon,
    FactoredDecPOMDPDiscreteInterface* p
    ) :
    PlanningUnitDecPOMDPDiscrete(params,horizon,p) 
    ,_m_fDecPOMDP(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitFactoredDecPOMDPDiscrete(PlanningUnitMADPDiscreteParameters params, size_t horizon, FactoredDecPOMDPDiscreteInterface* p  called" << endl;
}

PlanningUnitFactoredDecPOMDPDiscrete::PlanningUnitFactoredDecPOMDPDiscrete(
    size_t horizon,
    FactoredDecPOMDPDiscreteInterface* p
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon,p)
    ,_m_fDecPOMDP(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitFactoredDecPOMDPDiscrete(size_t horizon, FactoredDecPOMDPDiscreteInterface* p  called" << endl;
}

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
