/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "QMDP.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointActionObservationHistoryTree.h"
#include "JointBelief.h"
#include "JointBeliefSparse.h"

using namespace std;

//Default constructor
QMDP::QMDP(const PlanningUnitDecPOMDPDiscrete* pu,
           bool useJaohQValuesCache) :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOH(pu),
    _m_p(0),
    _m_initialized(false),
    _m_useJaohQValuesCache(useJaohQValuesCache)
{
}

QMDP::QMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
           bool useJaohQValuesCache) :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOH(pu),
    _m_p(0),
    _m_initialized(false),
    _m_useJaohQValuesCache(useJaohQValuesCache)
{
}

//Destructor
QMDP::~QMDP()
{    
    DeInitialize();
}

void QMDP::Initialize()
{
    if(_m_useJaohQValuesCache)
        _m_QValues.resize(GetPU()->GetNrJointActionObservationHistories(),
                          GetPU()->GetNrJointActions(),
                          false);

    _m_p=new MDPValueIteration(*GetPU());

    _m_initialized=true;
}

void QMDP::DeInitialize()
{
    delete _m_p;
    _m_QValues.clear();
    _m_initialized=false;
}

void QMDP::SetPU(const PlanningUnitDecPOMDPDiscrete* pu)
{
    DeInitialize();
    QFunctionJAOH::SetPU(pu);
}

void QMDP::SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
{
    DeInitialize();
    QFunctionJAOH::SetPU(pu);
}

void QMDP::Compute()
{
    if(!_m_initialized)
        Initialize();

    _m_p->Plan();

    if(_m_useJaohQValuesCache)
        CacheJaohQValues();
}

void QMDP::ComputeWithCachedQValues(const string &filename,
                                    bool computeIfNotCached)
{
    if(!_m_initialized)
        Initialize();

    _m_p->PlanWithCache(filename,computeIfNotCached);

    if(_m_useJaohQValuesCache)
        CacheJaohQValues();
}

void QMDP::CacheJaohQValues()
{
    for(Index jaohI=0;
        jaohI!=GetPU()->GetNrJointActionObservationHistories();
        ++jaohI)
    {
        Index t = GetPU()->GetTimeStepForJAOHI(jaohI);
        for(Index jaI=0;jaI!=GetPU()->GetNrJointActions();++jaI)
            _m_QValues(jaohI,jaI)=
                _m_p->GetQ (t, *GetPU()->GetJointBeliefInterface(jaohI), jaI);
    }
}

double QMDP::GetQNoCache(Index jaohI, Index jaI) const
{
    Index t = GetPU()->GetTimeStepForJAOHI(jaohI);
    JointBeliefInterface * jb = GetPU()->GetJointBeliefInterface(jaohI);
    double q = _m_p->GetQ (t, *jb, jaI);
    delete jb;
    return( q );
}

void QMDP::Save(const string &filename) const
{
    if(_m_useJaohQValuesCache)
        QTable::Save(_m_QValues,filename);
    else
        QTable::Save(_m_p->GetQTables(),filename);
}

void QMDP::Load(const string &filename)
{
    if(!_m_initialized)
        Initialize();

    if(_m_useJaohQValuesCache)
        QTable::Load(filename,
                     GetPU()->
                     GetNrJointActionObservationHistories(),
                     GetPU()->GetNrJointActions(),
                     _m_QValues);
    else
    {
        QTables qs;
        QTable::Load(filename,
                     GetPU()->GetNrStates(),
                     GetPU()->GetNrJointActions(),
                     GetPU()->GetHorizon(),
                     qs);
        _m_p->SetQTables(qs);
    }
}
