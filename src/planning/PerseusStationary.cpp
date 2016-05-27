/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "PerseusStationary.h"
#include "argumentHandlers.h"

using namespace std;

//Default constructor
PerseusStationary::PerseusStationary(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    Perseus(pu)
{
}

PerseusStationary::PerseusStationary(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    Perseus(pu)
{
}

PerseusStationary::PerseusStationary(const PlanningUnitFactoredDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    Perseus(pu)
{
}

PerseusStationary::PerseusStationary(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    Perseus(pu)
{
}


//Destructor
PerseusStationary::~PerseusStationary()
{
    if(_m_beliefsInitialized)
        delete(_m_beliefs);
}

double PerseusStationary::GetQ(const JointBeliefInterface &b, Index jaI) const
{
    return(BeliefValue::GetValue(b,_m_qFunction[jaI]));
}

double PerseusStationary::GetQ(const JointBeliefInterface &b, Index jaI,
                               AlphaVector::BGPolicyIndex &betaMaxI) const
{
    Index maxI=BeliefValue::
        GetMaximizingVectorIndex(b,_m_qFunction[jaI]);
    betaMaxI=_m_qFunction[jaI][maxI].GetBetaI();
    return(b.InnerProduct(_m_qFunction[jaI][maxI].GetValues()));
}

double PerseusStationary::GetQ(const JointBeliefInterface &b, Index t,
                               Index jaI) const
{
    throw(E("PerseusStationary::GetQ not implemented for non-stationary case"));
    return(42);
}

double PerseusStationary::GetQ(const JointBeliefInterface &b, Index t,
                               Index jaI, AlphaVector::BGPolicyIndex &betaMaxI) const
{
    throw(E("PerseusStationary::GetQ not implemented for non-stationary case"));
    return(42);
}

/// If no belief is specified, one will be sampled using default
/// settings.
void PerseusStationary::SetBeliefSet(const BeliefSet &S)
{
    if(_m_beliefsInitialized)
        delete(_m_beliefs);
    _m_beliefs=new BeliefSet(S);
    _m_beliefsInitialized=true;
}

void PerseusStationary::ExportBeliefSet(const string &filename) const
{
    if(!_m_beliefsInitialized)
    {
        cout << "PerseusStationary::ExportBeliefSet: belief set not set"
             << endl;
    }
    else
        AlphaVectorPlanning::ExportBeliefSet(*_m_beliefs,filename);
}

/// Calls AlphaVectorPlanning::ExportValueFunction.
void PerseusStationary::ExportValueFunction(const string &filename) const
{
    AlphaVectorPlanning::ExportValueFunction(filename,_m_valueFunction);
}

void PerseusStationary::Save(const string &filename) const
{
    ExportValueFunction(filename);
}

void PerseusStationary::Load(const std::string &filename)
{
    _m_valueFunction=ImportValueFunction(filename);
}

std::vector<double> PerseusStationary::GetImmediateRewardBeliefSet() const
{
    return(BeliefValue::GetValues(*_m_beliefs,
                                  AlphaVectorPlanning::
                                  GetImmediateRewardValueFunction(GetPU())));
}

/** Also updates _m_qFunction. */
void PerseusStationary::SetValueFunction(const string &filename)
{
    _m_valueFunction=ImportValueFunction(filename);
    _m_qFunction=AlphaVectorPlanning::
        ValueFunctionToQ(_m_valueFunction,
                         GetPU()->GetNrJointActions(),
                         GetPU()->GetNrStates());
}

void PerseusStationary::InitializeBeliefs(int nrB, bool uniquify)
{
    ArgumentHandlers::Arguments args;
    args.nrBeliefs=nrB;
    args.uniqueBeliefs=uniquify;
    SetBeliefSet(SampleBeliefs(args));
}

void PerseusStationary::StoreValueFunction(const ValueFunctionPOMDPDiscrete &V)
{
    // store the resulting value function
    _m_valueFunction=V;
    // compute the Q functions from the value function
    _m_qFunction=ValueFunctionToQ(_m_valueFunction);
}

void PerseusStationary::StoreValueFunction(const QFunctionsDiscrete &Q)
{
    // store the resulting Q function and the value function
    _m_qFunction=Q;
    _m_valueFunction.clear();
    for(QFDcit i=Q.begin();i!=Q.end();++i)
        for(VFPDcit j=i->begin();j!=i->end();++j)
            if(j->GetAction()!=INT_MAX)
                _m_valueFunction.push_back(*j);
}
