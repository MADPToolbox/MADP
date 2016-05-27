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

#include "PerseusNonStationary.h"
#include "BeliefSetNonStationary.h"
#include "argumentHandlers.h"

using namespace std;

#define DEBUG_PerseusNonStationary 0

//Default constructor
PerseusNonStationary::
PerseusNonStationary(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    Perseus(pu)
{
    SetInitializeWithZero(true);
}

PerseusNonStationary::
PerseusNonStationary(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    Perseus(pu)
{
    SetInitializeWithZero(true);
}

//Destructor
PerseusNonStationary::~PerseusNonStationary()
{
    if(_m_beliefsInitialized)
        delete(_m_beliefs);
}

vector<double> PerseusNonStationary::GetImmediateRewardBeliefSet() const
{
    return(BeliefValue::GetValues(*_m_beliefs,
                                  AlphaVectorPlanning::
                                  GetImmediateRewardValueFunction(GetPU())));
}

void PerseusNonStationary::InitializeBeliefs(int nrB, bool uniquify)
{
    ArgumentHandlers::Arguments args;
    args.nrBeliefs=nrB;
    args.uniqueBeliefs=uniquify;
    SetBeliefSet(SampleBeliefsNonStationary(args));
}

void PerseusNonStationary::SetValueFunction(const string &filename)
{
    _m_qFunctions=
        AlphaVectorPlanning::ImportValueFunction(filename,GetPU()->GetHorizon(),
                                                 GetPU()->GetNrJointActions(),
                                                 GetPU()->GetNrStates());
}

void PerseusNonStationary::
StoreValueFunction(const QFunctionsDiscreteNonStationary &Q)
{
    _m_qFunctions=Q;
}

void PerseusNonStationary::SetBeliefSet(const BeliefSetNonStationary &S)
{
    if(_m_beliefsInitialized)
        delete(_m_beliefs);
    _m_beliefs=new BeliefSetNonStationary(S);
    _m_beliefsInitialized=true;
}

void PerseusNonStationary::ExportValueFunction(const string &filename) const
{
    AlphaVectorPlanning::ExportValueFunction(filename,_m_qFunctions);
}

void PerseusNonStationary::Save(const string &filename) const
{
    ExportValueFunction(filename);
}

void PerseusNonStationary::Load(const std::string &filename)
{
    _m_qFunctions=ImportValueFunction(filename,
                                      GetPU()->GetHorizon(),
                                      GetPU()->GetNrJointActions(),
                                      GetPU()->GetNrStates());
}

double
PerseusNonStationary::GetQ(const JointBeliefInterface &b, Index jaI,
                           AlphaVector::BGPolicyIndex &betaMaxI) const
{
    throw(E("PerseusNonStationary::GetQ not implemented for non-stationary case"));
    return(42);
}

double PerseusNonStationary::GetQ(const JointBeliefInterface &b,
                                  Index jaI) const
{
    throw(E("PerseusNonStationary::GetQ not implemented for non-stationary case"));
    return(42);
}

double PerseusNonStationary::GetQ(const JointBeliefInterface &b, Index t,
                                  Index jaI) const
{
    return(BeliefValue::GetValue(b,_m_qFunctions[t][jaI]));
}

double PerseusNonStationary::GetQ(const JointBeliefInterface &b, Index t,
                                  Index jaI, AlphaVector::BGPolicyIndex &betaMaxI) const
{
    Index maxI=BeliefValue::
        GetMaximizingVectorIndex(b,_m_qFunctions[t][jaI]);
    betaMaxI=_m_qFunctions[t][jaI][maxI].GetBetaI();
#if DEBUG_PerseusNonStationary
    cout << "jaI " << jaI << " maxI " << maxI << " betaMaxI " << betaMaxI 
         << SoftPrint(_m_qFunctions[t][jaI]) << endl;
#endif
    return(b.InnerProduct(_m_qFunctions[t][jaI][maxI].GetValues()));
}
