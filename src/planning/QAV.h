/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

/* Only include this header file once. */
#ifndef _QAV_H_
#define _QAV_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJointBelief.h"
#include "BeliefSet.h"
#include "ValueFunctionPOMDPDiscrete.h"
#include "AlphaVectorPlanning.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "QAVParameters.h"
#include "QFunctionJAOHInterface.h"

/** \brief QAV implements a QFunctionJointBelief using a planner based
 * on alpha functions, for instance the Perseus planners. */
template<class P>
class QAV : public QFunctionJointBelief,
            virtual public QFunctionJAOHInterface
{
private:    

    P *_m_p;

    void Initialize(){};
    void DeInitialize(){};

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QAV(const PlanningUnitDecPOMDPDiscrete* pu) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        { _m_p=new P(pu); }
    
    QAV(const PlanningUnitDecPOMDPDiscrete* pu,
        const QAVParameters &params) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        { _m_p=new P(pu,params); }

    QAV(const PlanningUnitDecPOMDPDiscrete* pu, const BeliefSet &B) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu);
            _m_p->SetBeliefSet(B);
        }

    QAV(const PlanningUnitDecPOMDPDiscrete* pu, const BeliefSet &B,
        const QAVParameters &params) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu,params);
            _m_p->SetBeliefSet(B);
        }

    QAV(const PlanningUnitDecPOMDPDiscrete* pu,
        const std::string &ValueFunctionFile) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu);
            _m_p->SetValueFunction(ValueFunctionFile);
        }

    QAV(const PlanningUnitDecPOMDPDiscrete* pu,
        const std::string &ValueFunctionFile,
        const QAVParameters &params) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu,params);
            _m_p->SetValueFunction(ValueFunctionFile);
        }

    QAV(const PlanningUnitDecPOMDPDiscrete* pu, P* POMDPPlanner) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=POMDPPlanner;
            if(GetPU()->GetHorizon()!=POMDPPlanner->GetPU()->GetHorizon())
                throw(E("QAV ctor: horizons must match"));
        }

    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        { _m_p=new P(pu); }
    
    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
        const QAVParameters &params) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        { _m_p=new P(pu,params); }

    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu, const BeliefSet &B) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu);
            _m_p->SetBeliefSet(B);
        }

    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu, const BeliefSet &B,
        const QAVParameters &params) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu,params);
            _m_p->SetBeliefSet(B);
        }

    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
        const std::string &ValueFunctionFile) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu);
            _m_p->SetValueFunction(ValueFunctionFile);
        }

    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
        const std::string &ValueFunctionFile,
        const QAVParameters &params) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=new P(pu,params);
            _m_p->SetValueFunction(ValueFunctionFile);
        }

    QAV(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu, P* POMDPPlanner) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu)
        {
            _m_p=POMDPPlanner;
            if(GetPU()->GetHorizon()!=POMDPPlanner->GetPU()->GetHorizon())
                throw(E("QAV ctor: horizons must match"));
        }


    /// Destructor.
    ~QAV()
        {
            delete _m_p;
        }

    
    void Compute()
        {
            _m_p->Plan();
        }

    double GetQ(const JointBeliefInterface &b, Index jaI) const
        {
            return(_m_p->GetQ(b,jaI));
        }

    double GetQ(const JointBeliefInterface &b, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const
        {
            return(_m_p->GetQ(b,jaI,betaMaxI));
        }

    double GetQ(const JointBeliefInterface &b, Index t, Index jaI) const
        {
            return(_m_p->GetQ(b,t,jaI));
        }

    double GetQ(const JointBeliefInterface &b, Index t, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const
        {
            return(_m_p->GetQ(b,t,jaI,betaMaxI));
        }

    double GetQ(Index jaohI, Index jaI) const
        {
            Index t = GetPU()->GetTimeStepForJAOHI(jaohI);
            //Index t = GetPU()->GetJointActionObservationHistoryTree(jaohI)->
            //GetJointActionObservationHistory()->GetLength();
            JointBeliefInterface * jb = GetPU()->GetJointBeliefInterface(jaohI);
            double q = _m_p->GetQ (*jb, t, jaI);
            delete jb;
            return( q );
        }

    P* GetPlanner()
        { return(_m_p); }

    virtual void Load(const std::string &filename)
        { _m_p->Load(filename); }

    virtual void Save(const std::string &filename) const
        { _m_p->Save(filename); }

    std::string SoftPrintBrief() const { return("QAV" + _m_p->SoftPrintBrief()); }

};

#endif /* !_QAV_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
