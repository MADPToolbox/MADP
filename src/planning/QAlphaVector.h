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
#ifndef _QALPHAVECTOR_H_
#define _QALPHAVECTOR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJointBelief.h"
#include "ValueFunctionPOMDPDiscrete.h"
#include "AlphaVectorPlanning.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "QFunctionJAOHInterface.h"
#include "BeliefValue.h"

/** \brief QAlphaVector implements a QFunctionJointBelief using an
 * alpha-vector based value function loaded from disk. */
class QAlphaVector : public QFunctionJointBelief,
            virtual public QFunctionJAOHInterface
{
private:    
    
    QFunctionsDiscreteNonStationary _m_Q;
    std::string _m_filename;

    void Initialize();

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QAlphaVector(const PlanningUnitDecPOMDPDiscrete* pu,
                 const std::string &filename) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu),
        _m_filename(filename)
        {
            Initialize();
        }
    QAlphaVector(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                 const std::string &filename) :
        QFunctionForDecPOMDP(pu), //virtual base first
        QFunctionJointBelief(pu),
        _m_filename(filename)
        {
            Initialize();
        }

    /// Destructor.
    ~QAlphaVector()
        {
        }

    
    void Compute()
        {
            // nothing to do here
        }

    double GetQ(const JointBeliefInterface &b, Index jaI) const
        {
            throw(E("QAlphaVector::GetQ not implemented for stationary case"));
        }

    double GetQ(const JointBeliefInterface &b, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const
        {
            throw(E("QAlphaVector::GetQ not implemented for stationary case"));
        }

    double GetQ(const JointBeliefInterface &b, Index t, Index jaI) const
        {
            return(BeliefValue::GetValue(b,_m_Q.at(t).at(jaI)));
        }

    double GetQ(const JointBeliefInterface &b, Index t, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const
        {
            Index maxI=BeliefValue::
                GetMaximizingVectorIndex(b,_m_Q.at(t).at(jaI));
            betaMaxI=_m_Q.at(t).at(jaI).at(maxI).GetBetaI();
            return(b.InnerProduct(_m_Q.at(t).at(jaI).at(maxI).GetValues()));
        }

    double GetQ(Index jaohI, Index jaI) const
        {
            Index t = GetPU()->GetTimeStepForJAOHI(jaohI);
            //Index t = GetPU()->GetJointActionObservationHistoryTree(jaohI)->
            //GetJointActionObservationHistory()->GetLength();
            JointBeliefInterface * jb = GetPU()->GetJointBeliefInterface(jaohI);
            double q = GetQ(*jb, t, jaI);
            delete jb;
            return( q );
        }

    std::string SoftPrintBrief() const { return("QAlphaVector"); }

};

#endif /* !_QALPHAVECTOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
