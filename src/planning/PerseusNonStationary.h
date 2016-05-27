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

/* Only include this header file once. */
#ifndef _PERSEUSNONSTATIONARY_H_
#define _PERSEUSNONSTATIONARY_H_ 1

/* the include directives */
#include "Globals.h"
#include "Perseus.h"

/** \brief PerseusNonStationary is Perseus for non-stationary policies. */
class PerseusNonStationary : public Perseus
{
private:    
    
protected:
    
    /// The resulting Q functions.
    QFunctionsDiscreteNonStationary _m_qFunctions;

    /// The non-stationary belief set.
    BeliefSetNonStationary *_m_beliefs;

    std::vector<double> GetImmediateRewardBeliefSet() const;

    void InitializeBeliefs(int nrB, bool uniquify);

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusNonStationary(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusNonStationary(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    virtual ~PerseusNonStationary();

    /// Sets the belief set \a S that Perseus should use.
    void SetBeliefSet(const BeliefSetNonStationary &S);

    /// Sets the value function
    void SetValueFunction(const std::string &filename);

    void StoreValueFunction(const QFunctionsDiscreteNonStationary &Q);

    void ExportValueFunction(const std::string &filename) const;
    void Save(const std::string &filename) const;
    void Load(const std::string &filename);

    double GetQ(const JointBeliefInterface &b, Index jaI) const;
    double GetQ(const JointBeliefInterface &b, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const;
    double GetQ(const JointBeliefInterface &b, Index t, Index jaI) const;
    double GetQ(const JointBeliefInterface &b, Index t, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const;
};


#endif /* !_PERSEUSNONSTATIONARY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
