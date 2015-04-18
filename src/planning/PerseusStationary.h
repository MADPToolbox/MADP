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
#ifndef _PERSEUSSTATIONARY_H_
#define _PERSEUSSTATIONARY_H_ 1

/* the include directives */
#include "Globals.h"
#include "Perseus.h"

/** \brief PerseusStationary is Perseus for stationary policies. */
class PerseusStationary : public Perseus
{
private:    
    
protected:
    
    /// The resulting value function.
    ValueFunctionPOMDPDiscrete _m_valueFunction;
    /// The resulting Q functions, derived from \a _m_valueFunction.
    QFunctionsDiscrete _m_qFunction;

    /// The belief set.
    BeliefSet *_m_beliefs;

    void StoreValueFunction(const ValueFunctionPOMDPDiscrete &V);
    void StoreValueFunction(const QFunctionsDiscrete &Q);

    void InitializeBeliefs(int nrB, bool uniquify);

    std::vector<double> GetImmediateRewardBeliefSet() const;

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    PerseusStationary(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusStationary(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    virtual ~PerseusStationary();

    /// Sets the belief set \a S that Perseus should use.
    void SetBeliefSet(const BeliefSet &S);

    /// Exports the belief set in a simple matrix format.
    void ExportBeliefSet(const std::string &filename) const;

    /// Sets the value function
    void SetValueFunction(const std::string &filename);

    ValueFunctionPOMDPDiscrete GetValueFunction() const
        { return(_m_valueFunction); }

    QFunctionsDiscrete GetQFunctions() const
        { return(_m_qFunction); }

    double GetQ(const JointBeliefInterface &b, Index jaI) const;

    double GetQ(const JointBeliefInterface &b, Index jaI,
                AlphaVector::BGPolicyIndex &betaMaxI) const;

    double GetQ(const JointBeliefInterface &b, Index t, Index jaI) const;
    double GetQ(const JointBeliefInterface &b, Index t, Index jaI,
               AlphaVector::BGPolicyIndex &betaMaxI) const;

    void ExportValueFunction(const std::string &filename) const;
    void Save(const std::string &filename) const;
    void Load(const std::string &filename);

};



#endif /* !_PERSEUSSTATIONARY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
