/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
