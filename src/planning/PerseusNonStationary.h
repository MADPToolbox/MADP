/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
