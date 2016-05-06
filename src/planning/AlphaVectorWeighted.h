/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ALPHAVECTORWEIGHTED_H_
#define _ALPHAVECTORWEIGHTED_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorBG.h"
#include "AlphaVectorPOMDP.h"

/// AlphaVectorWeighted implements a weighted BG/POMDP backup.
/** Returns a vector which is _m_weight*POMDP + (1-_m_weight)*BG. */
class AlphaVectorWeighted :
    public AlphaVectorPOMDP,
    public AlphaVectorBG
{
private:    

    bool _m_initialized;
    std::vector<double> _m_weights;
    std::vector<std::vector<double> > _m_weightsBackProjected;

    void BackProjectWeights();

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorWeighted(const PlanningUnitDecPOMDPDiscrete* pu,
                        double weight);

    AlphaVectorWeighted(const PlanningUnitDecPOMDPDiscrete* pu,
                        std::vector<double> weights);

    AlphaVectorWeighted(const PlanningUnitDecPOMDPDiscrete* pu);

    AlphaVectorWeighted(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                        double weight);

    AlphaVectorWeighted(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                        std::vector<double> weights);

    AlphaVectorWeighted(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~AlphaVectorWeighted();

    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G,
                             const QFunctionsDiscrete &Q) const;

    void SetWeights(std::vector<double> weights);
};


#endif /* !_ALPHAVECTORWEIGHTED_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
