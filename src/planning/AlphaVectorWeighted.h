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
