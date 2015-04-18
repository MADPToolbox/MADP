/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _ALPHAVECTORCONSTRAINEDPOMDP_H_
#define _ALPHAVECTORCONSTRAINEDPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorPlanning.h"

class PlanningUnitDecPOMDPDiscrete;
class AlphaVector;



/**AlphaVectorConstrainedPOMDP implements Constrained POMDP specific functionality for
 * alpha-vector based planning. */
class AlphaVectorConstrainedPOMDP : virtual public AlphaVectorPlanning
{
private:    
    
    ///The observation symbol which represents a false negative detection
    ///of an event
    int _m_falseNegativeObs;
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorConstrainedPOMDP(const PlanningUnitDecPOMDPDiscrete* pu, int falseNegativeObs);
    AlphaVectorConstrainedPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu, int falseNegativeObs);

    /// Destructor.
    virtual ~AlphaVectorConstrainedPOMDP();

    /// Computes the AlphaVector resulting from backing up JointBeliefInterface
    /// \a b with the back-projected vectors in \a G.
    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             const GaoVectorSet &G) const;
    
    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G) const;

    VectorSet BeliefBackupQ(const JointBeliefInterface &b,
                            const GaoVectorSet &G,
                            const ValueFunctionPOMDPDiscrete &V = ValueFunctionPOMDPDiscrete()) const;

    int GetFalseNegativeObs()
    {return _m_falseNegativeObs;}
};


#endif /* !_ALPHAVECTORCONSTRAINEDPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
