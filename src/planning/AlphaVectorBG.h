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
#ifndef _ALPHAVECTORBG_H_
#define _ALPHAVECTORBG_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "AlphaVectorPlanning.h"
#include "BGBackupType.h"

class BayesianGameIdenticalPayoff;
class PlanningUnitDecPOMDPDiscrete;
class AlphaVector;



/**AlphaVectorBG implements Bayesian Game specific functionality for
 * alpha-vector based planning.  */
class AlphaVectorBG : virtual public AlphaVectorPlanning
{
private:    

    boost::shared_ptr<BayesianGameIdenticalPayoff> _m_bgip;

    std::vector<std::vector<bool> > GetMask(const ValueFunctionPOMDPDiscrete &V) const;

    AlphaVector BeliefBackupBGIP_Solver(const JointBeliefInterface &b,
                                     Index a,
                                     const GaoVectorSet &G,
                                     const ValueFunctionPOMDPDiscrete &V,
                                     BGBackupType type) const;

    AlphaVector
    BeliefBackupExhaustiveOnlyKeepMax(const JointBeliefInterface &b,
                                      Index a,
                                      const GaoVectorSet &G,
                                      const ValueFunctionPOMDPDiscrete &V) const;

    AlphaVector
    BeliefBackupExhaustiveStoreAll(const JointBeliefInterface &b,
                                   Index a,
                                   const GaoVectorSet &G,
                                   const ValueFunctionPOMDPDiscrete &V) const;

protected:
    
public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorBG(const PlanningUnitDecPOMDPDiscrete* pu);
    AlphaVectorBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    virtual ~AlphaVectorBG();

    AlphaVector BeliefBackup(const JointBeliefInterface &b,
                             Index a,
                             const GaoVectorSet &G,
                             const ValueFunctionPOMDPDiscrete &V,
                             BGBackupType type) const;

    static std::string SoftPrintBackupType(BGBackupType bgBackupType);

};


#endif /* !_ALPHAVECTORBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
