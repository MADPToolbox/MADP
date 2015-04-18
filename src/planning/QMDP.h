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
#ifndef _QMDP_H_
#define _QMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "MDPValueIteration.h"
#include "QFunctionJAOH.h"
#include "QFunctionJointBeliefInterface.h"

/**\brief QMDP is a class that represents the QMDP heuristic.
 *
 * It is associated with a PlanningUnitDecPOMDPDiscrete which it uses for things
 * as horizon, action-/observation(history) indices, etc. *
 *  */
class QMDP : virtual public QFunctionJAOH,
             virtual public QFunctionJointBeliefInterface
{
private:

    MDPSolver *_m_p;
    
    bool _m_initialized, _m_useJaohQValuesCache;

    void Initialize();
    void DeInitialize();

    void CacheJaohQValues();

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QMDP(const PlanningUnitDecPOMDPDiscrete* pu,
         bool useJaohQValuesCache=false);
    QMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
         bool useJaohQValuesCache=false);
    /// Destructor.
    ~QMDP();
    
    //operators:
    
    //data manipulation (set) functions:
                
    /**Compute the heuristic. (after associated with an initialized 
     * PlanningUnitDecPOMDPDiscrete).
     *
     * Note: unlike QBG and QPOMDP, the QMDP heuristic can be computed 
     * without access to JointActionObservationHistory (Indices) and 
     * JointBeliefs. So there is only 1 compute version. However in
     * order to get the Q for a particular  JointActionObservationHistory 
     * (Index) this information is needed. Therefore there are multiple
     * GetQ functions for this class.*/
    void Compute();
    
    void ComputeWithCachedQValues(const std::string &filename,
                                  bool computeIfNotCached=true);
    
    //get (data) funct        
    /**Return the Qvalue for JointActionObservationHistory Index jaohI and
     * JointAction index jaI.
     */
    double GetQNoCache(Index jaohI, Index jaI) const;
    
    double GetQ(Index jaohI, Index jaI) const
        {  if(_m_useJaohQValuesCache) return (_m_QValues(jaohI,jaI));
            else return(GetQNoCache(jaohI,jaI)); }

    void SetPU(const PlanningUnitDecPOMDPDiscrete* pu);
    void SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    double GetQ(const JointBeliefInterface &b, Index jaI) const
        { return(_m_p->GetQ(b,jaI)); }

    double GetQ(const JointBeliefInterface &b, Index time_step,
                Index jaI) const
        { return(_m_p->GetQ(time_step,b,jaI)); }

    
    double GetQSA(Index t, Index sI, Index jaI) const
        { return(_m_p->GetQ (t, sI, jaI)); }

    
    void Save(const std::string &filename) const;
    
    void Load(const std::string &filename);

    std::string SoftPrintBrief() const { return("QMDP"); }
    
};


#endif /* !_QMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
