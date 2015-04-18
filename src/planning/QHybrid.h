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
#ifndef _QHYBRID_H_
#define _QHYBRID_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "QFunctionJAOHInterface.h"
#include "QFunctionForDecPOMDP.h"
#include "QTable.h"
#include "qheur.h"
#include "JointBeliefInterface.h"
#include "QFunctionJointBeliefInterface.h"

/**\brief QHybrid is a class that represents the QHybrid heuristic.
 *
 * It is associated with a PlanningUnitDecPOMDPDiscrete which it uses for things
 * as horizon, action-/observation(history) indices, etc.
 */
class QHybrid : public QFunctionJAOHInterface,
                virtual public QFunctionForDecPOMDP //implementation
{
private:
    
    ///  Table in which the Qvalues are stored for the first time steps.
    QTable _m_QValuesFirstTimeSteps;
    /// The type of Qheuristic used for the first time steps.
    qheur::Qheur_t _m_QheurTypeFirstTS;

    ///  Pointer to QFunction used for the last time steps
    QFunctionJointBeliefInterface *_m_QlastTimeSteps;
    size_t _m_horizonLastTimeSteps;
    size_t _m_horizonFirstTimeSteps;
    size_t _m_nrJAOHinFirstTS;
    
    /// Whether _m_horizonLastTimeSteps was externally optimized or not
    bool _m_optimizedHorLast;

    bool _m_initialized; 
    
    void Initialize();
    void DeInitialize();

    /// This function starts the recursive computing of Qvalues.
    void ComputeQ();

    /**Recursively compute the heuristic. This is called by Compute(). */
    double ComputeRecursively(size_t time_step, 
                              LIndex joahI,
                              Index lastJAI);

protected:
    
    public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QHybrid(const PlanningUnitDecPOMDPDiscrete* pu,
            qheur::Qheur_t QheurTypeFirstTimeSteps,
            QFunctionJointBeliefInterface *QlastTimeSteps,
            size_t horizonLastTimeSteps);
    QHybrid(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
            qheur::Qheur_t QheurTypeFirstTimeSteps,
            QFunctionJointBeliefInterface *QlastTimeSteps,
            size_t horizonLastTimeSteps);
    
    /// Destructor.
    virtual ~QHybrid();

    /**Compute the heuristic.*/
    void Compute();
    
    void ComputeWithCachedQValues(bool computeIfNotCached=true);
    
    void Load(const std::string& filename);
    
    void Save(const std::string& filename) const;

    double GetQ(Index jaohI, Index jaI) const;

    double GetQ(const JointBeliefInterface &b, Index time_step, Index jaI) const
        {
            if(time_step<_m_horizonFirstTimeSteps)
                throw(E("QHybrid::GetQ time step still refers to tree-based representation"));
            return(_m_QlastTimeSteps->GetQ(b, time_step-_m_horizonFirstTimeSteps, jaI));
        }


    std::string SoftPrintBrief() const;

    void SetOptimizedHorLast(bool opt) { _m_optimizedHorLast=opt; }

    bool StageRepresentedAsTree(Index ts) const;

};


#endif /* !_QHYBRID_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
