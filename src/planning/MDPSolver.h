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
#ifndef _MDPSOLVER_H_
#define _MDPSOLVER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QTable.h"

class PlanningUnitDecPOMDPDiscrete;
class JointBeliefInterface;

/**\brief MDPSolver is an interface for MDP solvers.
 */
class MDPSolver 
{
private:    

    /**A pointer to the PlanningUnit (which can only be a 
     * #const PlanningUnitDecPOMDPDiscrete or derived type).*/
    const PlanningUnitDecPOMDPDiscrete* _m_pu;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    MDPSolver(){};

    MDPSolver(const PlanningUnitDecPOMDPDiscrete& pu): _m_pu(&pu){};
    
    /**Returns a ref to the PlanningUnit.*/
    const PlanningUnitDecPOMDPDiscrete* GetPU() const
        { return(_m_pu); }

    void SetPU(const PlanningUnitDecPOMDPDiscrete& pu){ _m_pu=&pu; }

    /// Destructor.
    virtual ~MDPSolver();
    
    virtual void Plan() = 0;

    virtual void PlanWithCache(bool computeIfNotCached=true) = 0;

    virtual void PlanWithCache(const std::string &filenameCache,
                               bool computeIfNotCached=true) = 0;

    /// Get Q-value for finite-horizon case.
    virtual double GetQ(Index time_step, Index sI,
                        Index jaI) const = 0;

    /// Get Q-value for infinite-horizon case.
    virtual double GetQ(Index sI, Index jaI) const = 0;

    virtual double GetQ(Index time_step, const JointBeliefInterface& jb,
                        Index jaI) const;

    virtual double GetQ(const JointBeliefInterface& jb,
                        Index jaI) const;

    virtual QTables GetQTables() const = 0;

    virtual QTable GetQTable(Index time_step) const = 0;

    virtual void SetQTables(const QTables &Qs) = 0;

    virtual void SetQTable(const QTable &Q, Index time_step) = 0;

    void Print() const;

    Index GetMaximizingAction(Index time_step, Index sI);

    void LoadQTable(const std::string &filename, QTable &Q);

    void LoadQTables(const std::string &filename, int nrTables,
                     QTables &Qs);

};


#endif /* !_MDPSOLVER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
