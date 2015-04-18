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
#ifndef _MDPVALUEITERATION_H_
#define _MDPVALUEITERATION_H_ 1

/* the include directives */
#include <iostream>
#include <float.h>
#include "Globals.h"

#include "MDPSolver.h"
#include "TimedAlgorithm.h"

/**\brief MDPValueIteration implements value iteration for MDPs.
  */
class MDPValueIteration : public MDPSolver,
    public TimedAlgorithm
{
private:    
    
    /**_m_QValues represents the non-stationary MDP Q function. 
     * I.e. _m_QValues[t][sI][jaI] gives the expected reward at time-step
     * t (time-to-go = horizon - t). */
    QTables _m_QValues;

    /**Is the MDPValueIteration object initialized?.*/ 
    bool _m_initialized; 
    
    /// Are we solving a finite-horizon problem?
    bool _m_finiteHorizon;

    void Initialize();

    /**Vector<const M*> T is the vector of matrices specifying the transition
       model (one matrix for each joint action). */
    template <class M>
    void Plan(std::vector<const M*> T);

    /// Uses the GetTransitionProbability() interface, which is slow.
    void PlanSlow();

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    MDPValueIteration(){};

    MDPValueIteration(const PlanningUnitDecPOMDPDiscrete& pu);
    /// Destructor.
    ~MDPValueIteration();

    void Plan();

    void PlanWithCache(bool computeIfNotCached=true);

    void PlanWithCache(const std::string &filenameCache, 
                       bool computeIfNotCached=true);

    double GetQ(Index time_step, Index sI,
                Index jaI) const
        { return(_m_QValues[time_step](sI,jaI)); }

    double GetQ(Index sI, Index jaI) const
        { return(_m_QValues[0](sI,jaI)); }

    QTables GetQTables() const;
    QTable GetQTable(Index time_step) const;

    void SetQTables(const QTables &Qs);
    void SetQTable(const QTable &Q, Index time_step);

};


#endif /* !_MDPVALUEITERATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
