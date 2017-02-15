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
#ifndef _MDPPOLICYITERATION_H_
#define _MDPPOLICYITERATION_H_ 1

/* the include directives */
#include <iostream>
#include <float.h>
#include "Globals.h"

#include "MDPSolver.h"
#include "TimedAlgorithm.h"

/**\brief MDPPolicyIteration implements policy iteration for MDPs via GPU.
  */
class MDPPolicyIteration : public MDPSolver,
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


protected:

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    MDPPolicyIteration(){};

    MDPPolicyIteration(const PlanningUnitDecPOMDPDiscrete& pu);
    /// Destructor.
    ~MDPPolicyIteration();

    /// Uses the GetTransitionProbability() interface, which is slow.
    void PlanSlow();


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
