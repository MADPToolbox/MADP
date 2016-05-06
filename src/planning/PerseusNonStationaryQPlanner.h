/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PERSEUSNONSTATIONARYQPLANNER_H_
#define _PERSEUSNONSTATIONARYQPLANNER_H_ 1

/* the include directives */
#include "Globals.h"
#include "PerseusNonStationary.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief PerseusNonStationaryQPlanner is a Perseus planner that uses
 * non-stationary QFunctions. */
class PerseusNonStationaryQPlanner : public PerseusNonStationary
{
private:    

    virtual QFunctionsDiscrete
    BackupStageAll(const BeliefSet &S,
                const QFunctionsDiscrete &Q) const = 0;

    virtual QFunctionsDiscrete
    BackupStageSampling(const BeliefSet &S,
                        const QFunctionsDiscrete &Q) const = 0;

    void PlanAll();
    void PlanSampling();

protected:
    
public:

    PerseusNonStationaryQPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusNonStationaryQPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~PerseusNonStationaryQPlanner();

    void Plan();

};


#endif /* !_PERSEUSNONSTATIONARYQPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
