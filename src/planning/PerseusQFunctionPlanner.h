/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PERSEUSQFUNCTIONPLANNER_H_
#define _PERSEUSQFUNCTIONPLANNER_H_ 1

/* the include directives */
#include "Globals.h"
#include "PerseusStationary.h"

class PlanningUnitDecPOMDPDiscrete;

/** \brief PerseusQFunctionPlanner is a Perseus planner that uses
 * QFunctions. */
class PerseusQFunctionPlanner : public PerseusStationary

{
private:    

    virtual QFunctionsDiscrete
    BackupStage(const BeliefSet &S,
                const QFunctionsDiscrete &V) const = 0;
    
protected:
    
public:

    PerseusQFunctionPlanner(const PlanningUnitDecPOMDPDiscrete* pu);
    PerseusQFunctionPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~PerseusQFunctionPlanner();

    void Plan();

};


#endif /* !_PERSEUSQFUNCTIONPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
