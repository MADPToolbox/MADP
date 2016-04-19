/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _MONAHANPOMDPPLANNER_H_
#define _MONAHANPOMDPPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "MonahanPlanner.h"

/** MonahanPOMDPPlanner implements Monahan's (1982) POMDP algorithm,
 * which basically generates all possible next-step alpha vectors,
 * followed by pruning. The pruning is simply checking the values at
 * each corner of the belief simplex (Eagle's algorithm). It can do a
 * simple form of Incremental Pruning.
 */
class MonahanPOMDPPlanner : public MonahanPlanner
{
private:    

    /// Compute a backup stage.
    virtual QFunctionsDiscrete
    BackupStage(const QFunctionsDiscrete& Q, size_t maxNrAlphas=0);

    void Initialize();

    void MonahanCrossSum(const GaoVectorSet &G,
                         QFunctionsDiscrete &Q,
                         Index a,
                         bool doIncPrune,
                         size_t maxNrAlphas=0) const;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    MonahanPOMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                        bool doIncPrune=true);
    MonahanPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                        bool doIncPrune=true);
    /// Destructor.
    ~MonahanPOMDPPlanner();

    std::string SoftPrintBrief() const { return("MonahanPOMDP"); }
};


#endif /* !_MONAHANPOMDPPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
