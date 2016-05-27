/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _MONAHANBGPLANNER_H_
#define _MONAHANBGPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "MonahanPlanner.h"
#include "AlphaVectorPlanning.h"

typedef boost::multi_array<VectorSet*,3> GaobetaVectorSet;

/**MonahanBGPlanner is the Bayesian Game version of
 * MonahanPOMDPPlanner.  */
class MonahanBGPlanner : public MonahanPlanner
{
private:    
    
    /// Compute a backup stage.
    virtual QFunctionsDiscrete
    BackupStage(const QFunctionsDiscrete &Qs, size_t maxNrAlphas=0);

    /// Compute a backup stage.
    QFunctionsDiscrete
    BackupStageSlow(const QFunctionsDiscrete &Qs);

    GaobetaVectorSet 
    BackProjectMonahanBG(const QFunctionsDiscrete &Qs) const;

    GaobetaVectorSet BackProjectMonahanBG(const ValueFunctionPOMDPDiscrete &V) const;

    void MonahanCrossSum(const GaobetaVectorSet &G,
                         QFunctionsDiscrete &Q,
                         Index a,
                         bool doIncPrune,
                         size_t maxNrAlphas=0) const;

protected:

    VectorSet* ComputeGaoa(const GaoVectorSet &Gao,
                           const ValueFunctionPOMDPDiscrete &V,
                           Index a,
                           Index o,
                           Index aPrime) const;

    GaobetaVectorSet ComputeAllGaoa(const ValueFunctionPOMDPDiscrete &V) const;
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    MonahanBGPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                     bool doIncPrune=true);
    MonahanBGPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                     bool doIncPrune=true);
    /// Destructor.
    ~MonahanBGPlanner();

    void Initialize();

    QFunctionsDiscrete GetQFunctions(size_t horizon)
        { return(_m_qFunction[horizon-1]); }

    ValueFunctionPOMDPDiscrete GetValueFunction(size_t horizon);

    std::string SoftPrintBrief() const { return("MonahanBG"); }

};

#endif /* !_MONAHANBGPLANNER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
