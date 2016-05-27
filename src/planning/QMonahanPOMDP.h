/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QMONAHANPOMDP_H_
#define _QMONAHANPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJAOH.h"

class MonahanPOMDPPlanner;



/** QMonahanPOMDP implements a QFunctionJAOH using
 * MonahanPOMDPPlanner. */
class QMonahanPOMDP : public QFunctionJAOH
{
private:    

    MonahanPOMDPPlanner *_m_p;
    
    void Initialize(){};
    void DeInitialize(){};

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QMonahanPOMDP(const PlanningUnitDecPOMDPDiscrete* pu,
                  bool doIncPrune=true);
    QMonahanPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                  bool doIncPrune=true);
    /// Destructor.
    ~QMonahanPOMDP();
    
    void Compute();

    double GetQ(Index jaohI, Index jaI) const;

    std::string SoftPrintBrief() const { return("QMonahanPOMDP"); }
};


#endif /* !_QMONAHANPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
