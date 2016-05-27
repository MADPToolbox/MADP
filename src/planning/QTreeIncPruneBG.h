/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QTREEINCPRUNEBG_H_
#define _QTREEINCPRUNEBG_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJAOH.h"

class TreeIncPruneBGPlanner;

/**QTreeIncPruneBG implements a QFunctionJAOH using
 * TreeIncPruneBGPlanner.  */
class QTreeIncPruneBG : public QFunctionJAOH
{
private:    

    TreeIncPruneBGPlanner *_m_p;
    
    void Initialize(){};
    void DeInitialize(){};

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QTreeIncPruneBG(const PlanningUnitDecPOMDPDiscrete* pu);
    QTreeIncPruneBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    ~QTreeIncPruneBG();
    
    void Compute();

    double GetQ(Index jaohI, Index jaI) const;

    TreeIncPruneBGPlanner* GetPlanner()
        { return(_m_p); }

    std::string SoftPrintBrief() const { return("QBGTreeIncPrune"); }
};

#endif /* !_QTREEINCPRUNEBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
