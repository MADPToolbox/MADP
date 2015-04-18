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
