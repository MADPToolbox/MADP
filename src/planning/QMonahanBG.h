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
#ifndef _QMONAHANBG_H_
#define _QMONAHANBG_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJAOH.h"

class MonahanBGPlanner;



/**QMonahanBG implements a QFunctionJAOH using
 * MonahanBGPlanner.  */
class QMonahanBG : public QFunctionJAOH
{
private:    

    MonahanBGPlanner *_m_p;
    
    void Initialize(){};
    void DeInitialize(){};

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QMonahanBG(const PlanningUnitDecPOMDPDiscrete* pu,
               bool doIncPrune=true);
    QMonahanBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
               bool doIncPrune=true);
    /// Destructor.
    ~QMonahanBG();
    
    void Compute();

    double GetQ(Index jaohI, Index jaI) const;

    MonahanBGPlanner* GetPlanner()
        { return(_m_p); }

    std::string SoftPrintBrief() const { return("QMonahanBG"); }
};

#endif /* !_QMONAHANBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
