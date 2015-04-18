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
