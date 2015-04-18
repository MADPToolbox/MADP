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
#ifndef _FACTOREDQLASTTIMESTEPORQBG_H_
#define _FACTOREDQLASTTIMESTEPORQBG_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "FactoredQLastTimeStepOrElse.h"

class QBG;



/**FactoredQLastTimeStepOrQBG is a class that represents a Q-Function that
 * is factored for the last stage (i.e., the factored immediate reward function)
 * and the (non-factored) QBG function for the earlier stages.
 * */
class FactoredQLastTimeStepOrQBG : public FactoredQLastTimeStepOrElse
{
private:

    bool _m_initialized;

    QBG *_m_QBG;

protected:
    
public:

    /// (default) Constructor
    FactoredQLastTimeStepOrQBG(const 
                                PlanningUnitFactoredDecPOMDPDiscrete *puf);
    FactoredQLastTimeStepOrQBG(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf);

    //Destructor
    ~FactoredQLastTimeStepOrQBG();

    //implement QFunctionJAOHInterface
    double GetQ(Index jaohI, Index jaI) const;

    void Initialize();
    void DeInitialize();
    void Compute();

    std::string SoftPrintBrief() const { return("FactoredQLastTimeStepOrQBG"); }
};


#endif /* !_FACTOREDQLASTTIMESTEPORQBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
