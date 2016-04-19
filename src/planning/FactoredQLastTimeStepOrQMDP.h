/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _FACTOREDQLASTTIMESTEPORQMDP_H_
#define _FACTOREDQLASTTIMESTEPORQMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "FactoredQLastTimeStepOrElse.h"

class QMDP;



/**FactoredQLastTimeStepOrQMDP is a class that represents a Q-Function that
 * is factored for the last stage (i.e., the factored immediate reward function)
 * and the (non-factored) QMDP function for the earlier stages.
 */
class FactoredQLastTimeStepOrQMDP : public FactoredQLastTimeStepOrElse
{
private:

    bool _m_initialized;

    QMDP *_m_QMDP;

protected:
    
public:

    /// (default) Constructor
    FactoredQLastTimeStepOrQMDP(const 
                                PlanningUnitFactoredDecPOMDPDiscrete* puf);
    FactoredQLastTimeStepOrQMDP(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf);

    //Destructor
    ~FactoredQLastTimeStepOrQMDP();

    //implement QFunctionJAOHInterface
    double GetQ(Index jaohI, Index jaI) const;

    void Initialize();
    void DeInitialize();
    void Compute();

    std::string SoftPrintBrief() const { return("FactoredQLastTimeStepOrQBG"); }
};


#endif /* !_FACTOREDQLASTTIMESTEPORQMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
