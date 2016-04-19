/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _FACTOREDQLASTTIMESTEPORQPOMDP_H_
#define _FACTOREDQLASTTIMESTEPORQPOMDP_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "FactoredQLastTimeStepOrElse.h"

class QPOMDP;



/**FactoredQLastTimeStepOrQPOMDP is a class that represents a Q-Function that
 * is factored for the last stage (i.e., the factored immediate reward function)
 * and the (non-factored) QPOMDP function for the earlier stages. 
 */
class FactoredQLastTimeStepOrQPOMDP : public FactoredQLastTimeStepOrElse
{
private:

    bool _m_initialized;

    QPOMDP *_m_QPOMDP;

protected:
    
public:

    /// (default) Constructor
    FactoredQLastTimeStepOrQPOMDP(const 
                                PlanningUnitFactoredDecPOMDPDiscrete* puf);
    FactoredQLastTimeStepOrQPOMDP(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf);

    //Destructor
    ~FactoredQLastTimeStepOrQPOMDP();

    //implement QFunctionJAOHInterface
    double GetQ(Index jaohI, Index jaI) const;

    void Initialize();
    void DeInitialize();
    void Compute();

    std::string SoftPrintBrief() const { return("FactoredQLastTimeStepOrQPOMDP"); }
};


#endif /* !_FACTOREDQLASTTIMESTEPORQPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
