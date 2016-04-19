/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _VALUEFUNCTIONPOMDPDISCRETE_H_
#define _VALUEFUNCTIONPOMDPDISCRETE_H_ 1

/* the include directives */

#include "AlphaVector.h"
#include "PrintTools.h"
#include <string>

/// A ValueFunctionPOMDPDiscrete is just a vector of AlphaVector's.
typedef std::vector<AlphaVector> ValueFunctionPOMDPDiscrete;
/// A const_iterator for ValueFunctionPOMDPDiscrete.
typedef ValueFunctionPOMDPDiscrete::const_iterator VFPDcit;

/// A QFunctionsDiscrete is just a vector of
/// ValueFunctionPOMDPDiscrete, one for each action.
typedef std::vector<ValueFunctionPOMDPDiscrete> QFunctionsDiscrete;
typedef QFunctionsDiscrete::const_iterator QFDcit;

/// A QFunctionsDiscreteNonStationary is just a vector of
/// QFunctionsDiscrete, one for each time step.
typedef std::vector<QFunctionsDiscrete> QFunctionsDiscreteNonStationary;
typedef QFunctionsDiscreteNonStationary::const_iterator QFDNScit;

//I'm getting really annoyied with the errors throughout compilation:
//ValueFunctionPOMDPDiscrete.h:55: warning: 'std::string PrintTools::SoftPrint(const QFunctionsDiscrete&)' defined but not used
//ValueFunctionPOMDPDiscrete.h:70: warning: 'std::string PrintTools::SoftPrint(const ValueFunctionPOMDPDiscrete&)' defined but not used

//also why should this be put here? (and not in the PrintTools file?)
//well as long as they aren't used I disable them...
#if 0

namespace PrintTools {

static std::string SoftPrint(const QFunctionsDiscrete &Q)
{
    std::stringstream ss;
    ss << "QFD";
    unsigned int a=0;
    for(QFDcit i=Q.begin();i!=Q.end();++i)
    {
        ss << " a " << a++;
        for(VFPDcit j=i->begin();j!=i->end();++j)
            ss << " bI " << j->GetBetaI() << " "
               << SoftPrintVector(j->GetValues());
    }
    return(ss.str());
}

static std::string SoftPrint(const ValueFunctionPOMDPDiscrete &V)
{
    std::stringstream ss;
    ss << "VFPD";
    for(VFPDcit j=V.begin();j!=V.end();++j)
        ss << " a " << j->GetAction() << " bI " << j->GetBetaI() << " "
           << SoftPrintVector(j->GetValues());
    return(ss.str());
}

}
#endif

#endif /* !_VALUEFUNCTIONPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
