/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _ALPHAVECTORPRUNING_H_
#define _ALPHAVECTORPRUNING_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "ValueFunctionPOMDPDiscrete.h"

/**AlphaVectorPruning reduces sets of alpha vectors to their
 * parsimonious representation via LP-based pruning.  */
class AlphaVectorPruning
{
private:

    static ValueFunctionPOMDPDiscrete ParetoPrune(const ValueFunctionPOMDPDiscrete &V);

    static void RemoveFirst(ValueFunctionPOMDPDiscrete &V);
    static void RemoveFirstOccurrence(ValueFunctionPOMDPDiscrete &V,
                                      const AlphaVector &alpha);
    static bool Contains(const ValueFunctionPOMDPDiscrete &V,
                         const AlphaVector &alpha);
    static double InnerProduct(const AlphaVector &alpha,
                               const std::vector<double> &belief);
    
    static bool ParetoDominates(AlphaVector x, AlphaVector y);

    static bool FindBelief(const AlphaVector &p,
                           const ValueFunctionPOMDPDiscrete &uU,
                           std::vector<double> &belief);

    static bool LexGreater(const AlphaVector &alpha1,
                           const AlphaVector &alpha2);

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorPruning();
    /// Destructor.
    virtual ~AlphaVectorPruning();

    static ValueFunctionPOMDPDiscrete Prune(const ValueFunctionPOMDPDiscrete &V);

};

#endif /* !_ALPHAVECTORPRUNING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
