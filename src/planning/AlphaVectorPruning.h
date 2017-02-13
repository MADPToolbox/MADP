/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
                           std::vector<double> &belief,
                           size_t acceleratedPruningThreshold);
    static bool FindBeliefAccelerated(const AlphaVector &p,
                                      const ValueFunctionPOMDPDiscrete &uU,
                                      std::vector<double> &belief);

    static bool LexGreater(const AlphaVector &alpha1,
                           const AlphaVector &alpha2);

    static int GetVectorIndex(const AlphaVector &p,
                              const ValueFunctionPOMDPDiscrete &uU,
                              std::vector<double> &belief);
    static bool FindBeliefNormal(const AlphaVector &p,
                                 const ValueFunctionPOMDPDiscrete &uU,
                                 std::vector<double> &belief);

    static double GetBeliefDiff(std::vector<double> &belief0, std::vector<double> &belief1);

    static double GetNormalObj(const AlphaVector &p,
                               const ValueFunctionPOMDPDiscrete &uU);
                           

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorPruning();
    /// Destructor.
    virtual ~AlphaVectorPruning();

    static ValueFunctionPOMDPDiscrete Prune(const ValueFunctionPOMDPDiscrete &V,
                                            size_t acceleratedPruningThreshold=200);

};

#endif /* !_ALPHAVECTORPRUNING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
