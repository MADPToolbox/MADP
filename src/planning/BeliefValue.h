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
#ifndef _BELIEFVALUE_H_
#define _BELIEFVALUE_H_ 1

/* the include directives */
#include "BeliefSet.h"
#include "VectorSet.h"
#include "ValueFunctionPOMDPDiscrete.h"

class AlphaVector;
class JointBeliefInterface;
class BeliefSetNonStationary;

/**BeliefValue is a namespace for functions that compute the value of
 * a particular belief.*/
namespace BeliefValue 
{
    /// Get the values of the \a Beliefs for value function \a V.
    std::vector<double> GetValues(const BeliefSet &Beliefs,
                                  const ValueFunctionPOMDPDiscrete &V);
    /// Get the values of the \a Beliefs for alpha vector \a alpha.
    std::vector<double> GetValues(const BeliefSet &Beliefs,
                                  const AlphaVector &alpha);

    /// Get the values of the \a Beliefs for q functions \a Q.
    std::vector<double> GetValues(const BeliefSet &Beliefs,
                                  const QFunctionsDiscrete &Q);

    /// Get the values of the \a Beliefs for non-stationary q functions \a Q.
    std::vector<double> GetValues(const BeliefSetNonStationary &Beliefs,
                                  const QFunctionsDiscreteNonStationary &Q);

    /// Get the values of the \a Beliefs for value function \a V.
    std::vector<double> GetValues(const BeliefSetNonStationary &Beliefs,
                                  const ValueFunctionPOMDPDiscrete &V);

    /// Get the value of a single \a Belief for alpha vecotr \a alpha.
    double GetValue(const BeliefInterface &Belief,
                    const AlphaVector &alpha);

    /// Get the value of a single \a Belief for value function \a V.
    double GetValue(const BeliefInterface &Belief,
                    const ValueFunctionPOMDPDiscrete &V);

    /// Get the value of a single \a Belief for q functions \a Q.
    double GetValue(const BeliefInterface &Belief,
                    const QFunctionsDiscrete &Q);

    /// Get the value of a single \a Belief for non-stationary q functions \a Q.
    double GetValue(const BeliefInterface &Belief,
                    const QFunctionsDiscreteNonStationary &Q,
                    Index t);

    /// Get the value of a single \a Belief for \a V given the \a mask.
    double GetValue(const BeliefInterface &Belief,
                    const ValueFunctionPOMDPDiscrete &V,
                    const std::vector<bool> mask);
    
    /// Get the value of a \a Belief for \a v given the \a mask.
    double GetValue(const BeliefInterface &Belief,
                    const VectorSet &v,
                    const std::vector<bool> mask);

    /** Returns the index of the vector in \a v that maximizes the
     *  value of \a b. */
    int GetMaximizingVectorIndex(const BeliefInterface &b, 
                                 const VectorSet &v);

    int GetMaximizingVectorIndex(const BeliefInterface &b, 
                                 const ValueFunctionPOMDPDiscrete &V);

    /** Returns the index of the vector in \a v that maximizes the
     * value of \a b. Only vectors whose \a mask is true will be
     * considered. */
    int GetMaximizingVectorIndex(const BeliefInterface &b, 
                                 const VectorSet &v,
                                 const std::vector<bool> &mask);
    
    int GetMaximizingVectorIndexAndValue(const BeliefInterface &b, 
                                         const VectorSet &v,
                                         const std::vector<bool> &mask,
                                         double &value);
    
    /** Returns the alpha vector from \a V that maximizes the value of
     *  the \a k 'th belief in set \a S. */
    AlphaVector GetMaximizingVector(const BeliefSet &S,
                                    int k,
                                    const ValueFunctionPOMDPDiscrete &V);
}

#endif /* !_BELIEFVALUE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
