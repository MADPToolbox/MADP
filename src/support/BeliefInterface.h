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
#ifndef _BELIEFINTERFACE_H_
#define _BELIEFINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "VectorSet.h"

class BeliefIteratorGeneric;
class StateDistribution;

/** \brief BeliefInterface is an interface for beliefs, i.e.,
 * probability distributions over the state space. */
class BeliefInterface
{
private:
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BeliefInterface(){};
    
    /// Destructor.
    virtual ~BeliefInterface(){};

    /**\brief Copy assignment operator.
     *
     * This must be implemented by the derived class (with this prototype).
     * For an example, see PolicyPoolJPolValPair.
     *
     * For now, this function is purely abstract. Might there be some 
     * members added to this (base) class, then an implementation could
     * be made. This should then be called using
     *          PolicyPoolInterface::operator=(o)
     * from the copy assignment operator of the derived class. See also
     * http://www.icu-project.org/docs/papers/cpp_report/the_assignment_operator_revisited.html.
     */    
    virtual BeliefInterface& operator= (const BeliefInterface& o)=0;

    // operators:

    /// Gets the probability of the i'th state.
    virtual double& operator[] (Index& i) = 0;

    /// Gets the probability of the i'th state.
    virtual double& operator[] (int& i) = 0;

    //data manipulation (set) functions:

    /// Copy the \a belief into this object.
    virtual void Set(const StateDistribution& belief) = 0;

    /// Copy the \a belief into this object.
    virtual void Set(const std::vector<double> &belief) = 0;

    /// Copy the \a belief into this object.
    virtual void Set(const BeliefInterface &belief) = 0;

    /// Sets probability of the \a sI'th state to \a prob .
    virtual void Set(Index sI, double prob) = 0;

    //get (data) functions:

    /// Gets the probability of the sI'th state.
    virtual double Get(Index sI) const = 0;
    /// Gets a vector of probabilities representing the belief.
    virtual std::vector<double> Get() const=0;

    /// Clears the belief.
    virtual void Clear() = 0;

    /// Prints the belief.
    virtual std::string SoftPrint() const = 0;

    /// Prints the belief.
    virtual void Print() const = 0;

    /// Returns the size of this belief.
    virtual unsigned int Size() const = 0;

    /// Checks whether the belief is a valid probability distribution.
    virtual bool SanityCheck() const = 0;

    /// Computes the inner product of a belief with the vector \a values.
    virtual double InnerProduct(const std::vector<double> &values) const = 0;

    /// Computes the inner product of a belief with the VectorSet \a v.
    /** Returns the inner product with each vector in \a v. */
    virtual std::vector<double> InnerProduct(const VectorSet &v) const = 0;
    
    /// Computes the inner product of a belief with the VectorSet \a v.
    /** Returns the inner product with each vector in \a v, but only
     * for vectors whose \a mask is set to true. */
    virtual std::vector<double> InnerProduct(const VectorSet &v,
                                        const std::vector<bool> &mask) const = 0;

    virtual BeliefIteratorGeneric GetIterator() const = 0;

    /// Returns a pointer to a copy of this class.
    virtual BeliefInterface* Clone() const = 0;

};


#endif /* !_BELIEFINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
