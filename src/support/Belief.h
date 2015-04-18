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
#ifndef _BELIEF_H_
#define _BELIEF_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BeliefInterface.h"
#include "StateDistributionVector.h"

class BeliefIterator;

/// Belief represents a probability distribution over the state space.
/** It is stored as a full vector. */
class Belief : virtual public BeliefInterface
{
private:    
    
    friend class BeliefIterator;

protected:
    
    /// The vector of probability values.
    //std::vector<double> _m_b;
    StateDistributionVector _m_b;

public:

    /// Constructor which sets the \a size of the belief.
    Belief(size_t size=0);

    /// Constructor which copies \a belief in this belief.
    Belief(const std::vector<double> &belief);

    /// Constructor which copies \a belief in this belief.
    Belief(const BeliefInterface &belief);

    Belief(const StateDistribution& belief);
    /// Destructor.
    ~Belief();

    Belief& operator= (const Belief& o);
    BeliefInterface& operator= (const BeliefInterface& o);

    // operators:
    double& operator[] (Index& i) {
        return(_m_b[i]);
    }

    double& operator[] (int& i) {
        return(_m_b[i]);
    }

    //data manipulation (set) functions:

    virtual void Set(const std::vector<double> &belief);

    virtual void Set(Index sI, double prob) { _m_b[sI]=prob; }

    virtual void Set(const BeliefInterface &belief);
    
    virtual void Set(const StateDistribution& belief);

    //get (data) functions:

    double Get(Index sI) const { return(_m_b.at(sI)); };

    std::vector<double> Get() const { return(_m_b); };

    void Clear();

    std::string SoftPrint() const;

    void Print() const { std::cout << SoftPrint(); }

    unsigned int Size() const { return(_m_b.size()); }

    bool SanityCheck() const;

    double InnerProduct(const std::vector<double> &values) const;

    std::vector<double> InnerProduct(const VectorSet &v) const;

    std::vector<double> InnerProduct(const VectorSet &v,
                                const std::vector<bool> &mask) const;

    BeliefIteratorGeneric GetIterator() const;

    /// Returns a pointer to a copy of this class.
    virtual Belief* Clone() const
        { return new Belief(*this); }

};


#endif /* !_BELIEF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
