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
#ifndef _ALPHAVECTOR_H_
#define _ALPHAVECTOR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"


/**AlphaVector represent an alpha vector used in POMDP solving. It's
 * basically a vector of values and an action index.  */
class AlphaVector 
{
public:

#if USE_ARBITRARY_PRECISION_INDEX
    typedef LIndex BGPolicyIndex;
#else
    typedef long long int BGPolicyIndex;
#endif

private:    
    
    Index _m_action;
    BGPolicyIndex _m_betaI;
    std::vector<double> _m_values;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.

    /// Default constructor
    AlphaVector();

    /// Constructor that reserves memory for \a nrS values.
    AlphaVector(size_t nrS);

    /// Constructor that reserves memory for \a nrS values, and sets
    /// every value to \a val.
    AlphaVector(size_t nrS, double val);

    /// Destructor.
    ~AlphaVector();
    /// Copy assignment operator
    AlphaVector& operator= (const AlphaVector& o);
    
    //operators:
    friend std::ostream& operator<< (std::ostream& o, const AlphaVector& v);
    //data manipulation (set) functions:

    /// Set the action index this alpha vector represents to \a a.
    void SetAction(Index a){ _m_action=a; }
    /// Set the values over the state-space this alpha vector represents to \a vs.
    void SetValues(const std::vector<double> &vs);
    /// Set the value for state \a i to value \a v.
    void SetValue(double v,Index i){ _m_values.at(i)=v; }
    /// Set the Bayesian-Game-policy index to \a betaI.
    void SetBetaI(BGPolicyIndex betaI)
        {
            if(betaI<-1)
                throw(E("AlphaVector::SetBetaI() BG policy index has to be >=-1"));
            _m_betaI=betaI;
        }
    
    //get (data) functions:
    /// Get the action index of this alpha vector.
    Index GetAction() const { return(_m_action); }
    /// Get the Bayesian-Game-policy index of this alpha vector.
    BGPolicyIndex GetBetaI() const { return(_m_betaI); }
    /// Get a reference to the values this alpha vector contains.
    const std::vector<double> &GetValues() const { return(_m_values); }
    /// Get the value for a particular state index \a i.
    double GetValue(Index i) const { return(_m_values.at(i)); }

    /// Get the number of values represented by this alpha vector (state space size).
    unsigned int GetNrValues() const { return(_m_values.size()); }

    /// Get a textual representation of this alpha vector.
    std::string SoftPrint() const;
    /// Print out a textual representation of this alpha vector.
    void Print() const { std::cout << SoftPrint(); }

    /// Compare whether this alpha vector is equal to \a alpha.
    bool Equal(const AlphaVector &alpha) const;


    /// Compare whether this alpha vector's values are equal to those of \a alpha.
    bool EqualValues(const AlphaVector &alpha) const;
    
    /// Add B to this and return as a new vector
    AlphaVector Add(const AlphaVector& B) const;
    /// Add B to this (in place Add)
    void IAdd(const AlphaVector& B);
};

#endif /* !_ALPHAVECTOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
