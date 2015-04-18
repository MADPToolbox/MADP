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
#ifndef _BELIEFSPARSE_H_
#define _BELIEFSPARSE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BeliefInterface.h"

#include "boost/numeric/ublas/vector_sparse.hpp"
#include "boost/numeric/ublas/io.hpp"

class BeliefIteratorSparse;
class StateDistribution;

/// BeliefSparse represents a probability distribution over the state space.
/** It is stored as a sparse vector. */
class BeliefSparse : virtual public BeliefInterface
{
private:    

    friend class BeliefIteratorSparse;

protected:

#if BOOST_1_32_OR_LOWER // they renamed sparse_vector to mapped_vector
    typedef boost::numeric::ublas::sparse_vector<double> BS;
#else    
    typedef boost::numeric::ublas::compressed_vector<double> BS;
#endif    

    typedef BS::const_iterator BScit;
    typedef BS::iterator BSit;
    
    /// The sparse vector to store the belief.
    BS _m_b;
    
public:

    /// Default Constructor
    BeliefSparse();

    /// Constructor which sets the \a size of the joint belief.
    BeliefSparse(size_t size);
        
    /// Constructor which copies \a belief in this joint belief.
    BeliefSparse(const std::vector<double> &belief);

    /// Constructor which copies \a belief in this joint belief.
    BeliefSparse(const BeliefInterface &belief);
    BeliefSparse(const StateDistribution& belief);

    /// Destructor.
    ~BeliefSparse();

    // operators:
    BeliefSparse& operator= (const BeliefSparse& o);
    BeliefInterface& operator= (const BeliefInterface& o);

#if BOOST_1_32_OR_LOWER 
    // uses sparse_vector instead of mapped_vector. The former, however, does
    // not define .ref()
    double& operator[] (Index& i) {
        return 
            //_m_b(i); 
            //_m_b.BS::operator[](i);
            *_m_b.find_element(i);
    }

    double& operator[] (int& i) {
        return 
            //_m_b(i); 
            //_m_b.BS::operator()(i);
            *_m_b.find_element(i);
    }
#else    
    double& operator[] (Index& i) {
        return _m_b.ref(i); //_m_b.BS::operator[](i);
    }

    double& operator[] (int& i) {
        return _m_b.ref(i); //_m_b.BS::operator[](i);
    }
#endif

    //data manipulation (set) functions:
    void Set(const BS &belief);

    void Set(const std::vector<double> &belief);

    void Set(Index sI, double prob) { _m_b[sI]=prob; }

    void Set(const BeliefInterface &belief);
    
    virtual void Set(const StateDistribution& belief);

    //get (data) functions:

    double Get(Index sI) const { return(_m_b[sI]); };
    std::vector<double> Get() const 
    { 
        throw E("BeliefSparse::Get() is not yet implemented");
    };

    void Clear();

    std::string SoftPrint() const;

    void Print() const { std::cout << SoftPrint(); }

    unsigned int Size() const { return(_m_b.size()); }

    unsigned int NumberNonZeros() const { return(_m_b.nnz()); }

    bool SanityCheck() const;

    double InnerProduct(const std::vector<double> &values) const;

    std::vector<double> InnerProduct(const VectorSet &v) const;

    std::vector<double> InnerProduct(const VectorSet &v,
                                     const std::vector<bool> &mask) const;

    BeliefIteratorGeneric GetIterator() const;

    /// Returns a pointer to a copy of this class.
    virtual BeliefSparse* Clone() const
        { return new BeliefSparse(*this); }

};


#endif /* !_BELIEFSPARSE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
