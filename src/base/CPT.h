/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _CPT_H_
#define _CPT_H_ 1

/* the include directives */
#include "boost/numeric/ublas/matrix.hpp"
#include "Globals.h"
#include "CPDDiscreteInterface.h"

/// CPT implements a conditional probability table.
/** Uses full matrices. */
class CPT : public CPDDiscreteInterface
{
public:

    typedef boost::numeric::ublas::matrix<double> Matrix;

private:

    Matrix _m_probTable;

protected:
    size_t nrX() const
    { return _m_probTable.size1(); }
    size_t nrY() const
    { return _m_probTable.size2(); }
    
public:
    // Constructor, destructor and copy assignment.
    /// Constructor without arguments, needed for serialization.
    CPT();

    /// Constructor with sizes of sets
    CPT(size_t X, size_t Y);

    /// Copy constructor.
    CPT(const CPT& cpt);

    /// Destructor.
    ~CPT();    
        
    /// Returns \f$ P(x|y) \f$
    double Get(Index x, Index y) const
    { return( _m_probTable(x,y)); }
    
    /// Returns an (index of a) x drawn according to \f$ P(x|y) \f$
    Index Sample (Index y) const;

    //data manipulation funtions:
    ///Sets P(x|y)
    /**x, y are indices of the 'state': e.g. x is an index to the x-th element
     * in X (the set of values x can take)
     * taken joint action and resulting successor state. prob is 
     * the probability. The order of events is s, ja, s', so is the arg. list
     */
    void Set(Index x, Index y, double prob)
        { _m_probTable(x,y)=prob; }

    /// Get a pointer to the CPT.
    const Matrix* GetMatrixPtr() const
        { return(&_m_probTable); }
        
    virtual void SanityCheck() const;

    /// Returns a pointer to a copy of this class.
    virtual CPT* Clone() const
        { return new CPT(*this); }

    std::string SoftPrint() const;

    ///This will randomize the CPT
    void SetRandom();

};

#endif /* !_CPT_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
