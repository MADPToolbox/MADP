/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BELIEFITERATORSPARSE_H_
#define _BELIEFITERATORSPARSE_H_ 1

/* the include directives */
#include "Globals.h"
#include "BeliefIteratorInterface.h"
#include "BeliefSparse.h"

#define BeliefIteratorSparse_UseCIT 1

/** \brief BeliefIteratorSparse is an iterator for sparse beliefs. */
class BeliefIteratorSparse : public BeliefIteratorInterface
{
private:    

#if BeliefIteratorSparse_UseCIT
    BeliefSparse::BScit _m_it;
#else
    Index _m_i;
#endif
    const BeliefSparse *_m_belief;

protected:
    
public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
#if BeliefIteratorSparse_UseCIT
    BeliefIteratorSparse(const BeliefSparse *b) : _m_belief(b)
        {
            _m_it=_m_belief->_m_b.begin();
            if(_m_belief->Size()==0)
                throw(E("BeliefIteratorSparse ctor: belief has size 0"));
            if(_m_belief->NumberNonZeros()==0)
                throw(E("BeliefIteratorSparse ctor: belief is empty"));
        }
#else
    BeliefIteratorSparse(const BeliefSparse *b) : _m_i(0), _m_belief(b)
        {
            if(_m_belief->_m_b.nnz()==0)
                throw(E("BeliefIteratorSparse ctor: belief is empty"));
        }
#endif

    /// Destructor.
    virtual ~BeliefIteratorSparse(){}

#if BeliefIteratorSparse_UseCIT
    double GetProbability() const { return(*_m_it); }
    Index GetStateIndex() const { return(_m_it.index()); }
    bool Next()
        {
            _m_it++;
            if(_m_it==_m_belief->_m_b.end())
                return(false);
            else
                return(true);
        }
#else
    double GetProbability() const { return(_m_belief->
                                           _m_b.value_data()[_m_i]); }
    Index GetStateIndex() const { return(_m_belief->
                                         _m_b.index_data()[_m_i]); }
    bool Next()
        {
            if(_m_i>=(_m_belief->_m_b.nnz()-1))
                return(false);
            else
            {
                _m_i++;
                return(true);
            }
        }
#endif

    /// Returns a pointer to a copy of this class.
    virtual BeliefIteratorSparse* Clone() const
        { return new BeliefIteratorSparse(*this); }


};

#endif /* !_BELIEFITERATORSPARSE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
