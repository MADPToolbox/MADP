/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BELIEFITERATOR_H_
#define _BELIEFITERATOR_H_ 1

/* the include directives */
#include "Globals.h"
#include "BeliefIteratorInterface.h"
#include "Belief.h"

/** \brief BeliefIterator is an iterator for dense beliefs. */
class BeliefIterator : public BeliefIteratorInterface
{
private:    

    Index _m_i;
    const Belief *_m_belief;

protected:
    
public:

    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BeliefIterator(const Belief *b) : _m_i(0), _m_belief(b){}

    /// Destructor.
    virtual ~BeliefIterator(){}

    double GetProbability() const { return(_m_belief->_m_b[_m_i]); }
    Index GetStateIndex() const { return(_m_i); }
    bool Next()
        {
            if(_m_i>=(_m_belief->_m_b.size()-1))
                return(false);
            else
            {
                _m_i++;
                return(true);
            }
        }

    /// Returns a pointer to a copy of this class.
    virtual BeliefIterator* Clone() const
        { return new BeliefIterator(*this); }

};


#endif /* !_BELIEFITERATOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
