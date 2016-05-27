/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BELIEFITERATORINTERFACE_H_
#define _BELIEFITERATORINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief BeliefIteratorInterface is an interface for iterators over
 * beliefs. */
class BeliefIteratorInterface 
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BeliefIteratorInterface(){};

    /// Destructor.
    virtual ~BeliefIteratorInterface(){};

    /// Gets the probability at the current state.
    virtual double GetProbability() const = 0;
    /// Gets the index of the current state.
    virtual Index GetStateIndex() const = 0;
    /// Advance the iterator. Returns false if at the end.
    virtual bool Next() = 0;

    /// Returns a pointer to a copy of this class.
    virtual BeliefIteratorInterface* Clone() const = 0;

};


#endif /* !_BELIEFITERATORINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
