/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _DISCRETEENTITY_H_
#define _DISCRETEENTITY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

/// DiscreteEntity is a general class for tracking discrete entities.
/**DiscreteEntity represents entities in discrete spaces, that hence
 * can be represented by an index. For example, actions in a finite
 * action space. */
class DiscreteEntity 
{
private:
    

    /// The index of this discrete entity.
    Index _m_index;

protected:

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    DiscreteEntity(Index i=INDEX_MAX) : _m_index(i){}

    /// Destructor.
    virtual ~DiscreteEntity(){}

    /// Return this DiscreteEntity's index.
    Index GetIndex() const { return(_m_index); }

    /// Set this DiscreteEntity's index.
    void SetIndex(Index i) { _m_index=i; }

    /// The less (<) operator. This is needed to put DiscreteEntities in a set.
    bool operator< (const DiscreteEntity& a) const {
        return( _m_index < a._m_index );}

};


#endif /* !_DISCRETEENTITY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
