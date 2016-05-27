/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BELIEFITERATORGENERIC_H_
#define _BELIEFITERATORGENERIC_H_ 1

/* the include directives */
#include "Globals.h"
#include "BeliefIteratorInterface.h"

#define USE_BeliefIteratorGeneric 1

/** \brief BeliefIteratorGeneric is an iterator for beliefs.
 */
class BeliefIteratorGeneric
{
private:    

    BeliefIteratorInterface* _m_it;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BeliefIteratorGeneric(BeliefIteratorInterface *it) :
        _m_it(it)
        {}

    /// Destructor.
    virtual ~BeliefIteratorGeneric()
        {
            delete(_m_it);
        }

    double GetProbability() const { return(_m_it->GetProbability()); }
    Index GetStateIndex() const { return(_m_it->GetStateIndex()); }
    bool Next() const { return(_m_it->Next()); }

};


#endif /* !_BELIEFITERATORGENERIC_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
