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
