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
#ifndef _HISTORY_H_
#define _HISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

///History is a general class for histories.
class History 
{
private:    
    
protected:
    
    /// How long (how many time-steps) is this history?
    /** For example, history at... 
     * ts 0 - length 0 (received no action yet... )
     * ts 1 - length 1 (hist= (a0) )
     * ts 2 - length 2 (hist= (a0,a1) ) */
    size_t _m_length;
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    History(){};
    /// Destructor.
    virtual ~History(){};
    
    /// Returns the length of the history, i.e., the number of time steps.
    size_t GetLength() const
        {
            return(_m_length);
        }

    ///Set the length of the history, i.e., the number of time steps.
    void SetLength(size_t length)
        {
            _m_length=length;
        }
    
    /// Returns a pointer to a copy of this class.
    virtual History* Clone() const = 0;

    virtual void Print() const = 0;
};


#endif /* !_HISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
