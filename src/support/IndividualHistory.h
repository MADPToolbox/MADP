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
#ifndef _INDIVIDUALHISTORY_H_
#define _INDIVIDUALHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "History.h"

///IndividualHistory represents a history for a single agent.
class IndividualHistory : public History
{
private:    
    
protected:
    
    ///The agent this history belongs to.
    Index _m_agentI;
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    IndividualHistory(Index agentI) : _m_agentI(agentI){};
    /// Destructor.
    virtual ~IndividualHistory(){};
    
};


#endif /* !_INDIVIDUALHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
