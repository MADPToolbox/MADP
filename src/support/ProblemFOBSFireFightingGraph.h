/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Philipp Robbel 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _PROBLEMFOBSFIREFIGHTINGGRAPH_H_
#define _PROBLEMFOBSFIREFIGHTINGGRAPH_H_ 1

/* the include directives */
#include "Globals.h"
#include "ProblemFOBSFireFightingFactored.h"


/** \brief ProblemFOBSFireFightingGraph is an implementation of a
 * fully overservable FireFightingGraph problem. 
 */
class ProblemFOBSFireFightingGraph : public ProblemFOBSFireFightingFactored
{
private:    
    
    std::string SoftPrintBriefDescription(
        size_t nrAgents, size_t nrHouses, size_t nrFLs) const;
    std::string SoftPrintDescription(size_t nrAgents,
                                     size_t nrHouses,
                                     size_t nrFLs) const;
    
    ///Construct all the Actions and actionSets (the vector _m_actionVecs).
    void ConstructActions();

    size_t GetAgentLocation(Index action,
                            Index agI) const;
    size_t GetNrAgentsAtHouse(const std::vector< Index>& As,
                                                Index hI) const;

    Scope GetHousesAgentInfluences(Index agI) const;

    //overide scope functions
    void SetYScopes();

public:
    /// (default) Constructor
    ProblemFOBSFireFightingGraph(size_t nrAgents, size_t nrFireLevels);

    /// Destructor.
    virtual ~ProblemFOBSFireFightingGraph(){};
};

#endif /* !_PROBLEMFOBSFIREFIGHTINGGRAPH_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
