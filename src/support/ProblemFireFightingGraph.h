/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
#ifndef _PROBLEMFIREFIGHTINGGRAPH_H_
#define _PROBLEMFIREFIGHTINGGRAPH_H_ 1

/* the include directives */
#include "Globals.h"
#include "ProblemFireFightingFactored.h"

/** \brief ProblemFireFightingGraph is an implementation of the
 * FactoredFireFighting problem introduced in (Oliehoek, Spaan,
 * Whiteson, Vlassis, AAMAS 2008).  */
class ProblemFireFightingGraph : public ProblemFireFightingFactored
{
private:    

protected:
    ///used to set the problem name for output files, etc. (in ProblemFireFightingFactored.cpp)
    virtual std::string SoftPrintBriefDescription() const;
    virtual std::string SoftPrintDescription() const;

    ///Construct all the Actions and actionSets (the vector _m_actionVecs).
    void ConstructActions();

    size_t GetAgentLocation(Index action,
                            Index agI) const;
    virtual size_t GetNrAgentsAtHouse(const std::vector< Index>& As,
                                                Index hI) const;

    virtual Scope GetHousesAgentInfluences(Index agI) const;


    //overide scope functions
    virtual void SetYScopes();
    virtual void SetOScopes();

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ProblemFireFightingGraph(size_t nrAgents,
                             size_t nrFireLevels,
                             double multipleAgentExtinguishProb=1.0,
                             bool initialize=true //<- if FFG is the most derived class, it is in charge of 'initializing'
                             );

    /// Destructor.
    virtual ~ProblemFireFightingGraph(){};
    
};


#endif /* !_PROBLEMFIREFIGHTINGGRAPH_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
