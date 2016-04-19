/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
    
    std::string SoftPrintBriefDescription(
        size_t nrAgents, size_t nrHouses, size_t nrFLs) const;
    std::string SoftPrintDescription(size_t nrAgents,
                                     size_t nrHouses,
                                     size_t nrFLs) const;
    
    ///Construct all the Actions and actionSets (the vector _m_actionVecs).
    void ConstructActions();

    size_t GetAgentLocation(Index action,
                            Index agI) const;
    virtual size_t GetNrAgentsAtHouse(const std::vector< Index>& As,
                                                Index hI) const;

    virtual Scope GetHousesAgentInfluences(Index agI) const;

protected:

    //overide scope functions
    virtual void SetYScopes();
    virtual void SetOScopes();

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ProblemFireFightingGraph(size_t nrAgents,
                             size_t nrFireLevels);

    /// Destructor.
    virtual ~ProblemFireFightingGraph(){};
    
};


#endif /* !_PROBLEMFIREFIGHTINGGRAPH_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
