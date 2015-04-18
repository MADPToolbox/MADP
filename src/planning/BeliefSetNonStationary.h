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
#ifndef _BELIEFSETNONSTATIONARY_H_
#define _BELIEFSETNONSTATIONARY_H_ 1

/* the include directives */
#include "Globals.h"
#include "BeliefSet.h"

/** \brief BeliefSetNonStationary represents a non-stationary belief set. */
class BeliefSetNonStationary 
{
private:

    std::vector<BeliefSet> _m_beliefSets;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BeliefSetNonStationary(size_t nrTimeSteps);

    /// Destructor.
    ~BeliefSetNonStationary();

    /// Copy constructor.
    BeliefSetNonStationary (const BeliefSetNonStationary& a);

    /// Copy assignment operator
    BeliefSetNonStationary& operator= (const BeliefSetNonStationary& o);

    size_t Size(Index t) const;
    
    size_t Size() const;

    size_t GetNumberOfTimeSteps() const { return(_m_beliefSets.size()); }
    const BeliefSet& Get(Index t) const { return(_m_beliefSets.at(t)); }

    bool Add(size_t t, const JointBeliefInterface &jb, bool uniquify);

    std::string SoftPrint() const;
    void Print() const { std::cout << SoftPrint(); }
};


#endif /* !_BELIEFSETNONSTATIONARY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
