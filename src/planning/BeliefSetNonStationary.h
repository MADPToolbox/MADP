/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
