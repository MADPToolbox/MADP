/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _SIMULATION_H_
#define _SIMULATION_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include <limits.h>
#include <stdlib.h>

/** \brief Simulation is a class that simulates policies in order to
 * test their control quality. */
class Simulation 
{
private:

    int _m_nrRuns;

    int _m_random_seed;

    bool _m_verbose;

protected:

    static const int illegalRandomSeed=INT_MAX;
    
public:
    // Constructor, destructor and copy assignment.

    /// Constructor that specifies the number of runs and the random seed.
    Simulation(int nrRuns, int seed=illegalRandomSeed) :
        _m_nrRuns(nrRuns),
        _m_random_seed(seed),
        _m_verbose(false)
        {}


    /// Destructor.
    virtual ~Simulation(){};

    void SetVerbose(bool verbose) { _m_verbose=verbose; }

    bool GetVerbose() const { return(_m_verbose); }
    int GetNrRuns() const { return(_m_nrRuns); }
    int GetRandomSeed() const { return(_m_random_seed); }
    void SetRandomSeed( int s) { _m_random_seed = s; srand(s); }

};

#endif /* !_SIMULATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
