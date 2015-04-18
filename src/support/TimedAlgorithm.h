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
#ifndef _TIMEDALGORITHM_H_
#define _TIMEDALGORITHM_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "Timing.h"

/**\brief TimedAlgorithm allows for easy timekeeping of parts of an
 * algorithm.*/
class TimedAlgorithm 
{
private:    
    
    /// Stores the timing info.
    // This is a pointer, so that timing information can be stored in
    // const functions.
    Timing *_m_timer;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    TimedAlgorithm();
    /// Destructor.
    virtual ~TimedAlgorithm();

    /// Start to time an event identified by \a id.
    void StartTimer(const std::string & id) const;

    /// Stop to time an event identified by \a id.
    void StopTimer(const std::string & id) const;

    /// Print stored timing info.
    void PrintTimers() const;

    /// Sums data and prints out a summary.
    void PrintTimersSummary() const;

    /// Save collected timing info to file \a filename.
    void SaveTimers(const std::string & filename) const;

    /// Save collected timing info to ofstream \a of.
    void SaveTimers(std::ofstream &of) const;

    /// Load timing info from file \a filename.
    void LoadTimers(const std::string & filename);

    /// Adds event of certain duration, e.g., an external program call.
    void AddTimedEvent(const std::string & id, clock_t duration);

    /// Returns all stored durations (in s) for a particular event.
    std::vector<double> GetTimedEventDurations(const std::string & id);

};


#endif /* !_TIMEDALGORITHM_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
