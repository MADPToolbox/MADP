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
#ifndef _TIMING_H_
#define _TIMING_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include <time.h>
#include <string>
#include <map>
#include <vector>



/** \brief Timing provides a simple way of timing code. */
class Timing 
{
private:

    /// Stores the start and end of a timespan, in clock cycles.
    struct Times {
        clock_t start, end;
        bool hasEnded;
    };

    /// Keeps track of timing info.
    std::map <std::string, std::vector<Times> > _m_timesMap;
    
    /// The clock cycle at which the class is initialized.
    clock_t _m_timeAtInitialization;

    clock_t GetCpuTime() const;

    double ClockToSeconds(clock_t clockTicks) const;

    void Stop(const std::string & id, clock_t duration);

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    Timing();
    /// Destructor.
    ~Timing();

    /// Start to time an event identified by \a id.
    void Start(const std::string & id);

    /// Stop to time an event identified by \a id.
    void Stop(const std::string & id);

    /// Print stored timing info.
    void Print() const;

    /// Sums data and prints out a summary.
    void PrintSummary() const;

    /// Save collected timing info to file \a filename.
    void Save(const std::string & filename) const;

    /// Save collected timing info to ofstream \a of.
    void Save(std::ofstream &of) const;

    /// Load timing info from file \a filename.
    void Load(const std::string & filename);

    /// Adds event of certain duration, e.g., an external program call.
    void AddEvent(const std::string & id, clock_t duration);

    /// Returns how long ago (in s) a particular event has been started.
    std::vector<double> GetRunningEventDurations(const std::string & id) const;

    /// Returns all stored durations (in s) for a particular event.
    std::vector<double> GetEventDurations(const std::string & id) const;
};


#endif /* !_TIMING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
