/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */


#ifndef  TIMETOOLS_INC
#define  TIMETOOLS_INC

#include <sys/time.h>
#include <time.h>

namespace TimeTools{
    ///Returns the difference between start time and current time
    /**Returns a double, time is in microseconds
     */
    double GetDeltaTimeDouble(timeval start_time, timeval cur_time);

}

#endif   /* ----- #ifndef TIMETOOLS_INC  ----- */

