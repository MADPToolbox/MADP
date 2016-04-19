/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "TimeTools.h"

namespace TimeTools{

double GetDeltaTimeDouble(timeval start_time, timeval cur_time)
{
    if(gettimeofday(&cur_time, NULL) != 0)
        throw "Error with gettimeofday";

    time_t delta_sec = cur_time.tv_sec - start_time.tv_sec;
    suseconds_t delta_usec = cur_time.tv_usec - start_time.tv_usec;
    double delta = 1000000.0 * delta_sec + delta_usec; //in microsecond
    return delta;
}

}



