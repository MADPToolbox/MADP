/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QAVPARAMETERS_H_
#define _QAVPARAMETERS_H_ 1

#include "BGBackupType.h"
#include "PerseusBackupType.h"

struct QAVParameters
{
    double waitPenalty;
    double weight;
    PerseusBackupType backup;
    BGBackupType bgBackupType;
    int commModel;
    bool stationary;
    int falseNegativeObs;
};

#endif /* !_QAVPARAMETERS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
