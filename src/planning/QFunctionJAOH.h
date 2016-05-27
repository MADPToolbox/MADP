/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QFUNCTIONJOINTHISTORY_H_
#define _QFUNCTIONJOINTHISTORY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJAOHInterface.h"
#include "QFunctionForDecPOMDP.h"
#include "QTable.h"

class JointActionObservationHistoryTree;

#define QFunctionJAOH_useIndices 1

/**\brief QFunctionJAOH represents a Q-function that operates on
 * joint action-observation histories. */
class QFunctionJAOH : 
    public QFunctionJAOHInterface //Interface
    , virtual public QFunctionForDecPOMDP //implementation
{
private:    
    
protected:

    ///  Table in which the Qvalues are stored.
    QTable _m_QValues;

    /// See ComputeWithCachedQValues(), this version accepts a filename.
    virtual void ComputeWithCachedQValues(const std::string &filenameCache,
                                          bool computeIfNotCached=true);

public:
    // Constructor, destructor and copy assignment.
    /// Default constructor, requires a planning unit.
    QFunctionJAOH(const PlanningUnitDecPOMDPDiscrete *pu);
    QFunctionJAOH(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~QFunctionJAOH(){};

    virtual void ComputeWithCachedQValues(bool computeIfNotCached=true)
        {
            ComputeWithCachedQValues(GetCacheFilename(),computeIfNotCached);
        }
};


#endif /* !_QFUNCTIONJOINTHISTORY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
