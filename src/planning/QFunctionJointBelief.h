/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QFUNCTIONJOINTBELIEF_H_
#define _QFUNCTIONJOINTBELIEF_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "QFunctionJointBeliefInterface.h"
#include "QFunctionForDecPOMDP.h"

/**\brief QFunctionJointBelief represents a Q-function that operates
 * on joint beliefs. */
class QFunctionJointBelief : 
    public QFunctionJointBeliefInterface //interface
    , virtual public QFunctionForDecPOMDP //implementation
{
private:    
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QFunctionJointBelief(const PlanningUnitDecPOMDPDiscrete *pu)
        : QFunctionForDecPOMDP(pu){}
    QFunctionJointBelief(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
        : QFunctionForDecPOMDP(pu){}

    /// Destructor.
    virtual ~QFunctionJointBelief(){}

    // need to put this because GetQ() in QFunctionJAOHInterface hides the
    // QFunctionJointBeliefInterface one otherwise
    using QFunctionJointBeliefInterface::GetQ;

    /// Returns Q(\a jaohI, \a jaI).
    double GetQ(Index jaohI, Index jaI) const;

    virtual void ComputeWithCachedQValues(bool computeIfNotCached=true) 
        { throw E("QFunctionJointBelief::ComputeWithCachedQValues not yet implemented");}

    virtual void Load(const std::string &filename)
        { throw E("QFunctionJointBelief::Load not yet implemented");}

    virtual void Save(const std::string &filename) const
        { throw E("QFunctionJointBelief::Save not yet implemented");}


};


#endif /* !_QFUNCTIONJOINTBELIEF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
