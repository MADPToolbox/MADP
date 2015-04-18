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
