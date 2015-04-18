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
#ifndef _QFUNCTIONJOINTBELIEFINTERFACE_H_
#define _QFUNCTIONJOINTBELIEFINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"
#include "QFunctionForDecPOMDPInterface.h"

class JointBeliefInterface;

/** \brief QFunctionJointBeliefInterface is an interface for
 * QFunctionJointBelief. */
class QFunctionJointBeliefInterface 
    : virtual public QFunctionForDecPOMDPInterface
{
private:
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    QFunctionJointBeliefInterface(){}

    /// Destructor.
    virtual ~QFunctionJointBeliefInterface(){}

    /// Returns Q(\a b, \a jaI).
    virtual double GetQ(const JointBeliefInterface &b, Index jaI) const = 0;

    /// Returns Q(\a b, \a jaI) for a particular \a time_step.
    virtual double GetQ(const JointBeliefInterface &b,
                        Index time_step, Index jaI) const = 0;
};


#endif /* !_QFUNCTIONJOINTBELIEFINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
