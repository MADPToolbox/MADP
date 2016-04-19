/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
