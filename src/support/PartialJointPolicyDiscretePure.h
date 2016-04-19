/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARTIALJOINTPOLICYDISCRETEPURE_H_
#define _PARTIALJOINTPOLICYDISCRETEPURE_H_ 1

/* the include directives */
#include "Globals.h"
#include "JointPolicyDiscretePure.h"
#include "PartialJointPolicy.h"
#include "boost/shared_ptr.hpp"

class PartialJointPolicyDiscretePure;
typedef boost::shared_ptr<PartialJointPolicyDiscretePure> PJPDP_sharedPtr;

/** \brief PartialJointPolicyDiscretePure is a discrete and pure
 * PartialJointPolicy.  */
class PartialJointPolicyDiscretePure :
    //virtual <- this leads to pain in the ass...
    public JointPolicyDiscretePure
    , public PartialJointPolicy
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        PartialJointPolicyDiscretePure()
        {};
        PartialJointPolicyDiscretePure(
                const Interface_ProblemToPolicyDiscretePure* pu,
                PolicyGlobals::PolicyDomainCategory idc,
                double pastReward = 0.0
                )
            : JointPolicyDiscretePure(pu, idc)
              ,PartialJointPolicy(pastReward)
        {};
        PartialJointPolicyDiscretePure(
                const I_PtPDpure_constPtr &pu,
                PolicyGlobals::PolicyDomainCategory idc,
                double pastReward = 0.0
                )
            : JointPolicyDiscretePure(pu, idc)
              ,PartialJointPolicy(pastReward)
        {};
        /// Copy constructor.
        PartialJointPolicyDiscretePure(const PartialJointPolicyDiscretePure& a)
            : 
                JointPolicyDiscretePure(a)
                , PartialJointPolicy(a)
        {};
        /// Destructor.
        virtual ~PartialJointPolicyDiscretePure(){};

        /// Copy assignment operator
        virtual PartialJointPolicyDiscretePure& operator= 
                (const PartialJointPolicyDiscretePure& o);

        /// Returns a pointer to a copy of this class.
        virtual PartialJointPolicyDiscretePure* Clone() const = 0;

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
};


#endif /* !_PARTIALJOINTPOLICYDISCRETEPURE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
