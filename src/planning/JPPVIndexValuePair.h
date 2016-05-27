/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _JPPVINDEXVALUEPAIR_H_
#define _JPPVINDEXVALUEPAIR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointPolicyValuePair.h"

class JointPolicyPureVector;
class PartialJointPolicyDiscretePure;
class Interface_ProblemToPolicyDiscretePure;

/**\brief JPPVIndexValuePair represents a (JointPolicyPureVector,Value) pair.
 * It stores the LIndex of the the JointPolicyPureVector.
 *
 * In contrast, JPPVValuePair maintains a pointer to the JointPolicyPureVector.
 * This class only stores the LIndex of the JointPolicyPureVector, 
 * allowing to free memory.
 * Therefore, this class should be preferred over JPPVValuePair when a lot of
 * policies (i.e., policy items) will be retained in memory. E.g., in MAA* like
 * algorithms.
 *
 * For instance the NewPPI() function in GMAA_MAAstar uses this class (and 
 * consequently frees the memory).
 * */
class JPPVIndexValuePair : public JointPolicyValuePair
{
private:    

    JointPolicyPureVector* _m_jpol;
    LIndex _m_jpolIndex;
    size_t _m_jpolDepth;
    const Interface_ProblemToPolicyDiscretePure *_m_pu;
    
    void AllocateJPPV();

protected:
    
public:
    // Constructor, destructor and copy assignment.

    JPPVIndexValuePair(JointPolicyPureVector* jp, double value);
    JPPVIndexValuePair(const JointPolicyPureVector& jp, double value);

    /// Destructor.
    ~JPPVIndexValuePair();

    JointPolicyDiscretePure* GetJPol();
    JointPolicyPureVector* GetJPPV();

    std::string SoftPrint() const;
    std::string SoftPrintBrief() const;
};


#endif /* !_JPPVINDEXVALUEPAIR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
