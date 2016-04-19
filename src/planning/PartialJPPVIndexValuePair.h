/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARTIALJPPVINDEXVALUEPAIR_H_
#define _PARTIALJPPVINDEXVALUEPAIR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PartialJointPolicyValuePair.h"
#include "PartialJointPolicyPureVector.h"

//class PartialJointPolicyPureVector;


/**\brief PartialJPPVIndexValuePair represents a (PartialJointPolicyPureVector,Value) pair.
 * It stores the LIindex of the the PartialJointPolicyPureVector.
 *
 * In contrast, JPPVValPair maintains a pointer to the PartialJointPolicyPureVector.
 * This class only stores the LIndex of the PartialJointPolicyPureVector, 
 * allowing to free memory.
 * Therefore, this class should be preferred over JPPVValPair when a lot of
 * policies (i.e., policy items) will be retained in memory. E.g., in MAA* like
 * algorithms.
 *
 * For instance the NewPPI() function in GMAA_MAAstar uses this class (and 
 * consequently frees the memory).
 * */
class PartialJPPVIndexValuePair : public PartialJointPolicyValuePair
{
private:    

    PartialJointPolicyPureVector* _m_jpol;
    LIndex _m_jpolIndex;
    size_t _m_jpolDepth;
    double _m_pastR;
    const Interface_ProblemToPolicyDiscretePure *_m_pu;
    
    void AllocateJPPV();

protected:
    
public:
    // Constructor, destructor and copy assignment.

    PartialJPPVIndexValuePair(PartialJointPolicyPureVector* jp, double val);
    PartialJPPVIndexValuePair(const PartialJointPolicyPureVector& jp, double val);

    /// Destructor.
    ~PartialJPPVIndexValuePair();

    PartialJointPolicyDiscretePure* GetJPol()
    { return GetPartialJPPV();}
    PartialJointPolicyPureVector* GetPartialJPPV();

    std::string SoftPrint() const;
    std::string SoftPrintBrief() const;
};


#endif /* !_PARTIALJPPVINDEXVALUEPAIR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
