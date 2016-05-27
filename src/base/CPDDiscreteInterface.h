/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _CPDDISCRETEINTERFACE_H_
#define _CPDDISCRETEINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief CPDDiscreteInterface is an abstract base class that represents 
 * a conditional probability distribution \f$ Pr(x|y) \f$.
 *
 * The interface (so far) only implements Get.
 * In the future 
 * \li we may want to add a function that allows multiplication of
 * CPDDiscreteInterface's ?
 * \li I think it might not be convenient to add Set() functions (each 
 * implementation might use different set functions? E.g. a CPD based on rules
 * may use quite a different mechanism to set probabilities than a CPT)
 *
 * */
class CPDDiscreteInterface 
{
private:    
    
protected:
    
public:
    /// Destructor.
    virtual ~CPDDiscreteInterface(){};

    ///return the probability \f$ Pr(x|y) \f$ of x given y
    virtual double Get(Index x, Index y) const = 0;
    ///set the probability \f$ Pr(x|y) \f$ of x given y
    virtual void Set(Index x, Index y, double p) = 0;
    
    /// Returns an (index of a) x drawn according to \f$ P(x|y) \f$
    virtual Index Sample(Index y) const = 0;

    virtual void SanityCheck() const = 0;
    
    /// Returns a pointer to a copy of this class.
    virtual CPDDiscreteInterface* Clone() const = 0;

    virtual std::string SoftPrint() const = 0;
};

#endif /* !_CPDDISCRETEINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
