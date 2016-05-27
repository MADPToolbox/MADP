/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PDDISCRETEINTERFACE_H_
#define _PDDISCRETEINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief PDDiscreteInterface is an abstract base class that represents 
 * a joint probability distribution \f$ Pr(x_1,\dots,x_k ) \f$.
 *
 * The interface (so far) only implements Get.
 * In the future 
 * \li we may want to add a function that allows multiplication of
 * PDDiscreteInterface's ?
 * \li I think it might not be convenient to add Set() functions (each 
 * implementation might use different set functions? E.g. a PD based on rules
 * may use quite a different mechanism to set probabilities than a CPT)
 *
 * */
class PDDiscreteInterface 
{
private:    
    
protected:
    
public:
    /// Destructor.
    virtual ~PDDiscreteInterface(){};

    ///return the probability \f$ Pr(x) \f$ 
    virtual double Get(Index x) const = 0;
    virtual double Get(const std::vector<Index>& indices) const = 0;
    virtual double Get(const Scope& sc, 
                              const std::vector<Index>& indices_sc) const=0;

    ///set the probability \f$ Pr(x) \f$ of x 
    virtual void Set(Index x, double p) = 0;
    
    /// Returns a (joint index of an) x drawn \f$ P(x) \f$
    virtual Index Sample() const = 0;

    virtual void SanityCheck() const = 0;
    
    /// Returns a pointer to a copy of this class.
    virtual PDDiscreteInterface* Clone() const = 0;

    virtual std::string SoftPrint() const = 0;
};

#endif /* !_PDDISCRETEINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
