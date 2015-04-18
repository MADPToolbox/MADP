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
#ifndef _JPPVVALUEPAIR_H_
#define _JPPVVALUEPAIR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointPolicyValuePair.h"
#include "boost/shared_ptr.hpp"

class JointPolicyPureVector;
class JointPolicyDiscretePure;

#define DBG_COUNT 0

/**\brief JPPVValuePair represents a (JointPolicyPureVector,Value) pair, which
 * stores the full JointPolicyPureVector. */
class JPPVValuePair : public JointPolicyValuePair
{
private:   
#if DBG_COUNT
    static size_t count_construct;
    static size_t count_destruct;
#endif

    boost::shared_ptr<JointPolicyPureVector> _m_jpol;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.

    JPPVValuePair(const boost::shared_ptr<JointPolicyPureVector> &jp, double value);

    /// Destructor.
    ~JPPVValuePair();

    boost::shared_ptr<JointPolicyDiscretePure> GetJPol() const
    {
        //we want to inherit from JPolValuePair, but this class is not a real
        //PolicyPoolItem...
        throw E("JPPVValuePair::GetJPol should not be called!");
        return(boost::shared_ptr<JointPolicyDiscretePure> ());
    }
    boost::shared_ptr<JointPolicyPureVector> GetJPPV() const
        {return(_m_jpol);}

    std::string SoftPrint() const;
    std::string SoftPrintBrief() const;
};


namespace std{
    /**\brief Overload the less<Type> template for JPolValuePair* (we want less
     * to give an ordering according to values, not addresses...).*/
    template <> 
    struct less< JPPVValuePair * > //struct, so operator() is public by def. 
    {
        bool operator()(const JPPVValuePair* x, const JPPVValuePair* y) const
        { 
            //cout << "specialized less<JPPVValuePair> called!"<<endl;
            return( x->GetValue() < y->GetValue() );
        }

    };
    template <> 
    struct less< boost::shared_ptr<JPPVValuePair> > //struct, so operator() is public by def. 
    {
        bool operator()(const boost::shared_ptr<JPPVValuePair> x, const boost::shared_ptr<JPPVValuePair> y) const
        { 
            //cout << "specialized less<JPPVValuePair> called!"<<endl;
            return( x->GetValue() < y->GetValue() );
        }

    };
}


#endif /* !_JPPVVALUEPAIR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
