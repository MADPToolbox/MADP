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
#ifndef _PARTIALJOINTPOLICYVALUEPAIR_H_
#define _PARTIALJOINTPOLICYVALUEPAIR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "PartialPolicyPoolItemInterface.h"

/**\brief PartialJointPolicyValuePair is a wrapper for a partial joint
 *policy and its heuristic value.
 *
 *PartialJointPolicyValuePair is a class that represents a (joint policy,value) pair
 * the operator less is defined for these pairs so they can be but in ordered
 * containers. 
 *
 *PartialJointPolicyValuePair is a PolicyPoolItem (i.e., it implements PartialPolicyPoolItemInterface)
 * */
class PartialJointPolicyValuePair : public PartialPolicyPoolItemInterface                    
{   
    private:
        double _m_val;

    protected:
    public:
        /// (default) Constructor
        PartialJointPolicyValuePair(double val)
        {
            _m_val = val;
        }                

        /// Destructor.
        ~PartialJointPolicyValuePair(){};

        /// Copy assignment operator
        PartialJointPolicyValuePair& operator= (const PartialJointPolicyValuePair& o)
        {
            throw E("Assigning PartialJointPolicyValuePair - but operator= not def'd");
        }

        double GetValue() const
            {return(_m_val);}   

        void SetValue(double value)
            { _m_val=value; }
};

namespace std{
    /**Overload the less<Type> template for PartialJointPolicyValuePair* (we want less
     * to give an ordering according to values, not addresses...).*/
    template <> 
    struct less< PartialJointPolicyValuePair* > //struct, so operator() is public by def. 
    {
        bool operator()(const PartialJointPolicyValuePair* x, const PartialJointPolicyValuePair* y) const
        { 
            //cout << "specialized less<PartialJointPolicyValuePair> called!"<<endl;
            return( x->GetValue() < y->GetValue() );
        }

    };
    template <> 
    struct less< boost::shared_ptr<PartialJointPolicyValuePair> > //struct, so operator() is public by def. 
    {
        bool operator()(const boost::shared_ptr<PartialJointPolicyValuePair> x,
                        const boost::shared_ptr<PartialJointPolicyValuePair> y) const
        { 
            //cout << "specialized less<PartialJointPolicyValuePair> called!"<<endl;
            return( x->GetValue() < y->GetValue() );
        }

    };
}

#endif /* !_PARTIALJPOLVALPAIR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
