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
#ifndef _JOINTPOLICY_H_
#define _JOINTPOLICY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#define DEBUG_JPOLASSIGN 0

///JointPolicy is a class that represents a joint policy.
/** It contains the notion of the depth of a policy: a positive number
 * that specifies for how many time steps this policy is
 * specified. I.e., if depth < horizon, the object represents a
 * partially specified policy (specified for time steps
 * 0,...,depth-1)*/
class JointPolicy
{
private:    

    /**\brief The depth of this joint policy.
     *
     * The depth of the policy is the number of stages for which it specifies actions.
     * It ranges from: \n
     *     0 (the empty policy), \n
     *     1 (a policy specified only for ts=0), \n
     *     :
     *     MAXHORIZON (a policy specified for all stages, the default value). \n
     */
    size_t _m_depth;

protected:
    ///some other numbers we cache:
    size_t _m_nrAgents;
  
public:
    // Constructor, destructor and copy assignment.

    /// Constructor, initializes the depth to the maximum horizon.
    JointPolicy(size_t nrAg) : 
        _m_depth(MAXHORIZON),
        _m_nrAgents(nrAg)
    {};
    /// copy constructor:
    JointPolicy(const JointPolicy& o)
        : _m_depth(o._m_depth),_m_nrAgents(o._m_nrAgents)
    {};
    /// copy assignment operator
    virtual JointPolicy& operator= (const JointPolicy& o)
    {
#if DEBUG_JPOLASSIGN 
std::cerr << "JointPolicy& JointPolicy::operator= (const JointPolicy& o) called\n"<<std::endl;
#endif
        if (this == &o) return *this;   // Gracefully handle self assignment
        _m_depth = o._m_depth;
        _m_nrAgents = o._m_nrAgents;
        return *this;
    }
    
    /// less-than operator. 
    /** In many cases, it will be necessary to order Joint Policies)
     */
    virtual bool operator< (const JointPolicy& o) const = 0;
 
    ///Returns the depth of the joint policy.  \sa _m_depth
    size_t GetDepth() const {return _m_depth;}
    /// Sets the depth of the joint policy.  \sa _m_depth
    virtual void SetDepth(size_t d) {_m_depth = d;}

    /// Returns the number of agents for which the joint policy is defined.
    size_t GetNrAgents() const 
    { return _m_nrAgents; }

    /// Destructor.
    virtual ~JointPolicy(){}
      
    /// Returns a pointer to a copy of this class.
    virtual JointPolicy* Clone() const = 0;

    /// Prints a description of a joint policy to a string.
    virtual std::string SoftPrint() const = 0;
    /// Prints a description of a joint policy to cout.
    virtual void Print() const
        { std::cout << SoftPrint();}
};


#endif /* !_JOINTPOLICY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
