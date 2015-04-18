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
#ifndef _POLICY_H_
#define _POLICY_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PolicyGlobals.h"

///Policy is a class that represents a policy for a single agent.
/** It contains the notion of the depth of a policy: a positive number
 * that specifies for how many time steps this policy is
 * specified. I.e., if depth < horizon, the object represents a
 * partially specified policy (specified for time steps
 * 0,...,depth-1)*/
class Policy 
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
    Index _m_agentI;
    
    public:
    // Constructor, destructor and copy assignment.

    /// Constructor, initializes the depth to the maximum horizon.
    Policy(Index agentI) : 
        _m_depth(MAXHORIZON),
        _m_agentI(agentI)
    {};

    /// Destructor.
    virtual ~Policy(){};

    ///Returns the depth of the policy. \sa _m_depth
    size_t GetDepth() const {return _m_depth;}
    /// Sets the depth of the policy. \sa _m_depth
    virtual void SetDepth(size_t d) {_m_depth = d;}

    /// Returns a pointer to a copy of this class.
    virtual Policy* Clone() const = 0;

    /// Prints a description of a policy to a string.
    virtual std::string SoftPrint() const = 0;
    /// Prints a description of a policy to cout.
    virtual void Print() const
        { std::cout << SoftPrint();}

};


#endif /* !_POLICY_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
