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
#ifndef _POLICYGLOBALS_H_
#define _POLICYGLOBALS_H_ 1

/* the include directives */



namespace PolicyGlobals{

    /** \brief The definition of the 'PolicyDomainCategory' type.
     *
     * There are different (currently 4) PolicyDomainCategory's for
     * JointPolicyDiscrete's: A JointPolicyDiscrete is a mapping from
     * domain indices to (prob.distr over) joint actions.  The
     * PolicyDomainCategory's are the possible domains for the domain
     * indices.
     */
    enum PolicyDomainCategory {TYPE_INDEX, OHIST_INDEX, AOHIST_INDEX, STATE_INDEX};

}

#endif /* !_POLICYGLOBALS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
