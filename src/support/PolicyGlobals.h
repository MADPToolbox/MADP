/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
