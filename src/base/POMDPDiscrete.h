/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _POMDPDISCRETE_H_
#define _POMDPDISCRETE_H_ 1

/* the include directives */
#include "Globals.h"
#include "DecPOMDPDiscrete.h"

/** \brief POMDPDiscrete models discrete POMDPs. It is basically a
 * wrapper for a Dec-POMDP with a single agent.
 *
 **/
class POMDPDiscrete : public DecPOMDPDiscrete
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        /** Constructor that sets the name, description, and problem file,
         * and subsequently loads this problem file. */
        POMDPDiscrete(const std::string &name="received unspec. by POMDPDiscrete", 
                      const std::string &descr="received unspec. by POMDPDiscrete", 
                      const std::string &pf="received unspec. by POMDPDiscrete");

        /// Copy constructor.
        ///        POMDPDiscrete(const POMDPDiscrete& a);
        /// Destructor.
        virtual ~POMDPDiscrete();
        /// Copy assignment operator
//         POMDPDiscrete& operator= (const POMDPDiscrete& o);

        // these shadow the ones defined in DecPOMDPDiscrete (on purpose)
        size_t GetNrActions() { return(GetNrJointActions()); }
        size_t GetNrObservations() { return(GetNrJointObservations()); }
};


#endif /* !_POMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
