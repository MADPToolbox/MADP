/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TOIDECMDPDISCRETE_H_
#define _TOIDECMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "TOIDecPOMDPDiscrete.h"

/**\brief TOIDecMDPDiscrete is a class that represents a transition
 * observation indepedent discrete DecMDP. */
class TOIDecMDPDiscrete :
    public TOIDecPOMDPDiscrete
{
private:    
    /**Boolean that tracks whether this TOIDecMDPDiscrete is initialized.*/
    bool _m_initialized;         
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    TOIDecMDPDiscrete(
        const std::string &name="received unspec. by TOI-DecMDPDiscrete", 
        const std::string &descr="received unspec. by TOI-DecMDPDiscrete", 
        const std::string &pf="received unspec. by TOI-DecMDPDiscrete",
        bool cacheFlatModels=false);
    /// Destructor.
    virtual ~TOIDecMDPDiscrete();
    
    /** Sets _m_initialized to b. When setting to true, a verification of
     * member elements is performed. (i.e. a check whether all vectors
     * have the correct size and non-zero entries) */
    bool SetInitialized(bool b);   
    
    /**This function creates the 'state observations'. I.e., for each agent
     * the same number of observations as local states is created. The
     * observation probabilities are set such that in a particular state
     * the corresponding observation is received with certainty.*/
    void CreateStateObservations();
};


#endif /* !_TOIDECMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
