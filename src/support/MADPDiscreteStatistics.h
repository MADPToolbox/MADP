/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _MADPDISCRETESTATISTICS_H_
#define _MADPDISCRETESTATISTICS_H_ 1

/* the include directives */
#include "Globals.h"
#include "MultiAgentDecisionProcessDiscreteInterface.h"

/** \brief MADPDiscreteStatistics is a class that represents 
 * an object that can compute some statistics for a MADP Discrete.
 * These statistics are typically also available from the planning unit
 * but in some cases, one will want to know them *before* initilizing a PU.
 * (e.g., to know whether we can cache the joint beliefs).
 * */
class MADPDiscreteStatistics 
{
    private:    
        MultiAgentDecisionProcessDiscreteInterface* _m_madp;
        size_t _m_h;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        MADPDiscreteStatistics(MultiAgentDecisionProcessDiscreteInterface* madp,
                size_t horizon);
        /// Copy constructor.
        MADPDiscreteStatistics(const MADPDiscreteStatistics& a);
        /// Destructor.
        ~MADPDiscreteStatistics();
        /// Copy assignment operator
        MADPDiscreteStatistics& operator= (const MADPDiscreteStatistics& o);

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        size_t ComputeNrJointActionObservationHistories();
        size_t ComputeEstimatedSizeForCachingJointBeliefs();
};


#endif /* !_MADPDISCRETESTATISTICS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
