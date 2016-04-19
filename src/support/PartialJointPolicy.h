/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PARTIALJOINTPOLICY_H_
#define _PARTIALJOINTPOLICY_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief PartialJointPolicy represents a joint policy that is only
 * specified for t time steps instead of for every time step.
 *
 * I.e., it is specified for time steps 0 until t, instead of 0 until
 * horizon.
 */
class PartialJointPolicy 
{
    private:    
        ///Stores the past reward the partial policy achieves 
        double _m_pastReward;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        PartialJointPolicy(double r = 0.0)
            : _m_pastReward(r)
        {};

        /// Copy constructor.
        PartialJointPolicy(const PartialJointPolicy& a)
            : _m_pastReward(a._m_pastReward)
        {};
        /// Destructor.
        //~PartialJointPolicy();
        /// Copy assignment operator
        PartialJointPolicy& operator= (const PartialJointPolicy& o);

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:        
        double GetPastReward() const
        { return _m_pastReward;}
        void SetPastReward(double r) 
        { _m_pastReward = r; }


};


#endif /* !_PARTIALJOINTPOLICY_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
