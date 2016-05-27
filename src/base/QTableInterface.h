/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QTABLEINTERFACE_H_
#define _QTABLEINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief QTableInterface is the abstract base class for Q(., a) functions.
 * It represents functions mapping from some domain (e.g. states, local states,
 * histories, etc.) and some action domain (individual, joint or group actions)
 * to a real number representing some form of payoff (long term reward, or 
 * immediate reward).
 *
 * Note the argument of the functions defined here assume Q(s,a), but is
 * should be clear that for s_i any general domain index may be used.
 *
 * */
class QTableInterface 
{
    private:    
    
    protected:
    
    public:
        virtual double Get(Index s_i, Index ja_i) const = 0;
        virtual void Set(Index s_i, Index ja_i, double rew) = 0;

        virtual ~QTableInterface(){};

        /// Returns a pointer to a copy of this class.
        virtual QTableInterface* Clone() const = 0;

};


#endif /* !_QTABLEINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
