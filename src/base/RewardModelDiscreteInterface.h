/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _REWARDMODELDISCRETEINTERFACE_H_
#define _REWARDMODELDISCRETEINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"
#include "QTableInterface.h"

/// RewardModelDiscreteInterface is an interface for discrete reward models.
class RewardModelDiscreteInterface :
    public QTableInterface
{
private:

protected:

public:
    // Constructor, destructor and copy assignment.
    /// default Constructor
    RewardModelDiscreteInterface(){};

    /// Destructor.
    virtual ~RewardModelDiscreteInterface(){};

    /// Returns R(s,ja)
    virtual double Get(Index s_i, Index ja_i) const = 0;

    //data manipulation funtions:
    /// Sets R(s_i,ja_i)
    /** Index ja_i, Index s_i, are indices of the state and taken
     * joint action. r is the reward. The order of events is s, ja, so
     * is the arg. list. */
    virtual void Set(Index s_i, Index ja_i, double rew) = 0;

    /// Returns a pointer to a copy of this class.
    virtual RewardModelDiscreteInterface* Clone() const = 0;

    /// Prints a description of *this* to a string.
    virtual std::string SoftPrint() const = 0;

    ///Print *this* to cout.
    void Print() const
        { std::cout << SoftPrint();}

};

#endif /* !_REWARDMODELDISCRETEINTERFACE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***

