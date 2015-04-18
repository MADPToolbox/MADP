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
#ifndef _QFUNCTIONJAOHINTERFACE_H_
#define _QFUNCTIONJAOHINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "QFunctionForDecPOMDPInterface.h"

/**\brief QFunctionJAOHInterface is a class that is an interface for heuristics
 * of the shape
 * Q(JointActionObservationHistory, JointAction)
 *
 * (As these type of heuristics specify only 1 reward, they are (implicitly) for
 * Dec-POMDPs only.)
 **/
class QFunctionJAOHInterface 
    : virtual public QFunctionForDecPOMDPInterface
{
    private:
    
    protected:
    
    public:
        QFunctionJAOHInterface(){}
        /**Destructor. there is a (big) chance that we will call 
         *     delete base_class_pointer
         * where base_class_pointer actually points to a derived object. To 
         * make sure that the derived destructor is called, this destructor is
         * declared virtual.*/
        virtual ~QFunctionJAOHInterface() {};

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        
        /// Returns Q(joint A-O history \a jaohI, \a jaI).
        virtual double GetQ(Index jaohI, Index jaI) const = 0;

};


#endif /* !_QFUNCTIONJAOHINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
