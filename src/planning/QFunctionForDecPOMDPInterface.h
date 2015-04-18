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
#ifndef _QFUNCTIONFORDECPOMDPINTERFACE_H_
#define _QFUNCTIONFORDECPOMDPINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"
#include "QFunctionInterface.h"

class PlanningUnitDecPOMDPDiscrete;
/** \brief QFunctionForDecPOMDPInterface is a class that represents 
 * a Q function for a Dec-POMDP. This is a base class
 * that only stores a pointer to the Dec-POMDP planning unit. */

class QFunctionForDecPOMDPInterface :
   virtual public QFunctionInterface
{
    private:    
        
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        QFunctionForDecPOMDPInterface()
        {};
        /// Destructor.
        virtual ~QFunctionForDecPOMDPInterface(){};
/*
        /// Copy constructor.
        QFunctionForDecPOMDPInterface(const QFunctionForDecPOMDPInterface& a);
        /// Copy assignment operator
        QFunctionForDecPOMDPInterface& operator= (const QFunctionForDecPOMDPInterface& o);
*/
        //operators:

        //data manipulation (set) functions:
        virtual void SetPU(const PlanningUnitDecPOMDPDiscrete* pu) = 0;
        virtual void SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) = 0;

        //get (data) functions:
        virtual const PlanningUnitDecPOMDPDiscrete* GetPU() const = 0;

};


#endif /* !_QFUNCTIONFORDECPOMDPINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
