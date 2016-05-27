/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
