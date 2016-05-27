/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _QFUNCTIONFORFACTOREDDECPOMDPINTERFACE_H_
#define _QFUNCTIONFORFACTOREDDECPOMDPINTERFACE_H_ 1

/* the include directives */
#include "Globals.h"
#include "QFunctionForDecPOMDPInterface.h"
class PlanningUnitFactoredDecPOMDPDiscrete;

/** \brief QFunctionForFactoredDecPOMDPInterface is a class that represents 
 * the interface for a QFunction for a Factored DecPOMDP.
 *
 * In particular this interface declares 
 * GetPUF() and SetPUF().
 * */
class QFunctionForFactoredDecPOMDPInterface 
    :
        virtual public QFunctionForDecPOMDPInterface
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        //QFunctionForFactoredDecPOMDPInterface();
        /// Copy constructor.
        //QFunctionForFactoredDecPOMDPInterface(const QFunctionForFactoredDecPOMDPInterface& a);
        /// Destructor.
        virtual ~QFunctionForFactoredDecPOMDPInterface(){};
        /// Copy assignment operator
        //QFunctionForFactoredDecPOMDPInterface& operator= (const QFunctionForFactoredDecPOMDPInterface& o);

        //operators:
        virtual void SetPUF(const PlanningUnitFactoredDecPOMDPDiscrete* pu) = 0;
        virtual void SetPUF(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &pu) = 0;
        virtual const PlanningUnitFactoredDecPOMDPDiscrete* GetPUF() const = 0;

        //data manipulation (set) functions:
        
        //get (data) functions:
};


#endif /* !_QFUNCTIONFORFACTOREDDECPOMDPINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
