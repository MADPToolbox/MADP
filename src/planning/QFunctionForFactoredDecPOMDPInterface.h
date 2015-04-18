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
