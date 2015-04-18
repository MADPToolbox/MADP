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
#ifndef _QFUNCTIONFORDECPOMDP_H_
#define _QFUNCTIONFORDECPOMDP_H_ 1

/* the include directives */
#include "Globals.h"
#include "QFunction.h"
#include "QFunctionForDecPOMDPInterface.h"
#include "boost/shared_ptr.hpp"

class PlanningUnitDecPOMDPDiscrete;
/** \brief QFunctionForDecPOMDP is a class that represents 
 * a Q function for a Dec-POMDP. This is a base class
 * that only stores a pointer to the Dec-POMDP planning unit. */

class QFunctionForDecPOMDP : 
    virtual public QFunctionForDecPOMDPInterface,   //inherited interface
    public QFunction                        //inherited implementation

{
    private:    
        const PlanningUnitDecPOMDPDiscrete* _m_pu;
        boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> _m_puShared;
        
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        QFunctionForDecPOMDP(const PlanningUnitDecPOMDPDiscrete* pu)
            :
                _m_pu(pu) 
        {};
        QFunctionForDecPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
            :
                _m_puShared(pu) 
        {};

        virtual std::string GetCacheFilename() const;
    
        //data manipulation (set) functions:
        void SetPU(const PlanningUnitDecPOMDPDiscrete* pu)
        { _m_pu = pu; }
        void SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
        { _m_puShared = pu; }

        //get (data) functions:
        const PlanningUnitDecPOMDPDiscrete* GetPU() const
        {
            if(_m_pu)
                return(_m_pu);
            else
                return(_m_puShared.get());
        }

};


#endif /* !_QFUNCTIONFORDECPOMDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
