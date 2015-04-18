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
#ifndef _PLANNINGUNITFACTOREDDECPOMDPDISCRETE_H_
#define _PLANNINGUNITFACTOREDDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "FactoredDecPOMDPDiscreteInterface.h"

class FactoredStateAOHDistribution;
class PartialJointPolicyDiscretePure;

/**PlanningUnitFactoredDecPOMDPDiscrete is a class that represents a
 * planning unit for factored discrete Dec-POMDPs. */
class PlanningUnitFactoredDecPOMDPDiscrete : 
//    public Referrer<FactoredDecPOMDPDiscreteInterface>,
    public PlanningUnitDecPOMDPDiscrete 
{
    private:    
        FactoredDecPOMDPDiscreteInterface* _m_fDecPOMDP;

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /**the (default) Constructor. */
        PlanningUnitFactoredDecPOMDPDiscrete(
                const PlanningUnitMADPDiscreteParameters &params,
                size_t horizon=3,
                FactoredDecPOMDPDiscreteInterface* p=0
            );
        PlanningUnitFactoredDecPOMDPDiscrete(
                size_t horizon=3,
                FactoredDecPOMDPDiscreteInterface* p=0
            );

        //operators:

        //data manipulation (set) functions:

        /**Set the problem (FactoredDecPOMDPDiscreteInterface) using a pointer*/
        void SetProblem(FactoredDecPOMDPDiscreteInterface* p);

        //get (data) functions:

        /**Returns the FactoredDecPOMDPDiscreteInterface pointer.
           Tell which GetFDPOMDPD to use by default... (nl. the 
           Referrer<FactoredDecPOMDPDiscreteInterface>::GetFDPOMDPD(), not the 
           PlanningUnitDecPOMDPDiscrete::GetFDPOMDPD().
        */
        ///GetFDPOMDPDModel is unique name and can be used publicly
        const FactoredDecPOMDPDiscreteInterface* GetFDPOMDPD() const
        {return( _m_fDecPOMDP);}
        
        // Get the vector of Factor indices corresponding to stateI */
        std::vector<Index> StateIndexToFactorValueIndices(Index stateI) const
        {return _m_fDecPOMDP->StateIndexToFactorValueIndices(stateI);}

//factored state distributions etc.:       
        virtual FactoredStateAOHDistribution* 
            GetNewFactoredStateAOHDistribution() const;
        void ComputeFSAOHDist(
                FactoredStateAOHDistribution* fsaoh, 
                const PartialJointPolicyDiscretePure& pJPol) const;



};


#endif /* !_PLANNINGUNITFACTOREDDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***`
