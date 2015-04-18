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
#ifndef _PLANNINGUNITTOIDECPOMDPDISCRETE_H_
#define _PLANNINGUNITTOIDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "TOIDecPOMDPDiscrete.h"

class JointPolicyPureVector;

/** \brief PlanningUnitTOIDecPOMDPDiscrete represents a planning unit for
 * transition observation independent discrete Dec-POMDPs. */
class PlanningUnitTOIDecPOMDPDiscrete : 
    public Referrer<TOIDecPOMDPDiscrete>,
    public PlanningUnitDecPOMDPDiscrete 
{
private:

    bool SanityCheck() const;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    ///the (default) Constructor. 
    PlanningUnitTOIDecPOMDPDiscrete(
        const PlanningUnitMADPDiscreteParameters &params,
        size_t horizon=3,
        TOIDecPOMDPDiscrete* p=0
        );
        
    PlanningUnitTOIDecPOMDPDiscrete(
        size_t horizon=3,
        TOIDecPOMDPDiscrete* p=0
        );

    //operators:
    
    //data manipulation (set) functions:
    
    /// Tell which SetReferred to use by default.
    void SetReferred(TOIDecPOMDPDiscrete* p) 
        {Referrer<TOIDecPOMDPDiscrete>::SetReferred(p);}
    ///Set the problem (TOIDecPOMDPDiscrete) using a pointer.
    void SetProblem(TOIDecPOMDPDiscrete* p);

    //get (data) functions:
    
    /// Returns the TOIDecPOMDPDiscrete pointer.
    /** Tell which GetReferred to use by default... (nl. the
     * Referrer<TOIDecPOMDPDiscrete>::GetReferred(), not the
     * PlanningUnitMADPDiscrete::GetReferred(). */
    TOIDecPOMDPDiscrete* GetReferred() const
        {return(Referrer<TOIDecPOMDPDiscrete>::GetReferred() );}

    /**\brief returns a vector of individual (local) state indices
     * corresponding to joint state index jointSI.*/
    const std::vector<Index>& JointToIndividualStateIndices(Index jointSI) const
        { return(GetReferred()->JointToIndividualStateIndices(jointSI)); }

    /**\brief returns the joint index for indivStateIndices*/
    Index IndividualToJointStateIndices(const std::vector<Index>&
                                        indivStateIndices) const
        { return(GetReferred()->IndividualToJointStateIndices(indivStateIndices)); }

    std::vector<Index> SampleSuccessorState(const std::vector<Index> &sIs,
                                             const std::vector<Index> &aIs)
        const
        { return(GetReferred()->SampleSuccessorState(sIs,aIs)); }
                                             
    std::vector<Index> SampleJointObservation(const std::vector<Index> &aIs,
                                               const std::vector<Index> &sucIs)
        const
        { return(GetReferred()->SampleJointObservation(aIs,sucIs)); }

    std::vector<Index> SampleInitialStates(void) const
        { return(GetReferred()->SampleInitialStates()); }

};


#endif /* !_PLANNINGUNITTOIDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
