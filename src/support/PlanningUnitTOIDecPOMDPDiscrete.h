/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _PLANNINGUNITTOIDECPOMDPDISCRETE_H_
#define _PLANNINGUNITTOIDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "TOIDecPOMDPDiscrete.h"

class JointPolicyPureVector;

/** \brief PlanningUnitTOIDecPOMDPDiscrete represents a planning unit for
 * transition observation independent discrete Dec-POMDPs. */
class PlanningUnitTOIDecPOMDPDiscrete : 
    public PlanningUnitDecPOMDPDiscrete 
{
private:

    bool SanityCheck() const;

    ///ref to the pu
    TOIDecPOMDPDiscrete* _m_TOIDecPOMDPDiscrete;

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
    {_m_TOIDecPOMDPDiscrete=p;}

    ///Set the problem (TOIDecPOMDPDiscrete) using a pointer.
    void SetProblem(TOIDecPOMDPDiscrete* p);

    TOIDecPOMDPDiscrete* GetTOIDecPOMDPDiscrete()
    {return _m_TOIDecPOMDPDiscrete;}

    //get (data) functions:
    
    /// Returns the TOIDecPOMDPDiscrete pointer.
    /** Tell which GetReferred to use by default... (nl. the
     * Referrer<TOIDecPOMDPDiscrete>::GetReferred(), not the
     * PlanningUnitMADPDiscrete::GetReferred(). */
    TOIDecPOMDPDiscrete* GetReferred() const
    {return _m_TOIDecPOMDPDiscrete;}

    /**\brief returns a vector of individual (local) state indices
     * corresponding to joint state index jointSI.*/
    const std::vector<Index>& JointToIndividualStateIndices(Index jointSI) const
        { return(_m_TOIDecPOMDPDiscrete->JointToIndividualStateIndices(jointSI)); }

    /**\brief returns the joint index for indivStateIndices*/
    Index IndividualToJointStateIndices(const std::vector<Index>&
                                        indivStateIndices) const
        { return(_m_TOIDecPOMDPDiscrete->IndividualToJointStateIndices(indivStateIndices)); }

    std::vector<Index> SampleSuccessorState(const std::vector<Index> &sIs,
                                             const std::vector<Index> &aIs)
        const
        { return(_m_TOIDecPOMDPDiscrete->SampleSuccessorState(sIs,aIs)); }
                                             
    std::vector<Index> SampleJointObservation(const std::vector<Index> &aIs,
                                               const std::vector<Index> &sucIs)
        const
        { return(_m_TOIDecPOMDPDiscrete->SampleJointObservation(aIs,sucIs)); }

    std::vector<Index> SampleInitialStates(void) const
        { return(_m_TOIDecPOMDPDiscrete->SampleInitialStates()); }

};


#endif /* !_PLANNINGUNITTOIDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
