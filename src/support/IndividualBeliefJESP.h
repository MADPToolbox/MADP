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
#ifndef _INDIVIDUALBELIEFJESP_H_
#define _INDIVIDUALBELIEFJESP_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include "Globals.h"
#include "IndexTools.h"
#include "Belief.h"

class JointPolicyPureVector;
class PlanningUnitMADPDiscrete; //forward declaration to avoid including each other

/**
 * \brief IndividualBeliefJESP stores individual beliefs for the JESP
 * algorithm.
 */
class IndividualBeliefJESP :
        public Belief
{
private:  
    ///pointer to PlanningUnitMADPDiscrete 
    /**(might be 0 when this belief is not associated with a PUMADP)
     */
    const PlanningUnitMADPDiscrete* _m_pumadp;
    ///The time step this belief belongs to.
    Index _m_stage;
    ///the number of agents    
    size_t _m_nrAgents;
    ///The agent for which this belief is
    Index _m_agentI;
    ///The Scope of the others
    Scope _m_others;
    ///The number of observation histories of agents != _m_agentI
    std::vector<size_t> _m_nrOH_others;
    ///a vector describing the size of the belief: _m_sizeVec=<nrS,nrJOHothers>
    std::vector<size_t> _m_sizeVec;
    ///the step-size cache for eIndex <-> <S, joHistJ>  conversion
    size_t * _m_stepsizeSJOH;
    ///the step-size cache for joHistJ <-> <oHist1,...oHistnrA> conversion
    size_t * _m_stepsizeJOHOH;
    
protected:
    
public:        
        
    /**This constructor usess it arguments to set additional information
     */
    IndividualBeliefJESP(Index agentI, Index stage,
        const PlanningUnitMADPDiscrete& pu);
    // Constructor which copies \a belief in this joint belief.
    // IndividualBeliefJESP(const IndividualBeliefJESPInterface &belief);

    /// Destructor
    ~IndividualBeliefJESP();

    IndividualBeliefJESP& operator= (const IndividualBeliefJESP& o);

    // Constructor, destructor and copy assignment.

    ///The individual belief update
    /**This gets only an \em individual action \a lastAI and \em individual
     * observation \a newOI, as its arguments.
     */
    double Update(const IndividualBeliefJESP& b_prev, Index lastAI, 
            Index newOI, const JointPolicyPureVector* jpol);

    ///Get the state corresponding to augmented state index eI
    Index GetStateIndex(Index eI) const;
    ///Get the vector with others' observation history indices cor.to eI
    std::vector<Index> GetOthersObservationHistIndex(Index eI) const;
    //
    Index GetAugmentedStateIndex(Index sI, 
            const std::vector<Index>& oHist_others) const;

    std::string SoftPrint() const;
    void Print() const
    { std::cout << SoftPrint();};

};

inline
Index
IndividualBeliefJESP::GetStateIndex(Index eI) const
{
    std::vector<Index> v = 
        IndexTools::JointToIndividualIndicesStepSize(eI, _m_stepsizeSJOH, 2);
    return v.at(0);
}



#endif /* !_INDIVIDUALBELIEFJESP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
