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
#ifndef _PLANNINGUNITMADPDISCRETEPARAMETERS_H_
#define _PLANNINGUNITMADPDISCRETEPARAMETERS_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

/** \brief PlanningUnitMADPDiscreteParameters stores parameters of
 * PlanningUnitMADPDiscrete. 
 *
 * It controls which types of histories are generated when
 * initializing the PlanningUnitMADPDiscrete. What to generate depends
 * on the particular planning algorithm derived from it, for instance,
 * some algorithms use joint beliefs, others don't (in which case
 * there is no point in generating and storing them). This class also
 * indicates whether joint beliefs should represented sparsely or not.
 **/
class PlanningUnitMADPDiscreteParameters 
{
private:    

    /// Generate individual observation histories or not.
    bool _m_individualObservationHistories;
    /// Generate individual action histories or not.
    bool _m_individualActionHistories;
    /// Generate individual action-observation histories or not.
    bool _m_individualActionObservationHistories;
    /// Generate joint observation histories or not.
    bool _m_jointObservationHistories;
    /// Generate joint action histories or not.
    bool _m_jointActionHistories;
    /** \brief Whether or not joint action observation histories are
     * generated and stored.
     * 
     * If they are stored, also their (conditional) probabilities are
     * cached.*/
    bool _m_jointActionObservationHistories;
    /** \brief Whether joint beliefs are cached.
     *
     * Only applicable when jointActionObservationHistories are
     * generated.).*/
    bool _m_JointBeliefs;
    /// Use sparse beliefs or the full representation.
    bool _m_useSparseBeliefs;
    /**\brief Indicate whether the observation model
     * is defined over (s',a,s) (an event-driven model)
     * or the standard (s',a)*/
    bool _m_eventObservability;

protected:
    
public:
    /// Ensures no illegal combination of parameters has been set.
    /**Sanity check is called by the constructor of PlanningUnitMADPDiscrete.
     * 
     * The old procedure "Is called after each call to a Set function."
     * made no sense, as it was not possible to do 
     *      params.SetComputeJointActionHistories(false);
     *      params.SetComputeJointBeliefs(false);
     * as it started complaining at the first call...
     * */
    void SanityCheck() const;


    // Constructor, destructor and copy assignment.
    /// Default constructor.
    /** By default, all histories are generated, joint beliefs are
     * cached and not represented sparsely. */
    PlanningUnitMADPDiscreteParameters();
    /// Destructor.
    ~PlanningUnitMADPDiscreteParameters();

    /// Switch on or off the generation of all joint histories.
    void SetComputeAllJointHistories(bool val);
    /// Switch on or off the generation of all individual histories.
    void SetComputeAllIndividualHistories(bool val);
    /// Switch on or off the generation of all histories.
    void SetComputeAll(bool val);

    /// Switch on or off the generation of individual observation histories.
    void SetComputeIndividualObservationHistories(bool val){
        _m_individualObservationHistories = val; 
    }
    
    /// Switch on or off the generation of individual action histories.
    void SetComputeIndividualActionHistories(bool val){
        _m_individualActionHistories = val; 
    }
    
    /// Switch on or off the generation of individual action-observation histories.
    void SetComputeIndividualActionObservationHistories(bool val){
        _m_individualActionObservationHistories = val; 
    }
    
    /// Switch on or off the generation of joint observation histories.
    void SetComputeJointObservationHistories(bool val){
        _m_jointObservationHistories = val; 
    }
    
    /// Switch on or off the generation of joint action histories.
    void SetComputeJointActionHistories(bool val){
        _m_jointActionHistories = val; 
    }
    
    /// Switch on or off the generation of joint action-observation histories.
    void SetComputeJointActionObservationHistories(bool val){
        _m_jointActionObservationHistories = val; 
    }
    
    /// Switch on or off storing the joint beliefs.
    void SetComputeJointBeliefs(bool val){
        _m_JointBeliefs = val; 
    }

    /// Switch on or off whether joint beliefs should be represented sparsely.
    void SetUseSparseJointBeliefs(bool val){
        _m_useSparseBeliefs = val; 
    }

    /// Switch on or off the use of PS-dependent observation models.
    void SetEventObservability(bool val){
        _m_eventObservability = val; 
    }

    /// Are individual observation histories generated or not.
    bool GetComputeIndividualObservationHistories() const{
        return(_m_individualObservationHistories);
    }

    /// Are individual action histories generated or not.
    bool GetComputeIndividualActionHistories() const{
        return(_m_individualActionHistories);
    }

    /// Are individual action-observation histories generated or not.
    bool GetComputeIndividualActionObservationHistories() const{
        return(_m_individualActionObservationHistories);
    }

    /// Are joint observation histories generated or not.
    bool GetComputeJointObservationHistories() const{
        return(_m_jointObservationHistories);
    }

    /// Are joint action histories generated or not.
    bool GetComputeJointActionHistories() const{
        return(_m_jointActionHistories);
    }

    /// Are joint action-observation histories generated or not.
    bool GetComputeJointActionObservationHistories() const{
        return(_m_jointActionObservationHistories);
    }

    /// Are the joint beliefs cached or not.
    bool GetComputeJointBeliefs() const{
        return(_m_JointBeliefs);
    }

    /// Are sparse beliefs used or not.
    bool GetUseSparseJointBeliefs() const{
        return(_m_useSparseBeliefs);
    }

    /// Observable states or observable transitions.
    bool GetEventObservability() const{
        return(_m_eventObservability);
    }

    /// Print out the parameters to cout.
    void Print() const;

};


#endif /* !_PLANNINGUNITMADPDISCRETEPARAMETERS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
