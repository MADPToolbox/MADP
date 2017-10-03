/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
#ifndef _VALUEFUNCTIONDECPOMDPDISCRETE_H_
#define _VALUEFUNCTIONDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <set>
#include "Globals.h"
#include "ValueFunction.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointPolicyDiscretePure.h"
#include <boost/unordered_map.hpp>

/** \brief ValueFunctionDecPOMDPDiscrete represents and calculates the
 * value function of a (pure) joint policy for a discrete Dec-POMDP.
 */
class ValueFunctionDecPOMDPDiscrete : 
    public ValueFunction
{
    private:
    PlanningUnitDecPOMDPDiscrete* _m_pu;
    JointPolicyDiscretePure* _m_jpol;

    //some number we cache for speed:
    size_t _m_nrJOH;
    size_t _m_nrJO;
    size_t _m_nrS;
    size_t _m_h;
    typedef boost::unordered_map<Index,double> CacheType;
    CacheType _m_cache;
    
    PlanningUnitDecPOMDPDiscrete* _m_planningUnitDecPOMDPDiscrete;
    JointPolicyDiscretePure* _m_jointPolicyDiscretePure;

    Index GetCombinedIndex(Index sI, Index JOHI) const
    //{  return( sI*GetPU()->GetNrJointObservationHistories() + JOHI ); }
    {  return( sI*_m_nrJOH + JOHI ); }

    // writes val to cache
    void SetCached(Index sI, Index JOHI, double val);
    // returns what is in the cache or undefined is valid==false in case the cache entry not set
    double GetCached(Index sI, Index JOHI,bool &valid); 
    
    /// Deletes the current value function.
    void DeleteV();
    /** \brief Creates a brand new value function (deleting the
     * current one if it exists).
     */
    void CreateV();

    PlanningUnitDecPOMDPDiscrete* GetPU() const 
        {  return _m_pu;
            //return( Referrer<PlanningUnitDecPOMDPDiscrete>::GetReferred() ); 
        }
    JointPolicyDiscretePure* GetJPol() const 
        {   return _m_jpol; 
            //return( Referrer<JointPolicyDiscretePure>::GetReferred() ); 
        }

    protected:
    
    public:
    // Constructor, destructor and copy assignment.
    /// Constructor
    ValueFunctionDecPOMDPDiscrete(PlanningUnitDecPOMDPDiscrete& p , 
                                  JointPolicyDiscretePure& jp);
    /// Constructor
    ValueFunctionDecPOMDPDiscrete(PlanningUnitDecPOMDPDiscrete* p , 
                                  JointPolicyDiscretePure* jp);
    /// Copy constructor.
    ValueFunctionDecPOMDPDiscrete(const ValueFunctionDecPOMDPDiscrete& a);
    /// Destructor.
    ~ValueFunctionDecPOMDPDiscrete();

    /// Calculates the value function, which is stored and returns V(0).
    /// call with "CalculateV<true>()" for the cached version or
    /// call with "CalculateV<false>()" for the non-cached version
    template <bool CACHED>
    double CalculateV();
    
private:

    /**Function used by CalculateV0Recursively. This recursively 
     * Calculates the expected payoff of the joint policy starting from 
     * state sI and joint observation history johI.*/
    template <bool CACHED>
    double CalculateVsjohRecursively(Index sI,Index johI, Index stage);
};


#endif /* !_VALUEFUNCTIONDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
