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
#ifndef _VALUEFUNCTIONDECPOMDPDISCRETE_H_
#define _VALUEFUNCTIONDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <map>
#include "Globals.h"
#include "ValueFunction.h"
#include "Referrer.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointPolicyDiscretePure.h"
#include "boost/numeric/ublas/matrix.hpp"

/** \brief ValueFunctionDecPOMDPDiscrete represents and calculates the
 * value function of a (pure) joint policy for a discrete Dec-POMDP.
 */
class ValueFunctionDecPOMDPDiscrete : 
    public ValueFunction,
    public Referrer<PlanningUnitDecPOMDPDiscrete>,
    public Referrer<JointPolicyDiscretePure>
{
    private:
    PlanningUnitDecPOMDPDiscrete* _m_pu;
    JointPolicyDiscretePure* _m_jpol;

    //some number we cache for speed:
    size_t _m_nrJOH;
    size_t _m_nrJO;
    size_t _m_nrS;
    size_t _m_h;
    bool _m_V_initialized;
    typedef boost::numeric::ublas::matrix<double> Matrix;

    Matrix* _m_p_V; //stores V(sI, JOHistI)
    std::map<Index, bool> _m_cached; // (s,JOHist)-index -> bool

    Index GetCombinedIndex(Index sI, Index JOHI) const
    //{  return( sI*GetPU()->GetNrJointObservationHistories() + JOHI ); }
    {  return( sI*_m_nrJOH + JOHI ); }
    bool IsCached(Index sI, Index JOHI)
    {    return(_m_cached[GetCombinedIndex(sI, JOHI)]); }
    void SetCached(Index sI, Index JOHI);

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

    /**Function used by CalculateV0Recursively. This recursively 
     * Calculates the expected payoff of the joint policy starting from 
     * state sI and joint observation history johI.*/
    double CalculateVsjohRecursivelyNotCached(Index sI, Index johI,Index stage);
    /**Function used by CalculateV0Recursively. This recursively 
     * Calculates the expected payoff of the joint policy starting from 
     * state sI and joint observation history johI.*/
    double CalculateVsjohRecursivelyCached(Index sI, Index johI, Index stage);
    double CalculateV0RecursivelyCached();
    double CalculateV0RecursivelyNotCached();

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
    double CalculateV(bool cache = true);    

};


#endif /* !_VALUEFUNCTIONDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
