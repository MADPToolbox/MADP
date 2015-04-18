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
#ifndef _PLANNINGUNITDECPOMDPDISCRETE_H_
#define _PLANNINGUNITDECPOMDPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Referrer.h"
#include "PlanningUnitMADPDiscrete.h"
#include "DecPOMDPDiscreteInterface.h"
#include "boost/shared_ptr.hpp"

class JointPolicyPureVector;

/** \brief PlanningUnitDecPOMDPDiscrete represents a planning unit for
 * discrete Dec-POMDPs. */
class PlanningUnitDecPOMDPDiscrete : 
    //public Referrer<DecPOMDPDiscreteInterface>,
    public PlanningUnitMADPDiscrete 
{
    private:    
        DecPOMDPDiscreteInterface* _m_DecPOMDP;

    protected:
    
       /// Runs some consistency tests.
       bool SanityCheck() const;
    
    public:
        // Constructor, destructor and copy assignment.
        ///the (default) Constructor. 
        PlanningUnitDecPOMDPDiscrete(
                const PlanningUnitMADPDiscreteParameters &params,
                size_t horizon=3,
                DecPOMDPDiscreteInterface* p=0
            );
        PlanningUnitDecPOMDPDiscrete(
                size_t horizon=3,
                DecPOMDPDiscreteInterface* p=0
            );

        //operators:

        //data manipulation (set) functions:

        /// Tell which SetReferred to use by default.
        //void SetReferred(DecPOMDPDiscreteInterface* p) 
        //{Referrer<DecPOMDPDiscreteInterface>::SetReferred(p);}
        ///Set the problem (DecPOMDPDiscreteInterface) using a pointer.
        void SetProblem(DecPOMDPDiscreteInterface* p);

        //get (data) functions:

        /// Returns the DecPOMDPDiscreteInterface pointer.
        /** Tell which GetReferred to use by default... (nl. the
         * Referrer<DecPOMDPDiscreteInterface>::GetReferred(), not the
         * PlanningUnitMADPDiscrete::GetReferred(). *
        DecPOMDPDiscreteInterface* GetReferred() const
        {
            //std::cerr << "Warning PlanningUnitDecPOMDPDiscrete::GetReferred is deprecated, use GetDPOMDP instead"<<std::endl;
            return(_m_DecPOMDP);
        }*/
        
        DecPOMDPDiscreteInterface* GetDPOMDPD() const
        {return( _m_DecPOMDP );}

        /// Return the reward for state, joint action indices 
        double GetReward(Index sI, Index jaI) const
            { return(GetDPOMDPD()->GetReward(sI, jaI)); }
        
        ///Returns the discount parameter.
        double GetDiscount() const 
            {return GetDPOMDPD()->GetDiscount();}

        /// Returns the expected reward of the best found joint policy.
        virtual double GetExpectedReward() const = 0;

        /* Frans 20110908 - not sure if we need this here? 
         * Well removing it, also give all kinds of problems... */
        /// Returns the found joint policy.
        virtual boost::shared_ptr<JointPolicyPureVector> GetJointPolicyPureVector()
        {throw E("PlanningUnitDecPOMDPDiscrete::GetJointPolicyPureVector - Error this function should be overriden by the derived PU! Check if the used planning unit actually uses JointPolicyPureVector !");}
        
        /// Returns the found joint policy.
        virtual boost::shared_ptr<JointPolicy> GetJointPolicy()
        {throw E("PlanningUnitDecPOMDPDiscrete::GetJointPolicy - Error this function should be overriden by the derived PU!");}


    /// Exports the Dec-POMDP represented by \a pu to file named \a filename.
    static void ExportDecPOMDPFile(const std::string & filename,
                                   const DecPOMDPDiscreteInterface *decpomdp);

    /// Exports the Dec-POMDP to file named \a filename.
    void ExportDecPOMDPFile(const std::string & filename) const;

};


#endif /* !_PLANNINGUNITDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
