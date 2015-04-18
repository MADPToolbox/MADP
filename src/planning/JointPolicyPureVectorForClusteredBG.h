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
#ifndef _JOINTPOLICYPUREVECTORFORCLUSTEREDBG_H_
#define _JOINTPOLICYPUREVECTORFORCLUSTEREDBG_H_ 1

/* the include directives */
#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "PartialJointPolicyPureVector.h"
#include "boost/shared_ptr.hpp"

class JointPolicyPureVectorForClusteredBG;
typedef boost::shared_ptr<JointPolicyPureVectorForClusteredBG> JPPVfCBG_sharedPtr;
typedef boost::shared_ptr<const JointPolicyPureVectorForClusteredBG> JPPVfCBG_constPtr;

class BayesianGameIdenticalPayoffInterface;
class BayesianGameWithClusterInfo;
class PlanningUnitDecPOMDPDiscrete;
class TypeCluster;

/** \brief JointPolicyPureVectorForClusteredBG represents a joint
 * policy for a clustered CBG. */
class JointPolicyPureVectorForClusteredBG : 
    public PartialJointPolicyPureVector
{
    private:    
        boost::shared_ptr<const BayesianGameWithClusterInfo> _m_bg;
//        const BayesianGameWithClusterInfo* _m_bg;
        ///Stores a pointer to the previous policy for a clustered BG
        /**IFF the previous policy also is a JointPolicyPureVectorForClusteredBG
         * if not (the previous policy was a regular past policy, e.g., a
         * JointPolicyPureVector), then this variable is 0
         */
        JPPVfCBG_constPtr _m_prevJPolBG;
        

        //some aux. functions to convert to a JPPV
        void StartRecursiveConstructionPerAgent( 
            const PlanningUnitDecPOMDPDiscrete* pu,
            JPPV_sharedPtr jpolJPPV,
            Index ts,
            const std::vector<const JointPolicyDiscretePure*>& jpolBGVec,
            const std::vector<boost::shared_ptr<const BayesianGameWithClusterInfo> >& bgVec
        )const ;

        void RecursivelyFillPolicyForAgent(
            const PlanningUnitDecPOMDPDiscrete* pu,
            JPPV_sharedPtr jpolJPPV,
            Index agI,
            Index ts,
            Index aohI,
            Index ohI,
            const TypeCluster* tc,
            Index tI,
            Index aI,
            const std::vector<const JointPolicyDiscretePure*>& jpolBGVec,
            const std::vector<boost::shared_ptr<const BayesianGameWithClusterInfo> >& bgVec
        )const ;


        void StartRecursiveSoftPrintPerAgent( 
            const PlanningUnitDecPOMDPDiscrete* pu
            , std::stringstream& ss
            , Index ts
            , Index last_ts
            , const std::vector<const JointPolicyDiscretePure *>& jpolBGVec
            , std::vector<boost::shared_ptr<const BayesianGameWithClusterInfo> > bgVec
        )const;
        void RecursivelyPrintPolicyForAgent(
            const PlanningUnitDecPOMDPDiscrete* pu
            , std::stringstream& ss
            , Index agI
            , Index ts
            , Index last_ts
            , Index aohI    //the previous aohI
            , Index ohI    //the previous ohI
            , const TypeCluster* tc //the previous typecluster that represents aohI
            , Index tI //the index of tc (in the previous bg)
            , Index aI //the action taken for tI (i.e. for aohI)
            , const std::vector<const JointPolicyDiscretePure *>& jpolBGVec
            , std::vector<boost::shared_ptr<const BayesianGameWithClusterInfo> > bgVec
        )const;


    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.

        /// (default) Constructor
        JointPolicyPureVectorForClusteredBG(
                const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &pu,
                PolicyGlobals::PolicyDomainCategory idc=TYPE_INDEX,
                JPPVfCBG_constPtr prevJPolBG =
                JPPVfCBG_constPtr(),
                double pastReward = 0.0);

        /// Copy constructor.
        JointPolicyPureVectorForClusteredBG(
                const JointPolicyPureVectorForClusteredBG& a);

        /// Destructor.
        virtual ~JointPolicyPureVectorForClusteredBG();

        /// Copy assignment operator
        virtual JointPolicyPureVectorForClusteredBG& operator= 
            (const JointPolicyPureVectorForClusteredBG& o);
        virtual JointPolicyPureVectorForClusteredBG& operator= 
            (const PartialJointPolicyPureVector& o);
        virtual JointPolicyPureVectorForClusteredBG& operator= 
            (const PartialJointPolicyDiscretePure& o);
        virtual JointPolicyPureVectorForClusteredBG& operator= 
            (const JointPolicyDiscretePure& o);
        virtual JointPolicyPureVectorForClusteredBG& operator= 
            (const JointPolicyDiscrete& o);
        virtual JointPolicyPureVectorForClusteredBG& operator= 
            (const JointPolicy& o);

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:

        boost::shared_ptr<const BayesianGameWithClusterInfo> GetBG() const
        {return _m_bg;}

        JPPVfCBG_constPtr GetPrevJPPVfCBG() const
        {return _m_prevJPolBG;}
        
        void SetPrevJPPVfCBG(JPPVfCBG_constPtr prevJpol)
        { _m_prevJPolBG=prevJpol;}
        
        /**\brief Convert this joint policy to a JointPolicyPureVector.
         */
        JPPV_sharedPtr ToJointPolicyPureVector() const;

        //other functions
        
        /// Prints a description of this JointPolicyPureVector to a string.
        std::string SoftPrint() const; 

        /// Prints a brief description to a string.
        std::string SoftPrintBrief() const;

        virtual JointPolicyPureVectorForClusteredBG* Clone() const;

};


#endif /* !_JOINTPOLICYPUREVECTORFORCLUSTEREDBG_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
