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
#ifndef _BAYESIANGAMECOLLABORATIVEGRAPHICAL_H_
#define _BAYESIANGAMECOLLABORATIVEGRAPHICAL_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BayesianGameBase.h"
#include "BayesianGameIdenticalPayoffInterface.h"

#include "Scope.h"

class BayesianGameCollaborativeGraphical;
typedef boost::shared_ptr<BayesianGameCollaborativeGraphical> BGCG_sharedPtr;
typedef boost::shared_ptr<const BayesianGameCollaborativeGraphical> BGCG_constPtr;

class BayesianGameIdenticalPayoff;
class JointPolicyDiscretePure;
class PDDiscreteInterface;
/**\brief BayesianGameCollaborativeGraphical represents a
 * collaborative graphical Bayesian game.
 *
 * */
class BayesianGameCollaborativeGraphical 
    : 
        //public BayesianGameBase,
        public BayesianGameIdenticalPayoffInterface
{
    private:

        ///Private helper function to figure out whether the CGBG is fully connected.
        void RecurseOverAgents(Index currentAgent, std::vector<bool> &agentVisited) const;

    protected:

        size_t _m_nrLRFs;
        ///The components or LRFs of the CGBG
        std::vector<BayesianGameIdenticalPayoff*> _m_LRFs;
        std::vector<Scope> _m_agentScopes;
    
        /**a pointer to a (discrete) probablity distribution over types.
         * It should typically be compact!
         * */
        PDDiscreteInterface* _m_jt_pd;

    
    public:
        // Constructor, destructor and copy assignment.

        /// Constructor without arguments
        BayesianGameCollaborativeGraphical();

        /** Constructor - the constructor is called in the same way as a 
         * (regular) BayesianGameIdenticalPayoff, with the number of agents
         * and the number of actions, types per agent.
         *
         * The LRFs (components) of the payoff functions need to be added
         * manually with AddLRF below, and SetUtility(LRF, ...)*/
        BayesianGameCollaborativeGraphical(
                size_t nrAgents, 
                const std::vector<size_t>& nrActions,  
                const std::vector<size_t>& nrTypes);
    
        /// Copy constructor.
        BayesianGameCollaborativeGraphical(const BayesianGameCollaborativeGraphical& a);
        /// Destructor.
        virtual ~BayesianGameCollaborativeGraphical();
        /// Copy assignment operator
        BayesianGameCollaborativeGraphical& operator= (const BayesianGameCollaborativeGraphical& o);

        //operators:

        //data manipulation (set) functions:
        
        /**Add an LRF with scope s.*/
        void AddLRF(const Scope &s);

        /**Set and distributes probability: 
         *
         * 1) Set the probability of joint type jtI to p.
         * 2) For each LRF e, it *adds* p to the probability of 
         *    of the agSC_jtI ( the joint type of the agentScope of e ), 
         *    that is consistent with jtI.
         *      
         * E.g. assume we call DistributeProbability(<2,4>, .12)
         * if there is 2 LRFs (one with agent scope=<1>, and one with 
         * agentscope=<2>), 
         * then this adds .12 to P(LRF 1, <2>) and to P(LRF 2, <4>)
         */         
        void DistributeProbability(Index jtI, double p);

        virtual void SetProbability(Index jtI, double p)
        {
            throw(E("BayesianGameCollaborativeGraphical::SetProbability() for graphical games use DistributeProbability() instead of SetProbability()"));
        }

        void SetProbabilityDistribution(const PDDiscreteInterface* pd);
        //{ _m_jt_pd = pd->Clone(); } //<- will not allow forward declaration of PDDiscreteInterface

        /**For each LRF e this function
         * returns the probability of the jtI_agSC{e} consistent with
         * jtI.
         */
        std::vector<double> GetRestrictedJointTypeProbabilities(Index jtI) const;

        
        //get (data) functions:
        //
        /**Return the number of agents.*/
        //size_t GetNrAgents() const
        //{return _m_nrAgents;}
        /**Return the number of agents.*/
        virtual size_t GetNrLRFs() const
        {return _m_nrLRFs;}
        /**Return the scope of LRF e*/
        virtual Scope GetScope(Index e) const
        {return _m_agentScopes[e];}
        /**Return the number of joint types for LRF e.*/
        virtual size_t GetNrJointTypesForLRF(Index e);
        /**Return the number of joint actions for LRF e.
         * I.e. the number of joint group actions that are relevent for LRF
         * e. (I.e., the number of joint actions spawned by the agent scope of
         * LRF e);
         */
        virtual size_t GetNrJointActionsForLRF(Index e);

        /**Restrict the vector indivIndices to the agents in the scope of
         * LRF e (and returns that result)*/
        
        template <typename T>
        std::vector<T> RestrictIndividualIndicesToScope(
                const std::vector<T> &indivIndices, Index LRF) const;
        //virtual std::vector<Index> RestrictIndividualIndicesToScope(
        //        const std::vector<Index> &indivIndices, Index LRF) const;

        /**Returns the group type index jtGI corresponding to joint type index
         * jtI for LRF e.*/
        virtual Index JointToGroupTypeIndex(Index e, Index jtI) const;
        /**Returns the group type index jaGI corresponding to joint action index
         * jaI for LRF e.*/
        virtual Index JointToGroupActionIndex(Index e, Index jtI) const;

        virtual double GetProbability(Index e, 
                const std::vector< Index >& indTypes) const;
        virtual double GetProbability(Index e, Index jtI_e ) const;
        virtual void SetProbability(Index e, const std::vector< Index >& indTypes, double p);
        virtual void SetProbability(Index e, Index jtI_e, double p);
        
        virtual double GetUtility(Index e, Index jtI_e, Index jaI_e) const;
        virtual double GetUtility(Index e, 
                const std::vector< Index >& indTypes_e, 
                const std::vector< Index >& actions_e) const;
        virtual void SetUtility(Index e, Index jtI_e, Index jaI_e, double ut);
        virtual void SetUtility(Index e, 
            const std::vector< Index >& indTypes_e, 
            const std::vector< Index >& actions_e,
            double ut);

        virtual const BayesianGameIdenticalPayoff* GetBGIPforLRF(Index e) const
        { return _m_LRFs.at(e); }
        
        /**\brief evaluates the value of a joint policy.*/
        virtual double ComputeValueJPol(const JointPolicyDiscretePure & jpolBG) const;

//implement the BayesianGameIdenticalPayoffInterface:
        double GetProbability(Index jtype) const;
        

        virtual double GetUtility(Index jtype, Index ja) const;
        /**Gets the utility for (for all agents) joint type corresponding to 
         * the individual type indices (indTypeIndices) and joint action
         * corresponding to individual action indices (indActionIndices).*/
        double GetUtility(const std::vector<Index>& indTypeIndices, 
                          const std::vector<Index>& indActionIndices ) const;
        
        /** Prints a description of this  entire BayesianGameIdenticalPayoff 
         * to a string.*/
        std::string SoftPrint() const; 

        void SanityCheckBGCG() const;
    
        bool IsFullyConnected() const;


        virtual void ExportAsMAID() const
        { throw E("nyi BayesianGameCollaborativeGraphical::ExportAsMAID()");};

};

template<typename T> 
typename std::vector<T> 
BayesianGameCollaborativeGraphical::RestrictIndividualIndicesToScope(const std::vector<T> &indivIndices, Index LRF) const
{
    Index e = LRF;
    Scope agSc = _m_agentScopes[e];
    std::vector<T> restr(agSc.size());
    IndexTools::RestrictIndividualIndicesToScope(indivIndices, agSc, restr);
    return(restr);
}

#endif /* !_BAYESIANGAMECOLLABORATIVEGRAPHICAL_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
