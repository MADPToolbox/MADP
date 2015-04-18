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
#ifndef _BAYESIANGAMEBASE_H_
#define _BAYESIANGAMEBASE_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include <map>
#include "Globals.h"
#include "Interface_ProblemToPolicyDiscretePure.h"
#include "IndexTools.h"
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/vector_sparse.hpp"

/**\brief BayesianGameBase is a class that represents a Bayesian game.
 *
 * This is a self
 * contained class - meaning that it does not depend on any Multi-agent 
 * decision problem or Planning unit.
 * This implies that, in order to convert a time-step of a MADP Planning
 * Unit to a Bayesian game, indices of observation(-action) histories have
 * to be converted. This class uses its own indices.
 */
class BayesianGameBase :
    public Interface_ProblemToPolicyDiscretePure //this class implements the interface
        //used by PolicyPureVector
{
    private:

        bool _m_useSparse;
        ///the probability distribution over joint types. A mapping from 
        ///joint indices to probabilities
        std::vector<double> _m_jTypeProbs;
        typedef boost::numeric::ublas::mapped_vector<double> SparseVector;
        SparseVector _m_jTypeProbsSparse;

        ///An joint type -> indiv. type indices cache:
        std::vector< std::vector<Index> > *_m_jointToIndTypes;
        ///An joint type -> indiv. type indices cache:
        std::map<Index, std::vector<Index> > *_m_jointToIndTypesMap;

    protected:
        /// private bool to indicate whether this BG is initialized.
        bool _m_initialized;

        /// _m_verboseness >0 verbose, <0 is quiet
        int _m_verboseness;

        ///the number of players (or agents)
        size_t _m_nrAgents;
        ///the number of actions for each agent
        std::vector<size_t> _m_nrActions;
        ///the number of types for each agent
        std::vector<size_t> _m_nrTypes;
        ///the number of joint actions:
        size_t _m_nrJTypes;
        ///the number of joint types:
        size_t _m_nrJA;
        
        ///Boolean that indicates whether the number of joint actions can be represented by size_t (or that overflowing occurs)
        bool _m_JAoverflow;
        ///Boolean that indicates whether the number of joint types can be represented by size_t (or that overflowing occurs)
        bool _m_JToverflow;        
        ///an size_t array that caches the stepsize array for actions:
        size_t * _m_stepSizeActions;
        ///an size_t array that caches the stepsize array for types:
        size_t * _m_stepSizeTypes;

        void ChangeNrActions(Index agI, size_t new_nr);
        void ChangeNrTypes(Index agI, size_t new_nr);
        void Initialize();
    public:
        // Constructor, destructor and copy assignment.
        // (default) Constructor
        BayesianGameBase();
        BayesianGameBase(size_t nrAgents, 
                const std::vector<size_t> & nrActions,  
                const std::vector<size_t> & nrTypes,
                int verboseness=0);
        /// Copy constructor.
        BayesianGameBase(const BayesianGameBase& a);

        /// Destructor.
        ~BayesianGameBase();

        BayesianGameBase& operator= (const BayesianGameBase& o);

        //operators:

        
        //data manipulation (set) functions:
        
        /**\brief Sets the initialized status to b. When setting to true - checks are
         * performed to see if this is a consistent Bayesian Game.*/
        bool SetInitialized(bool b);
        /**\brief Sets the probability of joint type i to p.*/
        void SetProbability(Index i, double p)
        { if(_m_useSparse) _m_jTypeProbsSparse[i]=p;
            else _m_jTypeProbs[i]=p;}
        /**\brief Sets the probability of joint type corresponding to the individual
         * type indices (indIndices) to p.*/
        void SetProbability(const std::vector<Index>& indIndices, double p)
        {SetProbability(IndividualToJointTypeIndices(indIndices), p);}
        /**\brief Adds p to the probability of joint type i*/
        void AddProbability(Index i, double p)
            {if(_m_useSparse) _m_jTypeProbsSparse[i]+=p;
                else _m_jTypeProbs[i]+=p;}
        /**\brief Adds p to the probability of joint type corresponding to the 
         * individual type indices (indIndices).*/
        void AddProbability(const std::vector<Index>& indIndices, double p)
        {AddProbability(IndividualToJointTypeIndices(indIndices), p);}


        //get (data) functions:
        size_t GetNrJointActions() const;
        size_t GetNrJointTypes() const;
        const std::vector<size_t>& GetNrTypes() const {return _m_nrTypes;}
        size_t GetNrTypes(Index agI) const {return _m_nrTypes.at(agI);}
        
        /**\brief Gets the probability of joint type i*/
        virtual double GetProbability(Index i) const;
//        {if (_m_useSparse) return(_m_jTypeProbsSparse[i]);
//            else  return(_m_jTypeProbs[i]);}
        /**\brief Gets the probability of joint type corresponding to the individual
         * type indices (indIndices)*/
        virtual double GetProbability(const std::vector<Index>& indIndices) const
        {return(GetProbability(IndividualToJointTypeIndices(indIndices)));}


        //indices

        Index IndividualToJointActionIndices(const Index* IndArr)const
        { return IndexTools::IndividualToJointIndicesArrayStepSize(IndArr, 
                _m_stepSizeActions, _m_nrAgents);};
        Index IndividualToJointActionIndices(const std::vector<Index>& indices)
            const
        { return IndexTools::IndividualToJointIndicesStepSize(indices, 
                _m_stepSizeActions);};

        Index IndividualToJointTypeIndices(const std::vector<Index>& indices) const
        { return IndexTools::IndividualToJointIndicesStepSize(indices, 
                _m_stepSizeTypes);};
        std::vector<Index> JointToIndividualActionIndices(Index jaI) const
        { return IndexTools::JointToIndividualIndicesStepSize(jaI, 
                _m_stepSizeActions, _m_nrAgents);};
/*      std::vector<Index> JointToIndividualTypeIndices(Index jTypeI) const
        { return IndexTools::JointToIndividualIndicesStepSize(jTypeI, 
                _m_stepSizeTypes, _m_nrAgents);};
*/
        const std::vector<Index>& JointToIndividualTypeIndices(Index jTypeI) const
        {
            if(_m_useSparse)
            {
                if((*_m_jointToIndTypesMap).find(jTypeI)==
                   (*_m_jointToIndTypesMap).end())
                    (*_m_jointToIndTypesMap)[jTypeI]=
                        IndexTools::JointToIndividualIndices(
                            jTypeI, _m_nrTypes);
                return((*_m_jointToIndTypesMap)[jTypeI]);
                    
            }
            else
            {
                if((*_m_jointToIndTypes)[jTypeI].size()==0)
                    (*_m_jointToIndTypes)[jTypeI]=
                        IndexTools::JointToIndividualIndices(
                            jTypeI, _m_nrTypes);
                return((*_m_jointToIndTypes)[jTypeI]);
            }
        }



        /** Prints a description of this BayesianGameBase to a string.*/
        std::string SoftPrint() const;
        /**\brief Print this BayesianGameBase to cout.*/
        void Print() const
        {std::cout << SoftPrint();}

        /** Sanity check should be overriden by classes that do not use
         *  the implementation provided by this class (e.g. CGBGs)
         */
        virtual void SanityCheck()
        {SanityCheckBGBase();};
        void SanityCheckBGBase();

        /// implement the Interface_ProblemToPolicyDiscrete interface:
        size_t GetNrAgents() const 
            {return _m_nrAgents;}
        size_t GetNrActions(Index agentI) const 
            {return _m_nrActions[agentI];}
        const std::vector<size_t>& GetNrActions() const 
            {return _m_nrActions;}
        size_t GetNrPolicyDomainElements(
            Index agentI, 
            PolicyGlobals::PolicyDomainCategory cat,
            size_t depth=MAXHORIZON) const;
        LIndex GetNrPolicies(Index ag) const
        { return (Interface_ProblemToPolicyDiscretePure::GetNrPolicies(ag,
                PolicyGlobals::TYPE_INDEX) ); }
        LIndex GetNrJointPolicies() const
        { return (Interface_ProblemToPolicyDiscretePure::GetNrJointPolicies(
                PolicyGlobals::TYPE_INDEX) ); }
        //Index GetJointActionIndex(std::vector<Index>& indivIndices) const
        //    {return IndividualToJointActionIndices(indivIndices);}

        /**\brief implementation of JointToIndividualPolicyDomainIndices
         *
         * (specified in the Interface_ProblemToPolicyDiscrete )
         */
        std::vector<Index> JointToIndividualPolicyDomainIndices
            (Index jdI, PolicyGlobals::PolicyDomainCategory cat) const
        {
            if(cat != PolicyGlobals::TYPE_INDEX)
                throw(E("BG only supports type as the domain of the policy"));
            return(JointToIndividualTypeIndices(jdI));
        }        
        /**\brief implementation of JointToIndividualPolicyDomainIndicesRef
         *
         * (specified in the Interface_ProblemToPolicyDiscrete )
         */
        const std::vector<Index>& JointToIndividualPolicyDomainIndicesRef
            (Index jdI, PolicyGlobals::PolicyDomainCategory cat) const
        {
#if 0 // disable check for speed
            if(cat != PolicyGlobals::TYPE_INDEX)
                throw(E("BG only supports type as the domain of the policy"));
#endif
//            return( _m_jointToIndTypes[jdI]);
            return(JointToIndividualTypeIndices(jdI));
        }

        // A BG has types, not (A)OH histories. 
        // (and the joint -> individual type conversions are cached)
        bool CacheJointToIndivType_Indices() const {return true;};        
        bool CacheJointToIndivOH_Indices() const {return false;};
        bool CacheJointToIndivAOH_Indices() const {return false;};
        virtual bool AreCachedJointToIndivIndices(
            const PolicyGlobals::PolicyDomainCategory pdc) const;

        std::string SoftPrintType(Index agentI, Index typeIndex ) const;

        //The following functions are needed to implement the Interface_ProblemToPolicyDiscrete 
        std::string SoftPrintPolicyDomainElement(Index agentI, Index typeIndex,
               PolicyGlobals::PolicyDomainCategory cat ) const;
        std::string SoftPrintAction(Index agentI, Index actionI) const;

        /*\brief the default PolicyDomainCategory for the planning unit.
         *
         * as specified by Interface_ProblemToPolicyDiscrete. This can be 
         * overriden in derived classes.
         */
        virtual PolicyGlobals::PolicyDomainCategory GetDefaultIndexDomCat()
            const;


};

#endif /* !_BAYESIANGAMEBASE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
