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
#ifndef _FSAOHDIST_NECOF_H_
#define _FSAOHDIST_NECOF_H_ 1

/* the include directives */
#include "Globals.h"
#include "FactoredStateAOHDistribution.h"
#include "FSDist_COF.h"
#include "CPT.h"

class PlanningUnitFactoredDecPOMDPDiscrete;
class CPDDiscreteInterface;
class MADPComponentFactoredStates;
/** \brief FSAOHDist_NECOF is a class that represents 
 * a NEarly COmpletely Factored distribution over state factors and
 * action-observation histories.
 *
 * This class assumes that the observations of different agents are 
 * independent given a,s'. i.e., that
 * \f[ \Pr(o|a,s') = \prod_i  \Pr(o_i|a,s') \f]
 **/
class FSAOHDist_NECOF : public FactoredStateAOHDistribution
{
    private:

        const PlanningUnitFactoredDecPOMDPDiscrete* _m_puf;

        ///The stage for which this distribution is.
        Index _m_stage;


        ///the maintained state factor marginals
        FSDist_COF *_m_sfacMarginals;
                
        ///the conditional observation history probabilities        
        std::vector< CPDDiscreteInterface* > _m_oHistConditional;

        ///marginals of observation history probs for each agent
        /**used for updating the belief and by the GetOH... functions
         * _m_oHistMarginals[agI][ohI] contains the probability of observation
         * history ohI for agent agI.
         */
        std::vector< std::vector<double> > _m_oHistMarginals;

        ///the state factors in the scope of influence for each agent('s obs.)
        std::vector< Scope > _m_sfacSoI;
        ///the number of influence instantiations for those scopes
        /**(i.e., the number of local states that influence o_i) */
        std::vector< size_t > _m_sfacSoI_ii_size;
        ///the stepsize array (used to convert indiv<->joint indicies) is cached
        /**I.e., this is used to quickly convert from/to individiual state 
         * indices of the state factors in the scope for this agent
         */
        std::vector< size_t *> _m_stepsize;

    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.

        /// Constructor without arguments, needed for serialization.
        FSAOHDist_NECOF();

        /// (default) Constructor
        FSAOHDist_NECOF(const PlanningUnitFactoredDecPOMDPDiscrete* p);
        
        /// Copy constructor.
        FSAOHDist_NECOF(const FSAOHDist_NECOF& a);
        /// Destructor.
        virtual ~FSAOHDist_NECOF();
        /// Copy assignment operator
        FSAOHDist_NECOF& operator= (const FSAOHDist_NECOF& o);
        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:        
        virtual double GetAOHProb_relT(
                const Scope& agSc, 
                const std::vector<Index>& aohI) 
            const
        { throw E("GetAOHProb_relT nyi..."); };
        virtual double GetOHProb_relT(
                const Scope& agSc, 
                const std::vector<Index>& ohI)
            const;

        ///returns P(sfacI, ohI)
        virtual double GetXOHProb_relT(
                const Scope& sfSc, const std::vector<Index>& sfacI,
                const Scope& agSc, const std::vector<Index>& ohI
                ) const;
        ///returns P(sfacI, ohI) like GetXOHProb_relT. 
        /**This version requires that sfSc is a superset of the scope of 
         * influence for all observations (of agents in agSc).
         * I.e., this function will not perform any marginalization.
         */
        virtual double GetXOHProb_relT_SufficientSFscope(
                const Scope& sfSc, const std::vector<Index>& sfacI,
                const Scope& agSc, const std::vector<Index>& ohI
                ) const;
        virtual double GetXProb(
                const Scope& sfSc, 
                const std::vector<Index>& sfValI)
            const;
        /**\brief Performs an update of the distribution to the next stage.
         * In particular this function computes the new 
         * 
         * \li state factor marginals
         * \li OH conditionals
         * \li OH marginals
         *
         * from the previous stage marginals.
         */
        virtual void Update(const JointPolicyDiscretePure& pol);

        virtual void InitializeFromISD( const FactoredStateDistribution * d);

        virtual std::string SoftPrint() const;
        
        virtual void SanityCheck();

        virtual FactoredStateAOHDistribution *Clone() const
        { return(new FSAOHDist_NECOF(*this)); }
};


#endif /* !_FSAOHDIST_NECOF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
