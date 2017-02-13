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
#ifndef _PROBLEM_CGBG_FF_H_
#define _PROBLEM_CGBG_FF_H_ 1

/* the include directives */
#include "Globals.h"
#include "BayesianGameCollaborativeGraphical.h"

/** \brief Problem_CGBG_FF reprents a generalized single-shot fire fighting problem.
 *
 *  -there are x houses scattered uniformly in the 2D plane (in the box bounded by [0,0], [1,1])
 *  -the fire level of each house is uniformly drawn from {0, ..., nrFireLevels-1}
 *  -each agent is assigned a random position in the same box (also uniform).
 *  -each agent can fight fire at the z nearest houses.
 *  -each agent gets an observation of the y nearest houses.
 *  -each house induces a cost, 
 *      -more agents at a house means a lower cost, however, the effect of adding agents is sub-additive.
 *      -a higher fire-level means a higher penalty 
 *      formula ?
 *
 *  -In order to prevent an arbitrary number of agents from participating in a single house,
 *   we remove house from being selected when #agents that participate in it reaches a threshold k.
 *   "this will in general not lead to an optimal assignment, other assignments methods may be used in the construction of the CGBG"
 *
 *  
 *   Note, that this implementation only supports y<z. I.e., it will not correctly deal
 *   with the case where the agents can observe houses where they do not fight fire at.
 *   The problem is that in that case, even though an agent cannot influence such a house
 *   via its action, it does influence the local utility u(jt,ja) via its type.
 *   (However, I guess that that will not influence the optimal joint policy?)
 *
 **/

//#define NROBSPERHOUSE 2

/*TODO 24-03-2011
 * fix:
 * ERROR: BayesianGameCollaborativeGraphical::GetProbability(Index jtype) - no pd!
 * by overriding this function in some way.
 * (it is called by regular BGIP solvers - i.e., AM)
 *
 * I guess we need to construct a pd when constructing the problem...?
 */

class Problem_CGBG_FF : public BayesianGameCollaborativeGraphical

{        
    protected:
        typedef std::pair<double, Index > diPair;
        struct diPairComp{
                bool operator() (const diPair & a, const diPair & b ) const
                {
                    //this operator should test for *less than*
                    //since we want shorter distances to be ranked higher, 
                    // we return "a.first > b.first". 
                    //I.e., when a.first > b.first then a's distance 
                    return (a.first > b.first); 
                }
            };
    public:

    typedef Index observation_t;
    private:    
        Scope _m_allAgents;
        size_t _m_nrHouses;
        size_t _m_nrFireLevels;
        size_t _m_nrObsPerHouse;
        size_t _m_nrActionsPerAgent;
        size_t _m_nrObservedHousesPerAgents;
        size_t _m_k; // the max number of agents in a scope
        size_t _m_maxNrAgentsObservingAHouse; //the equivalent for observations.

        typedef std::map< std::vector<observation_t>, double > VecDMap;
        typedef std::pair< std::vector<observation_t>, double > VecDPair;
        VecDMap _m_Norm_cache;
    
        //stores a Scope for each house that indicates which agents participate in it.
        std::vector<Scope> _m_agentsForHouse_action;
        //store for each house which agents can observe it.
        std::vector<Scope> _m_agentsForHouse_obs;

        /// _m_houseIndices_obs[agI]  contains a vector of house indices, indicating
        ///  the houses that agent agI can observe.
        std::vector< std::vector<Index> > _m_houseIndices_obs;
        /// similarly, the houses that the agent has as its actions (can go to):
        std::vector< std::vector<Index> > _m_houseIndices_action;

        // positions of houses and agents within the plane
        std::vector< double > _m_housePositionX;
        std::vector< double > _m_housePositionY;
        std::vector< double > _m_agentPositionX;
        std::vector< double > _m_agentPositionY;

        std::vector< Index > TypeIndexToObservationIndices(Index agI, Index typeI) const;
        void TypeIndexToObservationIndices(Index typeI, std::vector<Index> & obsIndices) const;
        double ComputeHouseProbability(
                Index hI,  
                const Scope& tupleOfAgents,
                const std::vector<Index>& types
                //Index houseI, const std::vector<observation_t>& oVec_hI
                ) const;
        double ComputeLocalProbability( 
            const Scope& tupleOfAgents,
            const std::vector<Index>& types
            ) const;
        
        double ComputeDistanceAgentToHouse(Index agI, Index hI);
        
        double ComputeLocalUtility( 
                Index hI, 
                const Scope& tupleOfAgents,
                const std::vector<Index>& actions,
                const std::vector<Index>& types
                );
        double Likelihood(Index houseI, const std::vector<observation_t>& oVec_hI, Index fireLevel) const;
        double Prior(Index houseI, Index fireLevel) const;

        void ScatterHouses();
        void ScatterAgents();
        void AssignAgentActionsToHouses();
        void AssignAgentTypesToHouses();
        void AddEdge(Index hI);
        const Scope& GetAgentsForHouse(Index houseI);

        ///Extracts all the observations that concern house hI from the types of tupleOfAgents
        std::vector<observation_t> FilterObservationsForHouse(
                Index hI,  
                const Scope& tupleOfAgents,
                const std::vector<Index>& types
            ) const;
        double FLObservationProb(Index fireLevel, Problem_CGBG_FF::observation_t obs) const;

        size_t ComputeNumberOfAgentsPresentAtHouse(Index houseI, const std::vector<Index>& a_vec) const;
        double GetHouseReward(Index fl, size_t nrAgentsPresent) const;


        ///Compute the probability of an individual type given the vector of fire levels (used in MAID export)
        double ComputeIndividualTypeProb(Index agI, Index typeI, std::vector<Index> & FL_vec) const;
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        Problem_CGBG_FF(
                size_t nrHouses,
                size_t nrFLs,
                size_t nrAgents,
                size_t nrActionsPerAgent,   //i.e., z
                size_t nrObservedHousesPerAgent,     //i.e., y,
                size_t k                   //the maximum number of agents in a house's payoff function
            );
        /// Copy constructor.
        //Problem_CGBG_FF(const Problem_CGBG_FF& a);

        /// Destructor.
        ~Problem_CGBG_FF();
        /// Copy assignment operator
        //Problem_CGBG_FF& operator= (const Problem_CGBG_FF& o);

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:

//implement the BayesianGameIdenticalPayoffInterface - OVERRIDE version from BayesianGameCollaborativeGraphical
        std::string SoftPrintReadableType(Index agI, Index typeI) const;

        double GetProbability(Index jtype) const;

        void ExportToTextFiles(const std::string &suffix="",const std::string &prefix="/tmp/") const;
        
        virtual void ExportAsMAID(const std::string &suffix="",const std::string &prefix="/tmp/") const;

        std::string GetUnixName() const;
};



#endif /* !_PROBLEM_CGBG_FF_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
