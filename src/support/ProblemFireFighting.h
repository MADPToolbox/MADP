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
#ifndef _PROBLEMFIREFIGHTING_H_
#define _PROBLEMFIREFIGHTING_H_ 1

/* the include directives */
#include "Globals.h"
#include "DecPOMDPDiscrete.h"
#include "IndexTools.h"

/** \brief ProblemFireFighting is a class that represents 
 * the firefighting problem as described in #refGMAA (DOC-references.h).
 * */
class ProblemFireFighting : public DecPOMDPDiscrete
{
    private: 
        enum observation_t { FLAMES, NOFLAMES };
        size_t _m_nrAgents;
        size_t _m_nrHouses;
        size_t _m_nrFireLevels;
        bool _m_includePositions;

        size_t _m_nrStateFeatures;
        //vector that stores the number of values per state feature.
        std::vector<size_t> _m_nrPerStateFeatureVec;

        size_t _m_nrJointFirelevels;
        //vector that stores the number of values per state feature.
        std::vector<size_t> _m_nrFLs_vec;
        
        ///Construct all the Actions and actionSets (the vector _m_actionVecs).
        void ConstructActions();
        ///Construct all the observations and observation sets.
        void ConstructObservations();
        ///Fills the transition model with the  problem transitions.
        void FillTransitionModel();
        ///Fills the observation model with the  problem obs. probs.
        void FillObservationModel();
        ///Fills the reward model with the  problem rewards.
        void FillRewardModel();
    
        size_t NumberOfContainedStartPositions(const 
                std::vector<Index>& state) const;
    protected:
        static std::string SoftPrintBriefDescription(
                size_t nrAgents, size_t nrHouses, size_t nrFLs);
        static std::string SoftPrintDescription(size_t nrAgents,
                size_t nrHouses, size_t nrFLs);
        std::vector< Index> GetStateVector(Index sI) const
        {
            return IndexTools::JointToIndividualIndices
                (sI, _m_nrPerStateFeatureVec );
        }
        double ComputeTransitionProb(
                const std::vector< Index>& s1,
                const std::vector< Index>& ja,
                const std::vector< Index>& s2
                ) const;        
        double ComputeObservationProb(
                const std::vector< Index>& ja,
                const std::vector< Index>& s1,
                const std::vector< Index>& jo
                ) const;
        double ComputeReward(Index sI) const;
        //is a neighbor of house hI burning?
        static bool isNeighborBurning( const std::vector< Index>& s1, 
                Index hI) ;

    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        ProblemFireFighting(size_t nrAgents, size_t nrHouses, size_t nrFLs,
                double costOfMove=0.0, bool forcePositionRepres = false);
};


#endif /* !_PROBLEMFIREFIGHTING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
