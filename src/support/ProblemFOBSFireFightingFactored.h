/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Philipp Robbel 
 *
 * For contact information please see the included AUTHORS file.
 */

/* Only include this header file once. */
#ifndef _PROBLEMFOBSFIREFIGHTINGFACTORED_H_
#define _PROBLEMFOBSFIREFIGHTINGFACTORED_H_ 1

/* the include directives */
#include "FactoredDecPOMDPDiscrete.h"
#include "FactoredMMDPDiscrete.h"
#include <iostream>


/** \brief This is a template of how to implement a fully-observable problem
 * from scratch (by deriving from FactoredMMDPDiscrete).
 *
 * As an example, we'd like to implement a fully-observable FireFightingFactored.
 * Since MADP already contains a (partially-observable) implementation, we merely
 * copy most of it into our versions of ProblemFOBSFireFightingFactored 
 * below. The only differences concern the generation of observations which is
 * hidden from the user inside FactoredMMDPDiscrete in the fully-observable
 * case.
 * 
 * In summary, the MADP interface for defining decision problem instances remains
 * identical. For partially-observable problems, derive from
 * FactoredDecPOMDPDiscrete, for fully-observable problems from FactoredMMDPDiscrete
 * (and do not worry about defining observations in the latter case).
 */
class ProblemFOBSFireFightingFactored : public FactoredMMDPDiscrete
{
protected:
    size_t _m_nrAgents;
    size_t _m_nrHouses;
    size_t _m_nrFireLevels;
    double _m_costOfMove;
    bool _m_forcePositionRepres;

    bool _m_includePositions;

    size_t _m_nrStateFeatures;
    //vector that stores the number of values per state feature.
    std::vector<size_t> _m_nrPerStateFeatureVec;

    size_t _m_nrJointFirelevels;
    //vector that stores the number of values per state feature.
    std::vector<size_t> _m_nrFLs_vec;
    
    std::vector< Index> GetStateVector(Index sI) const {
        return IndexTools::JointToIndividualIndices
                (sI, _m_nrPerStateFeatureVec );
    }
    ///Compute the probability of value yVal of state variable y, given ii=<Xs,As,Ys>
    double ComputeTransitionProb(
        Index y,
        Index yVal,
        const std::vector< Index>& Xs,
        const std::vector< Index>& As,
        const std::vector< Index>& Ys) const;        
    
    virtual size_t GetNrAgentsAtHouse(const std::vector< Index>& As,
                                      Index hI) const;

    virtual size_t GetAgentLocation(Index action,
                                    Index agI) const;

    virtual std::string SoftPrintBriefDescription(
        size_t nrAgents, size_t nrHouses, size_t nrFLs) const;
    virtual std::string SoftPrintDescription(size_t nrAgents,
                                             size_t nrHouses,
                                             size_t nrFLs) const;
    
    ///Construct all the Actions and actionSets (the vector _m_actionVecs).
    virtual void ConstructActions();
    
    virtual Scope GetHousesAgentInfluences(Index agI) const;

    void InitializePFFF();

    virtual void SetYScopes();

public:
    ///Constructor
    ProblemFOBSFireFightingFactored(
        size_t nrAgents, size_t nrHouses, size_t nrFireLevels,
        double costOfMove=0.0, bool forcePositionRepres = false,
        bool initialize=true);

    /// Destructor.
    virtual ~ProblemFOBSFireFightingFactored(){};

    /// Returns a pointer to a copy of this class.
    virtual ProblemFOBSFireFightingFactored* Clone() const
    { return new ProblemFOBSFireFightingFactored(*this); }
};

#endif /* !_PROBLEMFOBSFIREFIGHTINGFACTORED_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
