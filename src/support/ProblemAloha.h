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
#ifndef _PROBLEMALOHA_H_
#define _PROBLEMALOHA_H_ 1

/* the include directives */
#include "Globals.h"
#include "FactoredDecPOMDPDiscrete.h"

class ProblemAloha : public FactoredDecPOMDPDiscrete
{
public:

    enum IslandConfiguration { TwoIslands, OneIsland, TwoIndependentIslands,
                               ThreeIslandsInLine, ThreeIslandsClustered,
                               SmallBigSmallInLine, FiveIslandsInLine,
                               FourIslandsInLine, FourIslandsInSquare,
                               SixIslandsInLine, SevenIslandsInLine, InLine};

    enum AlohaVariation { NoNewPacket, NewPacket, NewPacketSendAll,
                          NewPacketProgressivePenalty };

private:    
    
    IslandConfiguration _m_islandConf;
    AlohaVariation _m_variation;
    size_t _m_maxBacklog;
    size_t _m_nrIslands;
    size_t _m_nrAgentsPassedOnCommandline;

    void ConstructActions();
    void ConstructObservations();


    bool areNeighbors(Index x1, Index x2) const;
    double GetNewPacketProb(Index y) const;
    double backlogToReward(Index backlog) const;
    std::string transmissionStatusToString(Index status) const;

    size_t *_m_stepSizeState;
    size_t *_m_stepSizeObservations;
    void splitState(Index sI, Index &backlog, Index &newPacket) const;
    Index composeState(Index backlog, Index newPacket) const;
    void splitObservation(Index oI, Index &status, Index &newPacket) const;
    Index composeObservation(Index status, Index newPacket) const;


protected:

    enum { SEND, IDLEa };
    enum { SUCCESS, IDLEo, COLLISION };

    void InitializeAloha();
    
    virtual std::string SoftPrintBriefDescription() const;
    virtual std::string SoftPrintDescription() const;
    virtual std::string SoftPrintVariation(AlohaVariation variation) const;

    virtual bool successFullySendPackage(Index y,
                                         const std::vector< Index>& Xs,
                                         const std::vector< Index>& As) const;

    //overide scope functions
    virtual void SetYScopes();
    virtual void SetOScopes();

    double ComputeTransitionProb(
        Index y,
        Index yVal,
        const std::vector< Index>& Xs,
        const std::vector< Index>& As,
        const std::vector< Index>& Ys
        ) const;        
    double ComputeObservationProb(
        Index o,
        Index oVal,
        const std::vector< Index>& As,
        const std::vector< Index>& Ys,
        const std::vector< Index>& Os
        ) const;


public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    ProblemAloha(IslandConfiguration islands,
                 AlohaVariation variation,
                 size_t maxBacklog=2,
                 size_t nrAgents=2,
                 bool initialize=true);

    /// Destructor.
    virtual ~ProblemAloha(){};

    static std::string IslandConfigToString(IslandConfiguration conf);
    
    /// Returns a pointer to a copy of this class.
    virtual ProblemAloha* Clone() const
        { return new ProblemAloha(*this); }

    AlohaVariation GetVariation() const { return(_m_variation); }
    size_t GetMaxBacklog() const { return(_m_maxBacklog); }
    IslandConfiguration GetIslandConfiguration() const { return(_m_islandConf); }

};


#endif /* !_PROBLEMALOHA_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
