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
                               SixIslandsInLine, SevenIslandsInLine};

    enum AlohaVariation { NoNewPacket, NewPacket, NewPacketSendAll,
                          NewPacketProgressivePenalty };

private:    
    
    IslandConfiguration _m_islandConf;
    AlohaVariation _m_variation;
    size_t _m_maxBacklog;
    size_t _m_nrIslands;

    std::string SoftPrintBriefDescription(
        IslandConfiguration islands) const;
    std::string SoftPrintDescription(IslandConfiguration islands) const;
    std::string SoftPrintVariation(AlohaVariation variation) const;

    void InitializeAloha();
    
    void ConstructActions();
    void ConstructObservations();

    enum { SEND, IDLEa };
    enum { SUCCESS, IDLEo, COLLISION };


    bool successFullySendPackage(Index y,
                                 const std::vector< Index>& Xs,
                                 const std::vector< Index>& As) const;
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
                 size_t maxBacklog=2);

    /// Destructor.
    virtual ~ProblemAloha(){};

    static std::string IslandConfigToString(IslandConfiguration conf);
    
    /// Returns a pointer to a copy of this class.
    virtual ProblemAloha* Clone() const
        { return new ProblemAloha(*this); }

};


#endif /* !_PROBLEMALOHA_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
