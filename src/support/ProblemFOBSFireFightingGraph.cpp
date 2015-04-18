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

#include "ProblemFOBSFireFightingGraph.h"

using namespace std;

//Default constructor
ProblemFOBSFireFightingGraph::ProblemFOBSFireFightingGraph(size_t nrAgents,
                                                   size_t nrFireLevels) :
    // number of houses is one more than the number of agents
    ProblemFOBSFireFightingFactored(nrAgents,nrAgents+1,nrFireLevels,
                                // no move cost for Graph version
                                0.0,
                                // dont include positions of agents in state
                                false,
                                // don't initialize PFFF, need to do
                                // that after PFFG has been
                                // constructed
                                false)
{
    InitializePFFF();
}

string ProblemFOBSFireFightingGraph::SoftPrintBriefDescription(
    size_t nrAgents, size_t nrHouses, size_t nrFLs) const
{
    stringstream ss;
    ss << "_FireFightingGraph_" << nrAgents << 
        "_" << nrHouses <<
        "_" << nrFLs;
    return ss.str();
}

string ProblemFOBSFireFightingGraph::SoftPrintDescription(size_t nrAgents,
                                                      size_t nrHouses,
                                                      size_t nrFLs) const
{
    stringstream ss;
    ss << "The fully-observable factored graph FireFighting problem with " 
        << nrAgents << 
        " Agents, " << nrHouses << " houses and "
        << nrFLs << " fire levels for each house.\n" <<
"Factored means that the state space is factored, and thus that the \
transition and reward models are represented in a factored way \
(by a 2DBN and a collection of reward functions).\n\
Graph means that the actions of the agents are restricted (i.e., they \
can go to only two houses)";
    return ss.str();
}

void ProblemFOBSFireFightingGraph::ConstructActions()
{
    //cout << "ProblemFOBSFireFightingGraph::ConstructActions()" << endl;
    for(Index agentIndex=0; agentIndex < GetNrAgents(); agentIndex++)
    {
        AddAction(agentIndex,
                  "left",
                  "Go and fight fire on the house on the left");
        AddAction(agentIndex,
                  "right",
                  "Go and fight fire on the house on the right");
    }
}

size_t ProblemFOBSFireFightingGraph::GetAgentLocation(Index action,
                                                  Index agI) const
{
    return(agI+action);
}

Scope ProblemFOBSFireFightingGraph::GetHousesAgentInfluences(Index agI) const
{
    Scope houses;
    // agent's actions only influence two houses
    houses.Insert(agI);
    houses.Insert(agI+1);
    return(houses);
}


void ProblemFOBSFireFightingGraph::SetYScopes()
{    
//specify connections for the 2DBN
#if DEBUG_PFFF
    cout << "About to set the SoIs for all next-stage(NS) SFs Y..."<<endl;
#endif
    Scope allAgents;
    for(Index agI=0; agI < GetNrAgents(); agI++)
        allAgents.Insert(agI);

    for(Index yI=0; yI < _m_nrHouses; yI++)
    {
        //determine the X scope of influence (i.e., state factors at prev.stage)
        Scope x;
        Scope a;
        if(yI > 0)
        {
            x.Insert(yI-1);
            a.Insert(yI-1);
        }
        x.Insert(yI); // firelevel of house yI influenced by its PS firelevel
        if(yI < (_m_nrHouses-1) )
        {
            x.Insert(yI+1);
            a.Insert(yI);
        }

        SetSoI_Y( yI, //sfacI = 0 -> house 0
                  x,
                  a,
                  //allAgents,
                  Scope("<>") // no interdependencies between next-stage FLs
                );
    }
}

size_t ProblemFOBSFireFightingGraph::GetNrAgentsAtHouse(const std::vector< Index>& As, Index hI) const {
    Index y = hI;
    const Scope& ASoI_y = GetASoI_Y(y);

    size_t nrAgentsAtLocation = 0;
    for(Index i=0; i < ASoI_y.size(); i++)
    {
        Index agI = ASoI_y.at(i);
        if(GetAgentLocation(As.at(i),agI) == hI)
            nrAgentsAtLocation++;
    }
    return(nrAgentsAtLocation);
}
