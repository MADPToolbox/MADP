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

#include "ProblemAloha.h"
#include "StringTools.h"
#include "FSDist_COF.h"
#include "RewardModelMapping.h"
#include "MultiAgentDecisionProcessDiscreteFactoredStatesInterface.h"

using namespace std;

// useful for debugging etc, no observation noise and each timestep an
// island always gets a packet
#define DETERMINISTIC_ALOHA 0

//Default constructor
ProblemAloha::ProblemAloha(IslandConfiguration islands,
                           AlohaVariation variation,
                           size_t maxBacklog,
                           size_t nrAgents,
                           bool initialize):
    FactoredDecPOMDPDiscrete("", "", "none"),
    _m_islandConf(islands),
    _m_variation(variation),
    _m_maxBacklog(maxBacklog),
    _m_nrAgentsPassedOnCommandline(nrAgents)
{
    if(initialize)
        InitializeAloha();
}


void ProblemAloha::InitializeAloha()
{
    SetSparse(true);

    /* -----------'ConstructNrAgents()' ------- */
    switch(_m_islandConf)
    {
    case OneIsland:
        _m_nrIslands=1;
        break;
    case TwoIslands:
    case TwoIndependentIslands:
        _m_nrIslands=2;
        break;
    case ThreeIslandsInLine:
    case ThreeIslandsClustered:
    case SmallBigSmallInLine:
        _m_nrIslands=3;
        break;
    case FourIslandsInLine:
    case FourIslandsInSquare:
        _m_nrIslands=4;
        break;
    case FiveIslandsInLine:
        _m_nrIslands=5;
        break;
    case SixIslandsInLine:
        _m_nrIslands=6;
        break;
    case SevenIslandsInLine:
        _m_nrIslands=7;
        break;
    case InLine:
        _m_nrIslands=_m_nrAgentsPassedOnCommandline;
        break;
    default:
        throw(E("InitializeAloha() island config not handled"));
    }
    SetNrAgents(_m_nrIslands);

    // first we need to set the number of agents before the
    // SoftPrintBriefDescription() is correct
    SetName(SoftPrintBriefDescription());
    SetDescription(SoftPrintDescription());
    SetUnixName(SoftPrintBriefDescription());

    vector<size_t> nrElems(2);
    nrElems[0]=_m_maxBacklog;
    nrElems[1]=2; // yes or no
    _m_stepSizeState=IndexTools::CalculateStepSize(nrElems);
    nrElems[0]=3;
    nrElems[1]=2; // yes or no
    _m_stepSizeObservations=IndexTools::CalculateStepSize(nrElems);

    /* 
     * -----------'ConstructStateSpace() -------
     */
    for(Index iI = 0; iI < _m_nrIslands; iI++)
    {
        Index sfI = AddStateFactor( 
                StringTools::Append("BL",iI) , 
                StringTools::Append("The back log of island ",iI));

        switch(_m_variation)
        {
        case NewPacket:
        case NewPacketSendAll:
        case NewPacketProgressivePenalty:
        {
            for(Index sI=0;sI!=(_m_maxBacklog+1)*2;++sI)
            {
                Index bI,newPacket;
                splitState(sI,bI,newPacket);
                stringstream ss;
                ss << "bl" << bI;
                if(newPacket)
                    ss << "newP";
                else
                    ss << "none";
                
                AddStateFactorValue(sfI, ss.str());
            }
            break;
        }
        case NoNewPacket:
        {
            for(Index bI=0; bI <= _m_maxBacklog; bI++)
                AddStateFactorValue(sfI,
                                    StringTools::Append("bl",bI));
            break;
        }
        }
    }

    SetStatesInitialized(true);


    /*-----------'ConstructInitialStateDistribution' ------- */
    //after initialization we can add the ISD:
    FSDist_COF *isd=
        new FSDist_COF(*this);
    for(Index iI = 0; iI < _m_nrIslands; iI++)
    {
        switch(_m_variation)
        {
        case NewPacket:
        case NewPacketSendAll:
        case NewPacketProgressivePenalty:
            // each island starts with backlog 0 and no new packet
            isd->SetProbability(iI,composeState(0,0),1.0); 
            break;
        case NoNewPacket:
            isd->SetProbability(iI,0,1.0); // each island starts with backlog 0
            break;
        }
    }
    SetISD(isd);

// add actions:
    ConstructActions();
#if 0 // for factored models we typically don't want to use joint indices
    ConstructJointActions();
#endif
    SetActionsInitialized(true);

// add observations:
    ConstructObservations();
#if 0 // for factored models we typically don't want to use joint indices
    ConstructJointObservations();
#endif
    SetObservationsInitialized(true);

    // Initialize the 2DBN in MultiAgentDecisionProcessDiscreteFactoredStates
    Initialize2DBN();

//    cout << ">>>Trans./obs. models created"<<endl;
//    cout << _m_2dbn.SoftPrint();


//add rewards
    SetNrLRFs(_m_nrIslands);
    //first add scope for each reward function
    for(Index e=0; e < _m_nrIslands; e++)
    {
        //cout << "Setting scope for reward function e=" << e << endl;
        //Add scope
        Scope emptySc;
        Scope ySc;
        ySc.Insert(e);//reward e depends on backlog of island e
        Scope aSc;
        aSc.Insert(e); // this seems necessary...
        SetScopeForLRF(e, emptySc, aSc, ySc, emptySc);
        //const Scope& agSC = GetAgentScopeForLRF(e);
        //cout << "agent scope = " << agSC.SoftPrint();
    }
    // compute some bookkeeping from the scopes
    InitializeInstantiationInformation();

    // now we add the reward functions themselves
    for(Index e=0; e < _m_nrIslands; e++)
    {
        //cout << "Adding reward function e=" << e << endl;
        Scope emptySc;
        Scope ySc;
        ySc.Insert(e);//reward e depends on backlog of island e
        const Scope& sfSC = GetStateFactorScopeForLRF(e);
        const Scope& agSC = GetAgentScopeForLRF(e);
        string sf_descr = sfSC.SoftPrint();
        string ag_descr = agSC.SoftPrint();
        //the number of X instantiations (the size of the 'local' state space)
        size_t nrXIs = GetNrXIs(e);
        size_t nrAIs = GetNrAIs(e);
#if 0
        cout << "nrXIs " << nrXIs << " nrAIs " << nrAIs << " agSC " << agSC
             << " sfSC " << sfSC << endl;
#endif
        RewardModelMapping* RMe = 
            new RewardModelMapping(nrXIs, nrAIs, sf_descr, ag_descr);
        SetRM(e, RMe);
        
        for(Index bI=0; bI <= _m_maxBacklog; bI++)
        {
            vector<Index> emptyVec;
            vector<Index> yVals(1);
            
            switch(_m_variation)
            {
            case NewPacket:
            case NewPacketSendAll:
            case NewPacketProgressivePenalty:
            {
                for(Index j=0;j!=2;++j)
                {
                    yVals[0]=composeState(bI,j);
                    SetRewardForLRF(  e,
                                      emptySc, emptyVec, //X scope + value
                                      emptySc, emptyVec, //A scope + value
                                      ySc, yVals,        //Y scope + value
                                      emptySc, emptyVec, //O scope + value
                                      backlogToReward(bI) //the reward
                        );
                }
                break;
            }
            case NoNewPacket:
            {
                yVals[0]=bI;
                SetRewardForLRF(  e,
                                  emptySc, emptyVec, //X scope + value
                                  emptySc, emptyVec, //A scope + value
                                  ySc, yVals,        //Y scope + value
                                  emptySc, emptyVec, //O scope + value
                                  backlogToReward(bI) //the reward
                    );
                break;
            }
            }
        }
    }

    FactoredDecPOMDPDiscrete::SetInitialized(true);
}

double ProblemAloha::backlogToReward(Index backlog) const
{
    double reward=42;
    switch(_m_variation)
    {
    case NewPacket:
    case NewPacketSendAll:
    case NoNewPacket:
        reward=-static_cast<double>(backlog);
        break;
    case NewPacketProgressivePenalty:
        reward=1-pow(2,static_cast<double>(backlog));
        break;
    }
    return(reward);
}

void
ProblemAloha::splitState(Index sI, Index &backlog, Index &newPacket) const
{
    vector<Index> splitS=
        IndexTools::JointToIndividualIndicesStepSize(sI,_m_stepSizeState,2);
    backlog=splitS[0];
    newPacket=splitS[1];
}

Index ProblemAloha::composeState(Index backlog, Index newPacket) const
{
    vector<Index> splitS(2);
    splitS[0]=backlog;
    splitS[1]=newPacket;
    return(IndexTools::IndividualToJointIndicesStepSize(splitS,
                                                        _m_stepSizeState));
}


void
ProblemAloha::splitObservation(Index oI, Index &status, Index &newPacket) const
{
    vector<Index> splitO=
        IndexTools::JointToIndividualIndicesStepSize(oI,
                                                     _m_stepSizeObservations,
                                                     2);
    status=splitO[0];
    newPacket=splitO[1];
}

Index ProblemAloha::composeObservation(Index status, Index newPacket) const
{
    vector<Index> splitO(2);
    splitO[0]=status;
    splitO[1]=newPacket;
    return(IndexTools::IndividualToJointIndicesStepSize(splitO,
                                                        _m_stepSizeObservations));
}

bool ProblemAloha::areNeighbors(Index x, Index y) const
{
    bool neighbors=false;
    if(x==y) // you cannot be a neighbor of yourself
        return(false);

    switch(_m_islandConf)
    {
    case TwoIslands:
        // with two islands everybody's a neigbor
        neighbors=true;
        break;
    case OneIsland:
        break;
    case TwoIndependentIslands:
        break;
    case ThreeIslandsInLine:
    case SmallBigSmallInLine:
    case FourIslandsInLine:
    case FiveIslandsInLine:
    case SixIslandsInLine:
    case SevenIslandsInLine:
    case InLine:
        if(abs(static_cast<int>(x)-
               static_cast<int>(y))==1)
            neighbors=true;
        break;
    case ThreeIslandsClustered:
        // three islands clustered together
        neighbors=true;
        break;
    case FourIslandsInSquare:
        if(((x==0 && y==1) || (y==0 && x==1)) || // island 0 and 1 are neighbors
           ((x==1 && y==2) || (y==1 && x==2)) || // as well as 1 and 2
           ((x==2 && y==3) || (y==2 && x==3)) || // as well as 2 and 3
           ((x==0 && y==3) || (y==0 && x==3))) // as well as 0 and 3
            neighbors=true;
        break;
    default:
        throw(E("ProblemAloha::areNeighbors not implemented for island configuration"));
    }
    return(neighbors);
}

double ProblemAloha::GetNewPacketProb(Index y) const
{
#if DETERMINISTIC_ALOHA
    double pHighProb=1.0,
        pLowProb=1.0,
        p=42;
#else
    double pHighProb=0.6,
        pLowProb=0.2,
        p=42;
#endif

    switch(_m_islandConf)
    {
    case TwoIslands:
    case OneIsland:
    case TwoIndependentIslands:
    case ThreeIslandsInLine:
    case ThreeIslandsClustered:
    case FiveIslandsInLine:
    case FourIslandsInLine:
    case FourIslandsInSquare:
    case SixIslandsInLine:
    case SevenIslandsInLine:
    case InLine:
        p=pHighProb;
        break;
    case SmallBigSmallInLine:
        if(y==1) // middle island is big -> high prob
            p=pHighProb;
        else
            p=pLowProb;
        break;
    default:
        throw(E("ProblemAloha::GetNewPacketProb not implemented for island configuration"));

    }
    return(p);
}

string ProblemAloha::IslandConfigToString(IslandConfiguration conf)
{
    switch(conf)
    {
    case TwoIslands:
        return("TwoIslands");
    case OneIsland:
        return("OneIsland");
    case TwoIndependentIslands:
        return("TwoIndependentIslands");
    case ThreeIslandsInLine:
        return("ThreeIslandsInLine");
    case ThreeIslandsClustered:
        return("ThreeIslandsClustered");
    case SmallBigSmallInLine:
        return("SmallBigSmallInLine");
    case FiveIslandsInLine:
        return("FiveIslandsInLine");
    case FourIslandsInLine:
        return("FourIslandsInLine");
    case FourIslandsInSquare:
        return("FourIslandsInSquare");
    case SixIslandsInLine:
        return("SixIslandsInLine");
    case SevenIslandsInLine:
        return("SevenIslandsInLine");
    case InLine:
        return("InLine");
    }

    return("IslandConf not handled");
}


string ProblemAloha::SoftPrintBriefDescription() const
{
    stringstream ss;
    ss << "Aloha_" << SoftPrintVariation(GetVariation())
       << "_";
    if(GetIslandConfiguration()==InLine)
        ss << GetNrAgents();
    ss << IslandConfigToString(GetIslandConfiguration()) << "_maxBL"
       << GetMaxBacklog();
    return ss.str();
}

string ProblemAloha::SoftPrintDescription() const
{
    return SoftPrintBriefDescription();
}

string ProblemAloha::SoftPrintVariation(AlohaVariation variation) const
{
    switch(variation)
    {
    case NoNewPacket:
        return("NoNewPacket");
    case NewPacket:
        return("NewPacket");
    case NewPacketSendAll:
        return("NewPacketSendAll");
    case NewPacketProgressivePenalty:
        return("NewPacketProgressivePenalty");
    }

    return("AlohaVariation not handled");
}

string ProblemAloha::transmissionStatusToString(Index status) const
{
    string str;
    switch(status){
    case(SUCCESS):
        str = "Succ";
        break;
    case(IDLEo):
        str = "Idle";
        break;
    case(COLLISION):
        str = "Coll";
        break;
    }
    return(str);
}

void ProblemAloha::ConstructActions()
{
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
//         _m_nrActions.push_back(2);
//         _m_actionVecs.push_back(vector<ActionDiscrete>() );
        if(SEND==0) // make sure we add them in the correct order
        {
            AddAction(agentIndex,"send","Attempt to send a packet");
            AddAction(agentIndex,"idle","Do nothing, remain idle");
        }
        else
        {
            AddAction(agentIndex,"send","Attempt to send a packet");
            AddAction(agentIndex,"idle","Do nothing, remain idle");
        }

    }
}

void ProblemAloha::ConstructObservations()
{
    /// add observations:
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        size_t nrObservationsThisAgent = 0;
        switch(_m_variation)
        {
        case NewPacket:
        case NewPacketSendAll:
        case NewPacketProgressivePenalty:
            nrObservationsThisAgent = 6;
            break;
        case NoNewPacket:
            nrObservationsThisAgent = 3; // success, idle, collision
            break;
        }

//         _m_nrObservations.push_back(nrObservationsThisAgent);
//         _m_observationVecs.push_back( vector<ObservationDiscrete>() );
        for(Index obsI=0; obsI < nrObservationsThisAgent; obsI++)
        {
            stringstream ss;

            switch(_m_variation)
            {
            case NewPacket:
            case NewPacketSendAll:
            case NewPacketProgressivePenalty:
            {
                Index status,newPacket;
                splitObservation(obsI,status,newPacket);
                ss << transmissionStatusToString(status);
                if(newPacket)
                    ss << "NewP";
                else
                    ss << "None";
                break;
            }
            case NoNewPacket:
            {
                ss << transmissionStatusToString(obsI);
                break;
            }
            }
            string name = ss.str();
            ss.str("");
            ss << "Observation " << obsI << " of agent "
               << agentIndex << ": " << name;
            string descr = ss.str();
            AddObservation(agentIndex,name,descr);
//             ObservationDiscrete od_temp = ObservationDiscrete(obsI,name,descr);
//             _m_observationVecs[agentIndex].push_back( od_temp );
        }
    }
}

void ProblemAloha::SetYScopes()
{    
//specify connections for the 2DBN

    for(Index yI=0; yI < _m_nrIslands; yI++)
    {
        //determine the X scope of influence (i.e., state factors at prev.stage)
        Scope x;
        Scope a;
        // depends on your own state and action
        x.Insert(yI);
        a.Insert(yI);
        for(Index yI1=0; yI1 < _m_nrIslands; yI1++)
            if(areNeighbors(yI,yI1)) // and on your neighbors
            {
                x.Insert(yI1);
                a.Insert(yI1);
            }

        SetSoI_Y( yI,
                  x,
                  a,
                  Scope("<>") // no interdependencies in next stage
            );
    }
}

void ProblemAloha::SetOScopes()
{
    for(Index oI=0; oI < _m_nrAgents; oI++)
    {
        Scope aSC,ySC;

        aSC.Insert(oI);
        for(Index yI1=0; yI1 < _m_nrIslands; yI1++)
            if(areNeighbors(oI,yI1)) // and on your neighbors
                aSC.Insert(yI1);
        //ySC=aSC;
        ySC.Insert(oI);

        SetSoI_O( oI,
                  aSC,
                  ySC,
                  Scope("<>") // no interdependencies between next-stage Os
            );
    }
}

double ProblemAloha::ComputeTransitionProb(
            Index y,
            Index yVal,
            const std::vector< Index>& Xs,
            const std::vector< Index>& As,
            const std::vector< Index>& Ys
        ) const
{
    double pNewPacket=GetNewPacketProb(y);
    double p=42;
    bool successfullSend=successFullySendPackage(y,Xs,As);

    const Scope& XSoI = GetXSoI_Y(y);
    Index Index_of_island_within_scope=XSoI.GetPositionForIndex(y);
    Index currentBacklog, nextBacklog;
    Index currentNewPacket, nextNewPacket;

    switch(_m_variation)
    {
    case NewPacket:
    case NewPacketSendAll:
    case NewPacketProgressivePenalty:
        splitState(Xs.at(Index_of_island_within_scope), currentBacklog,
                   currentNewPacket);
        splitState(yVal, nextBacklog, nextNewPacket);
        break;
    case NoNewPacket:
        currentBacklog=Xs.at(Index_of_island_within_scope);
        nextBacklog=yVal;
        break;
    }
    bool gotPacket=false;

    if(successfullSend)
    {
        switch(_m_variation)
        {
        case NewPacket:
        case NewPacketProgressivePenalty:
        case NoNewPacket:
            // in these variations we send 1 packet
            if(currentBacklog==0 && nextBacklog==0)
                p=1;
            else if(nextBacklog==currentBacklog) // sent a packet but got a new one
            {
                p=pNewPacket;
                gotPacket=true;
            }
            else if(nextBacklog==(currentBacklog-1)) // sent a packet
                p=1-pNewPacket;
            else
                p=0;
            break;
        case NewPacketSendAll:
            // in these variation we send all packets we have in the backlog
//             if(currentBacklog==0 && nextBacklog==0)
//                 p=1;
//             else
 if(nextBacklog==1) // sent all packets but got a new one
            {
                p=pNewPacket;
                gotPacket=true;
            }
            else if(nextBacklog==0) // sent all packets
                p=1-pNewPacket;
            else
                p=0;
            break;
        }
    }
    else
    {
        if(currentBacklog==_m_maxBacklog &&
           nextBacklog==currentBacklog)  // backlog is full
            p=1;
        else if(nextBacklog==(currentBacklog+1)) // we got a packet
        {
            p=pNewPacket;
            gotPacket=true;
        }
        else if(nextBacklog==currentBacklog) // no new packet
            p=1-pNewPacket;
        else
            p=0;
    }

    switch(_m_variation)
    {
    case NewPacket:
    case NewPacketSendAll:
    case NewPacketProgressivePenalty:
        // check if new state is consistent with whether we received a
        // packet or not
        if((nextNewPacket && !gotPacket) ||
           (!nextNewPacket && gotPacket))
            p=0;
        break;
    case NoNewPacket:
        break;
    }

#if 0
    if(p>0)
        cout << endl << "aloha y " << y << " yVal " << yVal << SoftPrintVector(Xs) << SoftPrintVector(As) << " p " << p << " send " << successfullSend << " cBL " << currentBacklog << " nBL " << nextBacklog << endl;
#endif

    return(p);
}

bool ProblemAloha::successFullySendPackage(Index y,
                                           const std::vector< Index>& Xs,
                                           const std::vector< Index>& As) const
{
    bool iSend=false;
    bool neighborsTryToSend=false;

    const Scope& XSoI = GetXSoI_Y(y);
    Index Index_of_island_within_scope=XSoI.GetPositionForIndex(y);

    if(As.at(Index_of_island_within_scope)==SEND)  // we tried to send a packet
        iSend=true;

    for(Index i=0;i!=XSoI.size();++i)
        if(XSoI[i]!=y)
        {
            if(As.at(i)==SEND) // (the neighbor tried to send
                neighborsTryToSend=true;
        }

    // if I try to send and none of my neighbors, transmission is a success
    if(iSend &&
       !neighborsTryToSend)
        return(true);
    else
        return(false);
}

double ProblemAloha::ComputeObservationProb(
            Index o,
            Index oVal,
            const std::vector< Index>& As,
            const std::vector< Index>& Ys,
            const std::vector< Index>& Os
                ) const
{
    double p=42;
    Index status;

    switch(_m_variation)
    {
    case NewPacket:
    case NewPacketSendAll:
    case NewPacketProgressivePenalty:
    {
        const Scope& YSoI = GetYSoI_O(o);
        Index Index_of_island_within_scope=YSoI.GetPositionForIndex(o);
        Index backlog,newPacket;
        splitState(Ys.at(Index_of_island_within_scope), backlog,
                   newPacket);
    
        Index newPacketObs;
        splitObservation(oVal,status,newPacketObs);
        
        // check for consistency of observation with state
        if(newPacket!=newPacketObs)
            return(0);
        break;
    }
    case NoNewPacket:
        status=oVal;
        break;
    }

    int nrSends=0;
    for(Index i=0;i!=As.size();++i)
        if(As[i]==SEND)
            nrSends++;
    Index correctObs;

    if(nrSends==0)
        correctObs=IDLEo;
    else if(nrSends==1)
        correctObs=SUCCESS;
    else
        correctObs=COLLISION;

#if DETERMINISTIC_ALOHA
    double pCorrectObs=1.0;
#else
    double pCorrectObs=0.9;
#endif
    if(status==correctObs)
        p=pCorrectObs;
    else
        p=(1-pCorrectObs)/2;

    return(p);
}
