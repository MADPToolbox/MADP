/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ProblemDecTigerWithCreaks.h"

using namespace std;

#define DEBUG_PDT 0
#define DEBUG_TM 0
#define DEBUG_CJA 0
#define DEBUG_CJO 0
#define DEBUG_CA 0
#define DEBUG_CO 0
#define DEBUG_GETJA 0

ProblemDecTigerWithCreaks::ProblemDecTigerWithCreaks() :
    DecPOMDPDiscrete(
        "The Dec-Tiger with Creaks Problem",
        "Variation of the DecTiger problem",
        "DecTigerWithCreaks"),
    NUMBER_OF_STATES(2),
    NUMBER_OF_AGENTS(2),
    NUMBER_OF_OBSERVATIONS(6),
    NUMBER_OF_ACTIONS(3)
{
    SetNrAgents(NUMBER_OF_AGENTS);
    SetNrStates(NUMBER_OF_STATES);

    SetUniformISD();
    SetDiscount(1);
    
    // add actions:
    ConstructActions();
    // add joint actions
    ConstructJointActions();

    SetActionsInitialized(true);

    // add observations:
    ConstructObservations();
    ConstructJointObservations();

    SetObservationsInitialized(true);

    // add the transition model
    CreateNewTransitionModel();
    FillTransitionModel();    

    // add observation model
    CreateNewObservationModel();
    FillObservationModel();
    MultiAgentDecisionProcessDiscrete::SetInitialized(true);

    // add rewards
    CreateNewRewardModel();
    FillRewardModel();
    DecPOMDPDiscrete::SetInitialized(true);
}

ProblemDecTigerWithCreaks::~ProblemDecTigerWithCreaks()
{
}

void ProblemDecTigerWithCreaks::ConstructActions()
{
    size_t nrJointActions = 1;
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        size_t nrActionsThisAgent = NUMBER_OF_ACTIONS;
        nrJointActions *= nrActionsThisAgent;
        vector<ActionDiscrete> av_temp;
        for(Index actionI=0; actionI < nrActionsThisAgent; actionI++)
        {
            string name("a");
            stringstream ss;
            string strActionI, strAgentI;
            ss << agentIndex;
            ss >> strAgentI;
            name += strAgentI;
            ss.clear();
            ss << actionI;
            ss >> strActionI;
            name += strActionI;
            string whatAction;
            switch(actionI){
            case(LISTEN):    whatAction = "Listen";
                break;
            case(OPENLEFT):    whatAction = "OpenLeft";
                break;
            case(OPENRIGHT):whatAction = "OpenRight";
                break;
            }
            name += ":" + whatAction;

            string descr("Action ");
            descr += whatAction+" - #";
            descr += strActionI + " of agent " + strAgentI;  

            AddAction(agentIndex, name, descr);
            name.clear();
            strActionI.clear();
            strAgentI.clear();
            whatAction.clear();    
            descr.clear();
        }
    }
}

void ProblemDecTigerWithCreaks::ConstructObservations()
{
    /// add observations:
    size_t nrJointObservations = 1;
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        size_t nrObservationsThisAgent = NUMBER_OF_OBSERVATIONS;
        nrJointObservations *= nrObservationsThisAgent;
        for(Index obsI=0; obsI < nrObservationsThisAgent; obsI++)
        {
            string name("o");
            stringstream ss;
            string strObsI, strAgentI;        
            ss << agentIndex;
            ss >> strAgentI;
            name += strAgentI;
            ss.clear();
            ss << obsI;
            ss >> strObsI;
            name += strObsI; 
            string whatObs;
            switch(obsI){
            case(GROWLLEFT_CREAKLEFT):    whatObs = "GrowlLeftCreakRight";
                break;
            case(GROWLLEFT_CREAKRIGHT):whatObs = "GrowlLeftCreakRight";
                break;
            case(GROWLLEFT_SILENCE):    whatObs = "GrowlLeftSilence";
                break;
            case(GROWLRIGHT_CREAKLEFT):whatObs = "GrowlRightCreakLeft";
                break;
            case(GROWLRIGHT_CREAKRIGHT):    whatObs = "GrowlRightCreakRight";
                break;
            case(GROWLRIGHT_SILENCE):whatObs = "GrowlRightSilence";
                break;
            }
            name += ":" + whatObs;
            
            string descr("Observation ");
            descr += whatObs + " - #";
            descr += strObsI + " of agent " + strAgentI;
            AddObservation(agentIndex,name,descr);
            name.clear(); 
            strObsI.clear(); 
            strAgentI.clear(); 
            whatObs.clear();
            descr.clear();
        }
    }
}

Index ProblemDecTigerWithCreaks::GetJointActionIndex(Index a0,
                                                     Index a1) const
{
    vector<Index> aIs;
    aIs.push_back(a0);
    aIs.push_back(a1);
    return(IndividualToJointActionIndices(aIs));
}

Index ProblemDecTigerWithCreaks::GetJointObsIndex(Index o0,
                                                  Index o1) const
{
    vector<Index> oIs;
    oIs.push_back(o0);
    oIs.push_back(o1);
    return(IndividualToJointObservationIndices(oIs));
}

void ProblemDecTigerWithCreaks::FillTransitionModel()
{
    // add transitions:
    for(Index s1=0; s1<NUMBER_OF_STATES;s1++) 
        for(Index s2=0; s2<NUMBER_OF_STATES;s2++) 
            for(Index ja=0; ja<GetNrJointActions(); ja++)
                SetTransitionProbability(s1, ja, s2, 0.5);

    Index jaListenListen=GetJointActionIndex(LISTEN,LISTEN);
    SetTransitionProbability(SLEFT, jaListenListen, SLEFT, 1.0);
    SetTransitionProbability(SLEFT, jaListenListen, SRIGHT, 0.0);
    SetTransitionProbability(SRIGHT, jaListenListen, SRIGHT, 1.0);
    SetTransitionProbability(SRIGHT, jaListenListen, SLEFT, 0.0);
}

double ProblemDecTigerWithCreaks::GetObsProb(Index ja,
                                             Index s,
                                             Index agentI,
                                             Index oI) const
{
    switch(agentI)
    {
    case 0:
        switch(oI)
        {
        case GROWLLEFT_CREAKLEFT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
            { if(s==SLEFT) return(0.85*0.9); else if(s==SRIGHT) return(0.15*0.9);}
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
                return(1.0/6);
        case GROWLLEFT_CREAKRIGHT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
            { if(s==SLEFT) return(0.85*0.9); else if(s==SRIGHT) return(0.15*0.9);}
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
                return(1.0/6);
        case GROWLLEFT_SILENCE:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.85*0.9); else if(s==SRIGHT) return(0.15*0.9);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
                return(1.0/6);
        case GROWLRIGHT_CREAKLEFT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
            { if(s==SLEFT) return(0.15*0.9); else if(s==SRIGHT) return(0.85*0.9);}
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
                return(1.0/6);
        case GROWLRIGHT_CREAKRIGHT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
            { if(s==SLEFT) return(0.15*0.9); else if(s==SRIGHT) return(0.85*0.9);}
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
                return(1.0/6);
        case GROWLRIGHT_SILENCE:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.15*0.9); else if(s==SRIGHT) return(0.85*0.9);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
                return(1.0/6);
        }
    case 1:
        switch(oI)
        {
        case GROWLLEFT_CREAKLEFT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
            { if(s==SLEFT) return(0.85*0.9); else if(s==SRIGHT) return(0.15*0.9);}
        case GROWLLEFT_CREAKRIGHT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
            { if(s==SLEFT) return(0.85*0.9); else if(s==SRIGHT) return(0.15*0.9);}
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
        case GROWLLEFT_SILENCE:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.85*0.9); else if(s==SRIGHT) return(0.15*0.9);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
            { if(s==SLEFT) return(0.85*0.05); else if(s==SRIGHT) return(0.15*0.05);}
        case GROWLRIGHT_CREAKLEFT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
            { if(s==SLEFT) return(0.15*0.9); else if(s==SRIGHT) return(0.85*0.9);}
        case GROWLRIGHT_CREAKRIGHT:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
            { if(s==SLEFT) return(0.15*0.9); else if(s==SRIGHT) return(0.85*0.9);}
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
        case GROWLRIGHT_SILENCE:
            if(ja==GetJointActionIndex(LISTEN,LISTEN))
            { if(s==SLEFT) return(0.15*0.9); else if(s==SRIGHT) return(0.85*0.9);}
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT))
                return(1.0/6);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN))
            { if(s==SLEFT) return(0.15*0.05); else if(s==SRIGHT) return(0.85*0.05);}
        }
    }

    throw(E("ProblemDecTigerWithCreaks::GetObsProb should never get here"));
    return(42);
}

void ProblemDecTigerWithCreaks::FillObservationModel()
{
    for(Index jo=0; jo<GetNrJointObservations();jo++) 
    {
        vector<Index> oIs=JointToIndividualObservationIndices(jo);
        for(Index  ja=0; ja< GetNrJointActions(); ja++)
            for(Index s1=0; s1<NUMBER_OF_STATES;s1++) 
            {
                double prob = GetObsProb(ja,s1,0,oIs[0]) *
                    GetObsProb(ja,s1,1,oIs[1]);
                SetObservationProbability(ja, s1, jo, prob);
            }
    }
}

double ProblemDecTigerWithCreaks::GetRewardForAgent(Index s,
                                                    Index ja,
                                                    Index agentI) const
{
    switch(agentI)
    {
    case 0:
        switch(s)
        {
        case SLEFT:
        {
            if     (ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT)) return(10);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT)) return(-100);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT)) return(10);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT)) return(-100);
            else if(ja==GetJointActionIndex(LISTEN,LISTEN)) return(-1);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT)) return(-1);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN)) return(10);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT)) return(-1);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN)) return(-100);
        }
        case SRIGHT:
        {
            if     (ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT)) return(-100);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT)) return(10);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT)) return(-100);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT)) return(10);
            else if(ja==GetJointActionIndex(LISTEN,LISTEN)) return(-1);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT)) return(-1);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN)) return(-100);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT)) return(-1);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN)) return(10);
        }
        }
    case 1:
        switch(s)
        {
        case SLEFT:
        {
            if     (ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT)) return(10);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT)) return(-100);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT)) return(-100);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT)) return(10);
            else if(ja==GetJointActionIndex(LISTEN,LISTEN)) return(-1);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT)) return(10);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN)) return(-1);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT)) return(-100);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN)) return(-1);
        }
        case SRIGHT:
        {
            if     (ja==GetJointActionIndex(OPENRIGHT,OPENRIGHT)) return(-100);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENLEFT)) return(10);
            else if(ja==GetJointActionIndex(OPENRIGHT,OPENLEFT)) return(10);
            else if(ja==GetJointActionIndex(OPENLEFT,OPENRIGHT)) return(-100);
            else if(ja==GetJointActionIndex(LISTEN,LISTEN)) return(-1);
            else if(ja==GetJointActionIndex(LISTEN,OPENRIGHT)) return(-100);
            else if(ja==GetJointActionIndex(OPENRIGHT,LISTEN)) return(-1);
            else if(ja==GetJointActionIndex(LISTEN,OPENLEFT)) return(10);
            else if(ja==GetJointActionIndex(OPENLEFT,LISTEN)) return(-1);
        }
        }
    }

    throw(E("ProblemDecTigerWithCreaks::GetRewardForAgent should never get here"));
    return(42);
}

void ProblemDecTigerWithCreaks::FillRewardModel()
{
    // the problem has different rewards for each agent, so to make it
    // a Dec-POMDP we sum the rewards of both agents
    for(Index  ja=0; ja< GetNrJointActions(); ja++)
        for(Index s=0; s<NUMBER_OF_STATES;s++)
            SetReward(s,ja,(GetRewardForAgent(s,ja,0)+
                            GetRewardForAgent(s,ja,1)));
}

