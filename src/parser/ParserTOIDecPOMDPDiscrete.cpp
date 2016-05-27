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

#include "ParserTOIDecPOMDPDiscrete.h"
#include "ParserDPOMDPFormat_Spirit.h"
#include <fstream>

using namespace std;

ParserTOIDecPOMDPDiscrete::
ParserTOIDecPOMDPDiscrete(TOIDecPOMDPDiscrete
                          *problem) :
    _m_problem(problem)
{
}

void ParserTOIDecPOMDPDiscrete::Parse()
{
    ParseBase();

    for(Index i=0;i!=_m_problem->GetNrAgents();++i)
        ParseAgent(i);

    ParseRewards();

    _m_problem->SetInitialized(true);
}


void ParserTOIDecPOMDPDiscrete::ParseRewards()
{
    vector<size_t> nrStates(_m_problem->GetNrAgents(),0),
        nrActions(_m_problem->GetNrAgents(),0);
    for(Index i=0;i!=_m_problem->GetNrAgents();++i)
    {
        nrStates[i]=_m_problem->GetIndividualMADPD(i)->GetNrStates();
        nrActions[i]=_m_problem->GetIndividualMADPD(i)->GetNrJointActions();
    }
    ParseRewards(_m_problem->GetNrAgents(),nrStates,nrActions);
}

void ParserTOIDecPOMDPDiscrete::ParseBase()
{
    unsigned int nrAgents;
    double gamma;

    string filename=_m_problem->GetProblemFile() + ".base";
    ifstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "ParserTOIDecPOMDPDiscrete::ParseBase: failed to "
             << "open file " << filename << endl;            
        abort();
    }

    int line=0;
    string buffer;
    while(!getline(fp,buffer).eof())
    {
        istringstream is(buffer);
        switch(line)
        {
        case 0:
            is >> nrAgents;
            break;
        case 1:
            is >> gamma;
            break;
        }
        line++;
    }

    _m_problem->SetNrAgents(nrAgents);
    _m_problem->SetDiscount(gamma);
}

void ParserTOIDecPOMDPDiscrete::ParseAgent(Index id)
{
    MultiAgentDecisionProcessDiscrete* madp = 
        _m_problem->GetIndividualMADPD(id);

    DecPOMDPDiscrete *decpomdp;
    decpomdp=new DecPOMDPDiscrete("", "", madp->GetProblemFile());
    decpomdp->SetSparse(madp->GetSparse());

    DPOMDPFormatParsing::ParserDPOMDPFormat_Spirit parser(decpomdp);
    parser.Parse();

    if(decpomdp->GetNrAgents()!=1)
        throw(E("ParserTOIDecPOMDPDiscrete::ParseAgent individual models can only be defined for a single agent"));

    StoreDecPOMDP(decpomdp,id);
}

void ParserTOIDecPOMDPDiscrete::StoreDecPOMDP(DecPOMDPDiscrete *decpomdp,
                                              Index id)
{
    MultiAgentDecisionProcessDiscrete* madp = 
        _m_problem->GetIndividualMADPD(id);
    decpomdp->ExtractMADPDiscrete(madp);
    _m_problem->SetIndividualDecPOMDPD(decpomdp,id);
}

/** Format for .rewards file is a set of lines:
 * <s_0 ... s_n> <a_0 ... a_n> <reward>
 */
void ParserTOIDecPOMDPDiscrete::ParseRewards(size_t nrAgents,
                                             const vector<size_t> &nrStates,
                                             const vector<size_t> &nrActions)
{
    vector<unsigned int> states, actions;
    unsigned int state,action;
    double reward;
    
    string filename=_m_problem->GetProblemFile() + ".rewards";
    ifstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "ParserTOIDecPOMDPDiscrete::ParseRewards: failed to "
             << "open file " << filename << endl;            
    }

    _m_problem->CreateNewRewardModel();

    string buffer;
    while(!getline(fp,buffer).eof())
    {
        istringstream is(buffer);
        states.clear();
        actions.clear();
        for(unsigned int i=0;i!=nrAgents;++i)
        {
            is >> state;
            states.push_back(state);
        }
        for(unsigned int i=0;i!=nrAgents;++i)
        {
            is >> action;
            actions.push_back(action);
        }
        is >> reward;

        _m_problem->
            SetReward(states, actions, reward);
    }
}
