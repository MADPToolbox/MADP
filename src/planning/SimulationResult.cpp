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

#include "SimulationResult.h"
#include <float.h>
#include <fstream>

using namespace std;

//Default constructor
SimulationResult::SimulationResult()
{
    _m_nr_stored=0;
    _m_avg_reward=-1;
}

/** 
 * @param horizon planning horizon
 * @param random_seed seed used for sampling
 * @param nrRuns number of runs simulated
 */
SimulationResult::SimulationResult(int horizon,int random_seed,int nrRuns)
{
    _m_horizon=horizon;
    _m_random_seed=random_seed;
    _m_rewards = vector<double>(nrRuns, 0.0);
    _m_nr_stored=0;
    _m_avg_reward=-1;
}

//Destructor
SimulationResult::~SimulationResult()
{
}

void SimulationResult::AddReward(double r)
{
    _m_nr_stored++;
    _m_rewards[_m_nr_stored-1]=r;

    UpdateStatistics();
}

double SimulationResult::GetReward(Index i) const
{
    return(_m_rewards.at(i));
}

vector<double> SimulationResult::GetRewards(void)
{
    vector<double> rewards(_m_nr_stored);

    for(unsigned int i=0;i<_m_nr_stored;i++)
        rewards[i]=_m_rewards[i];
    
    return(rewards);
}

void SimulationResult::UpdateStatistics()
{ 
    // update the average reward
    double sum=0;
    for(unsigned int i=0;i<_m_nr_stored;i++)
        sum+=_m_rewards[i];

    if(_m_nr_stored>0)
        _m_avg_reward=sum/_m_nr_stored;
    else
        _m_avg_reward=DBL_MAX;
}

void SimulationResult::Print(void)
{
    if(_m_nr_stored>_m_rewards.size())
        cerr << "SimulationResult::Print error _m_nr_stored " << _m_nr_stored  
             << " > " << " _m_rewards.size() " << _m_rewards.size() << endl;

    cout << "SimulationResult::Print horizon " << _m_horizon << " seed "
         << _m_random_seed << " entries " << _m_nr_stored << endl;
    cout << "SimulationResult::Print Rewards: ";
    for(unsigned int i=0;i<_m_nr_stored;i++)
        cout << _m_rewards[i] << " ";
    cout << endl;
    cout << "SimulationResult::Print Average reward " << _m_avg_reward << endl;
}

void SimulationResult::PrintSummary(void)
{
    cout << "Average reward: " << _m_avg_reward << " ("
         << _m_nr_stored << " samples)" << endl;
}

void SimulationResult::Save(string filename)
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "SimulationResult::Save failed to open file " << filename << endl;
        throw E(ss);
    }
   
    vector<double> rewards=GetRewards();

    for(unsigned i=0;i<rewards.size();i++)
        fp << rewards[i] << endl;
}

void SimulationResult::Load(string filename)
{
    double r;

    ifstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "SimulationResult::Load: failed to "
           << "open file " << filename;
        throw E(ss);
    }

    _m_rewards.clear();

    string buffer;
    while(!getline(fp,buffer).eof())
    {
        istringstream is(buffer);
        is >> r;
        _m_rewards.push_back(r);
    }

    _m_nr_stored=_m_rewards.size();
    UpdateStatistics();
}
