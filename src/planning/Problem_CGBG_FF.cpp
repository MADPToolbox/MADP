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

#include "Problem_CGBG_FF.h"
#include "BayesianGameIdenticalPayoff.h"
#include <set>
#include "FixedCapacityPriorityQueue.h"
#include <fstream>

using namespace std;

#define DEBUGUTILS 0
#define DEBUGPROBS 0
#define DEBUGASSIGN 1
#define DEBUGADDEDGE 0

#define USE_FLAMES_NOFLAMES_OBS 0

//Default constructor
Problem_CGBG_FF::Problem_CGBG_FF(
                size_t nrHouses,
                size_t nrFLs,
                size_t nrAgents,
                size_t nrActionsPerAgent,       //i.e., z
                size_t nrObservedHousesPerAgent,//i.e., y
                size_t k                        //the maximum number of agents in a house's payoff function

    )
    :
        BayesianGameCollaborativeGraphical(
                nrAgents, 
                vector<size_t>(nrAgents, nrActionsPerAgent),
#if USE_FLAMES_NOFLAMES_OBS
                vector<size_t>(nrAgents, pow(2, nrObservedHousesPerAgent))
#else
                vector<size_t>(nrAgents, pow(nrFLs, nrObservedHousesPerAgent))
#endif
                )
        ,_m_allAgents()
        ,_m_nrHouses(nrHouses)
        ,_m_nrFireLevels(nrFLs)
#if USE_FLAMES_NOFLAMES_OBS
        ,_m_nrObsPerHouse(2)
#else
        ,_m_nrObsPerHouse(nrFLs) // each firelevel is a type
#endif
        ,_m_nrActionsPerAgent(nrActionsPerAgent)
        ,_m_nrObservedHousesPerAgents(nrObservedHousesPerAgent)
        ,_m_k(k)
        //this means that there is no limit on how many agents can observe a house:
        //,_m_maxNrAgentsObservingAHouse(nrAgents)
        ,_m_maxNrAgentsObservingAHouse(k)//<-this means that there is no limit on how many agents can observe a house
        ,_m_Norm_cache()
        ,_m_agentsForHouse_action(nrHouses, Scope())
        ,_m_agentsForHouse_obs(nrHouses, Scope())
        ,_m_houseIndices_obs(nrAgents, vector<Index>() )
        ,_m_houseIndices_action(nrAgents, vector<Index>() )
        ,_m_housePositionX(nrHouses, -1.0)
        ,_m_housePositionY(nrHouses, -1.0)
        ,_m_agentPositionX(nrAgents, -1.0)
        ,_m_agentPositionY(nrAgents, -1.0)


{
    //cout << "Problem_CGBG_FF::constructor...\n";
    //cout << "nrFLs=" << nrFLs;
    //cout << "\nnrObservedHousesPerAgent="<<nrObservedHousesPerAgent;
    //cout << "\nnr types:"<< SoftPrintVector( this->GetNrTypes() ) << endl;
    for(Index agI=0; agI < GetNrAgents(); agI++)
        _m_allAgents.Insert(agI);
    ScatterHouses();
    ScatterAgents();
    AssignAgentActionsToHouses();
    AssignAgentTypesToHouses();

    //print summarization of assignment:
    cout << "-------------------"<<endl;
    cout << "Assignment summary:"<<endl;
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        cout << "Agent "<<agI<<"...\n\tcan fight fire at houses:"<<
        SoftPrintVector(_m_houseIndices_action.at(agI)) <<
        "\n\tand can observe houses:" <<
        SoftPrintVector(_m_houseIndices_obs.at(agI)) << endl;
    }
#if DEBUGASSIGN
    for(Index hI=0; hI < _m_nrHouses; hI++)
    {
        cout << "House "<<hI<<"...\n\tcan be visited by agents :"<<
        _m_agentsForHouse_action.at(hI) <<
        "\n\tand by observed by agents:" <<
        _m_agentsForHouse_obs.at(hI) << endl;
    }
#endif
    cout << "-------------------"<<endl;


    //can we do an automatic reduction? I.e., when same scopes, the 
    //edges are combined ? 
    //Would that be something we would want? (perhaps not? the number of edges becomes variable)
    for(Index hI=0; hI < _m_nrHouses; hI++)
        AddEdge(hI);

}
//Copy constructor.    
//Problem_CGBG_FF::Problem_CGBG_FF(const Problem_CGBG_FF& o) 
//{
//}
//Destructor
Problem_CGBG_FF::~Problem_CGBG_FF()
{
}
//Copy assignment operator
//Problem_CGBG_FF& Problem_CGBG_FF::operator= (const Problem_CGBG_FF& o)
//{
    //if (this == &o) return *this;   // Gracefully handle self assignment
    //// Put the normal assignment duties here...

    //return *this;
//}

namespace{
double SampleUniform()
{
    double r = (rand() / (double)RAND_MAX);
    return r;

}
}

void Problem_CGBG_FF::ScatterHouses()
{
    for(Index hI=0; hI < _m_nrHouses; hI++)
    {
        //we sample a position for this house
        _m_housePositionX[hI] = SampleUniform();
        _m_housePositionY[hI] = SampleUniform();
    }
}
void Problem_CGBG_FF::ScatterAgents()
{
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        //we sample a position for this agent
        _m_agentPositionX[agI] = SampleUniform();
        _m_agentPositionY[agI] = SampleUniform();
    }
}

//the following function assigns z=nrActionsPerAgent houses to each agent
//in principle it are the nearest houses, but if some houses already
//have k agents assoicated with them, the house becomes unavailable for
//further agents
void Problem_CGBG_FF::AssignAgentActionsToHouses()
{
#if DEBUGASSIGN
    cout << ">>Assigning action houses to agents<<"<<endl;
#endif
    set<Index> availableHouses;
    for(Index hI=0; hI < _m_nrHouses; hI++)
        availableHouses.insert(hI);

    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        FixedCapacityPriorityQueue< diPair, diPairComp > pq( _m_nrActionsPerAgent );
        for(set<Index>::iterator it=availableHouses.begin(); 
                it != availableHouses.end(); it++)            
        {
            Index hI = *it;
            double d = ComputeDistanceAgentToHouse(agI, hI);
            diPair p(d, hI);
            diPair overflown;
            pq.insert( p, overflown );
        }
        while(! pq.empty())
        {
            diPair p = pq.top();
            Index hI = p.second;
#if DEBUGASSIGN
            double d = p.first;
            cout << "Assigning house "<<hI<<" to agent " << agI <<
                " it has distance " << d << endl;
#endif
            _m_agentsForHouse_action.at(hI).Insert(agI);
            _m_houseIndices_action.at(agI).push_back(hI);
            if( _m_agentsForHouse_action.at(hI).size() == _m_k )
            {
#if DEBUGASSIGN
                cout << "House  "<<hI<<" is at full 'capacity' (it has k agents in its scope), removing it from further consideration\n";
#endif
                availableHouses.erase(hI);
            }
            pq.pop();
        }
        //check how many actions the agent actually got (could be less than _m_nrActionsPerAgent!)
        if (_m_houseIndices_action.at(agI).size() <  _m_nrActionsPerAgent)
        {
            size_t new_nr_actions =  _m_houseIndices_action.at(agI).size();            
            if(new_nr_actions == 0)
            {
                stringstream ss;
                ss<<"AssignAgentActionsToHouses - Error agent "<<agI<< " has 0 actions!";
                throw E(ss);
            }
            ChangeNrActions(agI, new_nr_actions);
        }
    }

    
}

void Problem_CGBG_FF::AssignAgentTypesToHouses()
{
#define TYPES_INDEP_OF_ACTIONS 0 
#if TYPES_INDEP_OF_ACTIONS     
#if DEBUGASSIGN
    cout << ">>Assigning observation houses to agents --- independent of actions <<"<<endl;
#endif
    set<Index> availableHouses;
    for(Index hI=0; hI < _m_nrHouses; hI++)
        availableHouses.insert(hI);

    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        FixedCapacityPriorityQueue< diPair, diPairComp > pq( _m_nrObservedHousesPerAgents );
        for(set<Index>::iterator it=availableHouses.begin(); 
                it != availableHouses.end(); it++)            
        {
            Index hI = *it;
            double d = ComputeDistanceAgentToHouse(agI, hI);
            diPair p(d, hI);
            diPair overflown;
            pq.insert( p, overflown );
        }
        while(! pq.empty())
        {
            diPair p = pq.top();
            Index hI = p.second;
#if DEBUGASSIGN
            double d = p.first;
            cout << "Assigning house "<<hI<<" to agent " << agI <<
                " it has distance " << d << endl;
#endif
            _m_agentsForHouse_obs.at(hI).Insert(agI);
            _m_houseIndices_obs.at(agI).push_back(hI);
            if( _m_agentsForHouse_obs.at(hI).size() == _m_maxNrAgentsObservingAHouse )
            {
#if DEBUGASSIGN
                cout << "House  "<<hI<<" is at full 'capacity' (it has k agents in its scope), removing it from further consideration\n";
#endif
                availableHouses.erase(hI);
            }
            pq.pop();
        }
        //check how many observer house the agent actually got (could be less than _m_nrObservedHousesPerAgents!)
        if (_m_houseIndices_obs.at(agI).size() <  _m_nrObservedHousesPerAgents)
        {
            size_t nrTypes =  pow(_m_nrObsPerHouse, _m_houseIndices_obs.at(agI).size());
            ChangeNrTypes(agI, nrTypes);
        }
    }
#else // if not TYPES_INDEP_OF_ACTIONS     
#if DEBUGASSIGN
    cout << ">>Assigning observation houses to agents --- only allowing 'action houses' <<"<<endl;
#endif
    //set<Index> availableHouses;
    //for(Index hI=0; hI < _m_nrHouses; hI++)
        //availableHouses.insert(hI);

    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        const std::vector<Index> & houses_agI = _m_houseIndices_action.at(agI);
        for(Index i = 0; i < houses_agI.size(); i++)
        {
            if (_m_houseIndices_obs.at(agI).size() == _m_nrObservedHousesPerAgents)     
            {
                //agent has reached the maximum number of house, no point in looping to remaining houses
                i =  houses_agI.size();
                continue;
            }

            Index hI = houses_agI.at(i); //<-the candidate house index
            //assign this house if it is not at capacity yet
            if  (_m_agentsForHouse_obs.at(hI).size() < _m_maxNrAgentsObservingAHouse)   //this house is not at full capacity?
            {
                _m_agentsForHouse_obs.at(hI).Insert(agI);
                _m_houseIndices_obs.at(agI).push_back(hI);
#if DEBUGASSIGN
                cout << "Assigning house "<<hI<<" to agent " << agI << endl;
#endif            
            }
            else
#if DEBUGASSIGN
                cout << "House  "<<hI<<" is at full 'capacity' (it has k agents in its scope), removing it from further consideration\n";
#endif
            }

/*        
        FixedCapacityPriorityQueue< diPair, diPairComp > pq( _m_nrObservedHousesPerAgents );
        for(set<Index>::iterator it=availableHouses.begin(); 
                it != availableHouses.end(); it++)            
        {
            Index hI = *it;
            double d = ComputeDistanceAgentToHouse(agI, hI);
            diPair p(d, hI);
            diPair overflown;
            pq.insert( p, overflown );
        }
        while(! pq.empty())
        {
            diPair p = pq.top();
            Index hI = p.second;
#if DEBUGASSIGN
            double d = p.first;
            cout << "Assigning house "<<hI<<" to agent " << agI <<
                " it has distance " << d << endl;
#endif
            _m_agentsForHouse_obs.at(hI).Insert(agI);
            _m_houseIndices_obs.at(agI).push_back(hI);
            if( _m_agentsForHouse_obs.at(hI).size() == _m_maxNrAgentsObservingAHouse )
            {
#if DEBUGASSIGN
                cout << "House  "<<hI<<" is at full 'capacity' (it has k agents in its scope), removing it from further consideration\n";
#endif
                availableHouses.erase(hI);
            }
            pq.pop();
        }
*/        
        //check how many observed houses the agent actually got (could be less than _m_nrObservedHousesPerAgents!)
        if (_m_houseIndices_obs.at(agI).size() <  _m_nrObservedHousesPerAgents)
        {
            size_t nrTypes =  pow(_m_nrObsPerHouse, _m_houseIndices_obs.at(agI).size());
            ChangeNrTypes(agI, nrTypes);
        }
    }
#endif // ends if TYPES_INDEP_OF_ACTIONS     
}

double Problem_CGBG_FF::ComputeDistanceAgentToHouse(Index agI, Index hI)
{
    double d = 
        pow(_m_housePositionX.at(hI) - _m_agentPositionX.at(agI), 2) +
        pow(_m_housePositionY.at(hI) - _m_agentPositionY.at(agI), 2);
    d = sqrt(d);
    //cout << "agent "<< agI<< ", house " << hI <<" -> d=" << d << endl;
    return d;    
}

void Problem_CGBG_FF::AddEdge(Index hI)
{
    const Scope & tupleOfAgents = _m_agentsForHouse_action.at(hI);
#if DEBUGADDEDGE     
    cout << "-------------------------------------"<<endl;
    cout << "Adding payoff component for house "<< hI  << endl
        << "\tscope="<< tupleOfAgents << endl;
    cout << "nr types:"<< SoftPrintVector( this->GetNrTypes() ) << endl;
#endif
    AddLRF(tupleOfAgents);
    //Index e=GetNrLRFs()-1;
    Index e=hI;
    size_t k = tupleOfAgents.size();
    vector<Index> actions_e(k, 0);
    const vector<size_t>& nr_actions = GetBGIPforLRF(e)->GetNrActions();
    vector<Index> types_e(k, 0);
    const vector<size_t>& nr_types = GetBGIPforLRF(e)->GetNrTypes();

#if DEBUGADDEDGE     
    cout<< "\tnr_actions=" << PrintTools::SoftPrintVector(nr_actions) << endl
        << "\tnr_types=" << PrintTools::SoftPrintVector(nr_types) << endl;
    cout << "-------------------------------------"<<endl;
#endif
    double psum = 0.0;
    do{
#if DEBUGPROBS        
        cout << "Computing local type prob. P(" << PrintTools::SoftPrintVector(types_e) << ")..."<<endl;
#endif
        double p = ComputeLocalProbability(tupleOfAgents, types_e);
#if DEBUGPROBS        
        cout << "...P(" << PrintTools::SoftPrintVector(types_e) << ")="<<p<<endl;
#endif
        psum += p;
        SetProbability(e, types_e, p);
    }while(!IndexTools::Increment(types_e, nr_types));
    if(! (abs(psum-1.0)<1e-6))
    {
        stringstream ss;
        ss << "Joint type probabilities for house "<<hI<<" do not sum to 1.0, but to " 
            << psum << ", a difference of " << abs(psum-1) << "!";
        throw E(ss);
    }
    do{
        do{
            SetUtility(e, types_e, actions_e, ComputeLocalUtility(hI, tupleOfAgents,  actions_e, types_e));
        }while(!IndexTools::Increment(types_e, nr_types));
    }while(!IndexTools::Increment(actions_e, nr_actions));
}

vector<Index> Problem_CGBG_FF::TypeIndexToObservationIndices(
        Index agI, Index typeI) const
{
    const vector<Index> & obsHouseIndices = _m_houseIndices_obs.at(agI);
    vector<Index> obsIndices(obsHouseIndices.size());
    TypeIndexToObservationIndices(typeI, obsIndices);
    return obsIndices;
}

void Problem_CGBG_FF::TypeIndexToObservationIndices(Index typeI, vector<Index> & obsIndices) const
{
    size_t nrObs = obsIndices.size();
    vector<size_t> nrElems(nrObs, _m_nrObsPerHouse);
    //TODO, check if cached, cache
    IndexTools::JointToIndividualIndices(typeI, nrElems, obsIndices); 
    return;
}



vector<Problem_CGBG_FF::observation_t> Problem_CGBG_FF::FilterObservationsForHouse(
        Index hI,  
        const Scope& tupleOfAgents,
        const vector<Index>& types
    ) const
{
    vector<Problem_CGBG_FF::observation_t> allObservationsForHouse_hI;

    for(Index i=0; i < tupleOfAgents.size(); i++)
    {
        Index agI = tupleOfAgents[i];
        Index typeI = types.at(i);
        //vector<Index> obsIndices = TypeIndexToObservationIndices(agI, typeAgI);
        const vector<Index> & obsHouseIndices = _m_houseIndices_obs.at(agI);
        vector<Index> obsIndices(obsHouseIndices.size());
        TypeIndexToObservationIndices(typeI, obsIndices);
#if DEBUGPROBS
        cout << "\tAgent " << agI << ", observations: " << SoftPrintVector(obsIndices) <<
            " for houses:" << SoftPrintVector(obsHouseIndices) << " (typeI="<<typeI<<")"<<endl;
#endif
 
        for(Index o=0; o < obsIndices.size(); o++)
            if(obsHouseIndices.at(o) == hI)
                allObservationsForHouse_hI.push_back(
                    (Problem_CGBG_FF::observation_t) obsIndices.at(o)
                    );
    }
    return allObservationsForHouse_hI;
}



        

/* this is WRONG!
  
   it assumes that types of different agents are independent, but they are not!
   (they are independent *given the state*, but that is already marginalized out
   in the below approach)

double p = 1.0;
for(Index i=0; i < tupleOfAgents.Size(); i++)
{
    Index agI = tupleOfAgents.at(i);
    Index typeI = types.at(i);
    p *= ComputeTypeProb(agI, typeI)
}
*/
double Problem_CGBG_FF::ComputeLocalProbability( 
        const Scope& tupleOfAgents,
        const vector<Index>& types
        ) const
{

    /* Let
        - si denote the fire level of house i
        - hi denote the observation of house i (by some agent left unspecified here)

       Given that 
        1) the fire levels at different houses are independent P(s)=P(s1)*...*P(sN),
        2) the observation made of different houses are independent: P(h1h2|s)=P(h1|s1)P(h2|s2)
        the following should work.
      
       assume 2 agents that observe 2 houses each:
        -agent 1 gets observations of houses 1 and 2: t1=<h1,h2>
        -agent 2 gets observations of houses 2 and 3: t2=<h2,h3>
       
       Then P(t1,t2)
        = \sum_s P(t1|s)P(t2|s)P(s)
        = \sum_{s1} \sum_{s2} \sum_{s3} [ P(h1|s1)P(h2|s2) ] * [ P(h2|s2)P(h3|s3) ] * P(s1)P(s2)P(s3)
        = [sum_{s1} P(h1|s1) P(s1)] * [sum_{s2} P(h2|s2)P(h2|s2)P(s2)] * [sum_{s3} P(h3|s3) P(s3)]

       So we can compute the probability of a joint type, as the product of a term for each house
       in the `observation scope':
          ComputeHouseProbability(h1, observations_for_h1) * 
          ComputeHouseProbability(h2, observations_for_h2) * 
          ComputeHouseProbability(h3, observations_for_h3)  
    */

    //first we detemine the houses that are 'in scope'
    std::set<Index> house_scope;
    for(Index i=0; i < tupleOfAgents.size(); i++)
    {
        Index agI = tupleOfAgents.at(i);
        const vector<Index> & obsHouseIndices = _m_houseIndices_obs.at(agI);
        house_scope.insert(obsHouseIndices.begin(), obsHouseIndices.end());
#if DEBUGPROBS
        cout << "Agent " << agI << " observes houses " << PrintTools::SoftPrintVector(obsHouseIndices) << endl;
#endif
    }
#if DEBUGPROBS
    cout << "'Combined houses scope' = " << PrintTools::SoftPrintSet(house_scope) << endl;
    cout << "Computing probability term (e.g. [sum_{s2} P(h2|s2)P(h2|s2)P(s2)] ) for each house..."<<endl;
#endif
    //for each house:
    std::set<Index>::iterator it =  house_scope.begin();
    std::set<Index>::iterator last =  house_scope.end();
    double p = 1.0;
    while (it != last)
    {
        Index hI = *it;
#if DEBUGPROBS
        cout << "house " << hI << "..."<<endl;
#endif
        double phI = ComputeHouseProbability(hI, tupleOfAgents, types);
#if DEBUGPROBS
        cout << "...house " << hI << " probability term, P(h_obs,h_obs) = " << phI << endl;
#endif
        p *= phI;
        it++;
    }
    return p;
}

//computes the probability of the observations in oVec_hI made about house houseI.
//
//That is, it computes the 
// sum_{s2} P(h2|s2)P(h2|s2)P(s2) = 
// sum_{s2} P(h2,h2,s2) = P(h2,h2)
//
//  (where the first prob. is the prob. of the first agent making the observation h2,
//  and the second term is the probability of the second agent. See comments
//  at ComputeLocalProbability )
//
//etc. in the above derivation. (see ComputeLocalProbability)
double Problem_CGBG_FF::ComputeHouseProbability(Index houseI,  
        const Scope& tupleOfAgents,
        const vector<Index>& types
        //const vector<Problem_CGBG_FF::observation_t>& oVec_hI
    ) const
{
#if DEBUGPROBS
    cout << "\tComputeHouseProbability for house "<<houseI
         << " tupleOfAgents=" << tupleOfAgents 
         << " types=" << PrintTools::SoftPrintVector(types)<<endl;
#endif
    vector<Problem_CGBG_FF::observation_t> oVec_hI = 
        FilterObservationsForHouse(houseI, tupleOfAgents, types);
#if DEBUGPROBS
    cout << "\tThe observation vector for this house is " << 
        PrintTools::SoftPrintVector(oVec_hI)<<endl;
#endif

    double p = 0.0;
    for(Index fl=0; fl < _m_nrFireLevels; fl++)
    {
        double p_oVec_fl = Likelihood(houseI, oVec_hI, fl) * Prior(houseI, fl);
#if DEBUGPROBS
        cout << "\tP(oVec, FL="<<fl<<") = " << p_oVec_fl << endl;
#endif
        p += p_oVec_fl;
    }
#if DEBUGPROBS
    cout << "P(oVec)="<< p << endl;
#endif
    return p;
}

double Problem_CGBG_FF::FLObservationProb(Index fireLevel, Problem_CGBG_FF::observation_t obs) const
{
    double p_this_obs = 0;

    enum oldObservation_t { NOFLAMES, FLAMES  }; //<- this leads to better binary interpretation than reversed.

    if(_m_nrObsPerHouse==2)
    {
        double pFlames = 0.0;
        switch(fireLevel)
        {
        case(0): //no fire
            pFlames = 0.2; // 0.2 prob. of incorrectly observing
            break;
        case(1):
            pFlames = 0.5;
            break;
        default:
            pFlames = 0.8;
        }
        double pNoFlames = 1.0-pFlames;                 
        switch(obs)
        {
        case(FLAMES):
            p_this_obs = pFlames;
            break;
        case(NOFLAMES):
            p_this_obs = pNoFlames;
            break;
        }
    }
    else if(_m_nrObsPerHouse==_m_nrFireLevels)
    {
        if(fireLevel==0)
        {
            switch(obs)
            {
            case 0:
                p_this_obs=0.8;
                break;
            case 1:
                p_this_obs=0.2;
                break;
            default:
                p_this_obs=0;
            }
        }
        else if(fireLevel==_m_nrFireLevels-1)
        {
            if(obs==_m_nrFireLevels-1)
                p_this_obs=0.8;
            else if(obs==_m_nrFireLevels-2)
                p_this_obs=0.2;
            else
                p_this_obs=0;
        }
        else
        {
            if(obs==fireLevel-1)
                p_this_obs=0.2;
            else if(obs==fireLevel)
                p_this_obs=0.6;
            else if(obs==fireLevel+1)
                p_this_obs=0.2;
            else
                p_this_obs=0;
        }
    }
    else
        throw(E("FLObservationProb nyi"));

    return p_this_obs;
}

//currently this ignores the houseI 
double Problem_CGBG_FF::Likelihood(Index houseI, 
        const vector<Problem_CGBG_FF::observation_t>& oVec_hI, Index fireLevel) const
{
    double p = 1.0;
    for(Index i=0; i < oVec_hI.size(); i++)
    {
        Problem_CGBG_FF::observation_t obs = oVec_hI.at(i);
        double p_this_obs = FLObservationProb(fireLevel, obs);

        //cout << "Likelihood: P(obs|FL)="<<p_this_obs<<" for obs, " << obs <<", FL="<<fireLevel<<endl;
        p *= p_this_obs;
    }
    return p;
}

//currently this ignores the houseI 
double Problem_CGBG_FF::Prior(Index houseI, Index fireLevel) const
{
    return 1.0 / _m_nrFireLevels;
}




/* in the light of the above error, this will not be useful at all...
double Problem_CGBG_FF::ComputeTypeProb( Index agI, Index typeI )
{
    //-convert typeI to <o_1,...,o_k> the observations for the different houses this agent can observe
    vector<Index> & obsHouseIndices = _m_houseIndices_obs.at(agI);
    vector<Index> obsIndices(houseIndices_obs.size());
    TypeIndexToObservationIndices(typeI, obsIndices);
    
    double p = 1.0;
    for(Index i=0; i < obsIndices.size(); i++)
    {
        //ASSUMPTION: observations of different houses by the same agent are independent
        p *= 
    }
    //-p=1 
    //-for each house h, 
    //      p *= P(o_h)
    //-return p
    TODO
}
*/

size_t Problem_CGBG_FF::ComputeNumberOfAgentsPresentAtHouse(Index houseI, const vector<Index>& a_vec) const
{    
    size_t nrAgentsPresent = 0;
    const vector<Index> & tupleOfAgents = _m_agentsForHouse_action.at(houseI);
    for(Index i=0; i < tupleOfAgents.size(); i++)
    {
        Index agI = tupleOfAgents.at(i); //<-agent index
        Index acI = a_vec.at(i);      //<-its action

        const vector<Index>& hs = _m_houseIndices_action.at(agI);
        //cout << "agI="<<agI<<" hs=" << PrintTools::SoftPrintVector(hs)<<endl;
        Index h = hs.at(acI);

        if(h==houseI)
            nrAgentsPresent++;
    }
    //cout << "nrAgentsPresent="<<nrAgentsPresent<<endl;
    return nrAgentsPresent;

}

double Problem_CGBG_FF::ComputeLocalUtility( 
        Index hI, 
        const Scope& tupleOfAgents,
        const vector<Index>& actions,
        const vector<Index>& types
        )
{
#if DEBUGUTILS    
    cout << "ComputeLocalUtility( " 
         << "hI=" << hI 
         << ", tupleOfAgents=" << tupleOfAgents
         << ", actions=" << PrintTools::SoftPrintVector(actions)
         << ", types=" << PrintTools::SoftPrintVector(types)
         << endl;
#endif

    size_t nrAgentsPresent = ComputeNumberOfAgentsPresentAtHouse(hI, actions);
    
    //compute the probabilities over the firelevels of this house, given the joint type
    //and compute 
    //      U^e(jt_e,ja_e) = \sum_s_e P(s_e|jt_e) R^e(s_e,ja_e)
    //(where e corresponds to this house hI)
    //
    //However, in this problem we have some simplying assumptions.
    //E.g., extending the example introduced above, for house 2, we have:
    //      U2(t1t2,a1a2) = \sum_s2 P(s2|t1t2) R(s2|a1a2)
    //
    //Here of course  P(s2|t1t2) =  P(s2,t1t2)/ P(t1t2)
    //
    //However <t1,t2> specify multiple observations, only some of which pertain to house 2,
    //let's rewrite <t1t2> = <h21,h22,hx1,hx2> 
    //(where hxi are to observations of agent i about different houses)
    //
    //  P(s2|h21,h22,hx1,hx2) = P(h21,h22,hx1,hx2|s2) P(s2) / P( h21,h22,hx1,hx2 )
    //                        = P(h21|s2)P(h22|s2)P(s2)P(hx1)P(hx2) / (P(hx1)P(hx2)*\sum_{s2} P(h21|s2)P(h22|s2)P(s2) )
    //                        = P(h21|s2)P(h22|s2)P(s2) / \sum_{s2} P(h21|s2)P(h22|s2)P(s2) 
    //                        = P(s2|h21,h22)
    //

    vector<Problem_CGBG_FF::observation_t> oVec_hI = 
        FilterObservationsForHouse(hI, tupleOfAgents, types);

    double p_oVec = 0.0;
    vector<double> p_oVec_fl(_m_nrFireLevels);
    for(Index fl=0; fl < _m_nrFireLevels; fl++)
    {
        p_oVec_fl.at(fl) = Likelihood(hI, oVec_hI, fl) * Prior(hI, fl);
        p_oVec += p_oVec_fl.at(fl);
        //cout << "\tP(oVec, FL="<<fl<<") = " << p_oVec_fl.at(fl) << endl;        
    }
    //cout << "P(oVec)="<< p_oVec << endl;

    double u_jt_ja = 0.0;

    if(EqualProbability(p_oVec,0.0))
        return(0.0);

    for(Index fl=0; fl < _m_nrFireLevels; fl++)
    {
        double p_fl__oVec = p_oVec_fl.at(fl) / p_oVec;
        double u_fl_ja = GetHouseReward(fl, nrAgentsPresent);
        //cout << "\tp_fl__oVec="<<p_fl__oVec<<" u_fl_ja="<<u_fl_ja<<endl;
        u_jt_ja += p_fl__oVec * u_fl_ja;
    }
#if DEBUGUTILS    
    cout << "u_jt_ja="<<u_jt_ja<<endl;
    if(isnan(u_jt_ja))
        abort();
#endif
    return u_jt_ja;
}

double Problem_CGBG_FF::GetHouseReward(Index fl, size_t nrAgentsPresent) const
{
#if 1
    if (fl==0)
        return 0.0;
    
    double base_penalty = fl;
    double discount = pow(0.7, (double)nrAgentsPresent);    
    double reward = -base_penalty * discount;
    //cout << "discount="<<discount<<", reward="<<reward<<endl;
    return reward;
#else
    
    // if there are 2 or more agents present, they can extinguish the
    // fire completely
    switch(nrAgentsPresent)
    {
    case 0:
    case 1:
    {
        double base_penalty = fl;
        return(-1.0 * base_penalty);
    }
    default:
        return(0.0);
    }
#endif
}
#if 0
    //compute the probabilities over the firelevels of this house, given the joint type.
    //
    //First note that each agent gets observations of a subset of houses, not necesarrily
    //the one under concern. I.e., from a given vector of joint types for 4 agents:
    //  < <F,N>, <N,F>, <F,F>, <N,F> >
    //we can extract the observations that correspond to this house hI, e.g:
    //  < F, F, (), N>
    //where () denotes that the 3rd agent did not get an observation of this house.
    //
    //Given the observation vector <F,F,N> we can now compute 
    //
    //P(FL|<F,F,N>) =   P(FL,<F,F,N>) / P(<F,F,N>)
    //              =   P(<F,F,N>|FL)*P(FL) / norm
    //where
    //  Likelihood  P(<F,F,N>|FL) = P(F|FL)*P(F|FL)P(N|FL)
    //  Prior       P(FL)
    //  Norm        P(<F,F,N>) = \sum{FL}  P( <F,F,N> |FL) * P(FL)
    //
    //Clearly, in order to compute the probabilities here, we need 
    //the normalization terms
    //and we will need this in the future as well, so we will cache it.
    
    //Alright, first we get the observations for this house:
    vector<Problem_CGBG_FF::observation_t> oVec_hI = FilterObservationsForHouse(hI, tupleOfAgents, types);

    //next let's compute the Norm term:
    VecDMap::iterator it = _m_Norm_cache.find( oVec_hI ); 
    double norm = 0.0;
    if (it != _m_Norm_cache.end())
    {
        //found the norm in cache:
        norm = vd.second;
        VecDPair & vd = *it;
    else
    {
        for(Index FL=0; FL < _m_nrFireLevels; FL++)
            norm += Likelihood(oVec_hI, FL) * Prior(FL);
        _m_Norm_cache.insert( VecDPair(oVec_hI, norm));
    }
    cout << "P(jt)="<<norm<< 
        "  (jt="<<SoftPrintVector(types)<<", oVec="<< SoftPrintVector(oVec_hI) <<
        ")"<<endl;

    if (Globals::EqualProbability(norm, 0.0))
        return 0.0;


    //Now we loop over all FLs to compute
    //u(jt,ja) = \sum_{FL} P(FL|jt) * U(FL,ja)
    for(Index FL=0; FL < _m_nrFireLevels; FL++)
    {
        double P_FL__oVec = Likelihood(oVec_hI, FL) * Prior(FL) / norm;
    }

}
    
#endif
         




double Problem_CGBG_FF::GetProbability(Index jtype) const
{
    const vector<Index>& types = JointToIndividualTypeIndices(jtype);
    return ComputeLocalProbability(_m_allAgents, types);
}

void Problem_CGBG_FF::ExportToTextFiles(const string &suffix,
                                        const string &prefix) const
{
    string filename=prefix + GetUnixName() + suffix;

    cout << "Exporting to " << filename << endl;
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "Problem_CGBG_FF::ExportToTextFiles failed to "
           << "open file " << filename;
        throw(E(ss.str()));
    }

    size_t lineLength=std::max(_m_nrHouses,_m_nrAgents);
    fp << "% Text format of an instance of the " << GetUnixName() << " problem" << endl;
    fp << "% Next line are the x coordinates of each house" << endl;
    for(Index h=0;h!=_m_nrHouses;++h)
        fp << _m_housePositionX.at(h) << " ";
    for(Index h=_m_nrHouses;h<lineLength;++h)
        fp << "-1" << " ";
    fp << endl;
    fp << "% Next line are the y coordinates of each house" << endl;
    for(Index h=0;h!=_m_nrHouses;++h)
        fp << _m_housePositionY.at(h) << " ";
    for(Index h=_m_nrHouses;h<lineLength;++h)
        fp << "-1" << " ";
    fp << endl;
    fp << "% Next line are the x coordinates of each agent" << endl;
    for(Index a=0;a!=_m_nrAgents;++a)
        fp << _m_agentPositionX.at(a) << " ";
    for(Index a=_m_nrAgents;a<lineLength;++a)
        fp << "-1" << " ";
    fp << endl;
    fp << "% Next line are the y coordinates of each agent" << endl;
    for(Index a=0;a!=_m_nrAgents;++a)
        fp << _m_agentPositionY.at(a) << " ";
    for(Index a=_m_nrAgents;a<lineLength;++a)
        fp << "-1" << " ";
    fp << endl;

    fp << "% Next lines are the agentsForHouse_action scopes" << endl;
    for(Index h=0;h!=_m_nrHouses;++h)
    {
        for(Index a=0;a!= _m_agentsForHouse_action.at(h).size();++a)
            fp << _m_agentsForHouse_action.at(h).at(a) << " ";
        for(Index x=_m_agentsForHouse_action.at(h).size();x<lineLength;++x)
            fp << "-1" << " ";
        fp << endl;
    }
    fp << "% Next lines are the agentsForHouse_obs scopes" << endl;
    for(Index h=0;h!=_m_nrHouses;++h)
    {
        for(Index a=0;a!= _m_agentsForHouse_obs.at(h).size();++a)
            fp << _m_agentsForHouse_obs.at(h).at(a) << " ";
        for(Index x=_m_agentsForHouse_obs.at(h).size();x<lineLength;++x)
            fp << "-1" << " ";
        fp << endl;
    }
}


void Problem_CGBG_FF::ExportAsMAID(const string &suffix,
                                   const string &prefix) const
{
    string name = GetUnixName();
    string filename=prefix + name + suffix + ".bif";

    cout << "Exporting as MAID, filename " << filename << endl;
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "Problem_CGBG_FF::ExportAsMAID failed to "
           << "open file " << filename;
        throw(E(ss.str()));
    }

    //write the header

    fp << "maid \"" << name << "\" \n\
{\n\
	format is \"BNFormat\";\n\
	version is 0.00;\n\
	creator is \"MADP Toolbox\";\n\
}\n\
\n\
agents\n\
{\n\
";
    //write the agents
    for(Index agI=0; agI < GetNrAgents(); agI++)
        fp << "        Ag" << agI << "\n";
    fp << "}\n\n";


    //write the nodes for the fire level of each house...
    for(Index hI=0; hI < _m_nrHouses; hI++)
    {        
        fp << 
"chance FL" << hI << "\n\
{\n\
        type = discrete["<< _m_nrFireLevels <<"]\n\
        {\n";
        for(Index flI=0; flI < _m_nrFireLevels; flI++)
        {
            fp << "                \"fl_" << flI << "\"";
            if(flI !=  _m_nrFireLevels - 1)
                fp << ",";
            fp << endl;
        }
        fp << "\
        };\n\
        position = (0, 0);\n\
}\n\n";
    }



    //now create the nodes for each agent's type and action...
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        fp << 
"chance T" << agI << "\n\
{\n\
        type = discrete["<< GetNrTypes(agI) <<"]\n\
        {\n";
        for(Index tI=0; tI < GetNrTypes(agI); tI++)
        {
            fp << "                \"" << SoftPrintReadableType(agI, tI) << "\"";
            if(tI !=  GetNrTypes(agI) - 1)
                fp << ",";
            fp << endl;
        }
        fp << "\
        };\n\
        position = (0, 0);\n\
}\n\
\n\
decision Ac_" << agI << " Ag" << agI << "\n\
{\n\
        type = discrete["<< GetNrActions(agI) <<"]\n\
        {\n";
        for(Index aI=0; aI < GetNrActions(agI); aI++)
        {
            fp << "                \"a_" << agI << "^" << aI << "\"";
            if(aI !=  GetNrActions(agI) - 1)
                fp << ",";
            fp << endl;
        }
        fp << "\
        };\n\
        infoparents T" << agI << ";\n\
        position = (0, 0);\n\
}\n\
\n";

    }
    //write the utility nodes (1 for each agent) for each house...
    for(Index hI=0; hI < _m_nrHouses; hI++)
    {
        if( _m_agentsForHouse_action.at(hI).size() == 0)
            continue; //<- no agents can fight fire at this house, omit util node (otherwise segfault)
        for(Index agI=0; agI < GetNrAgents(); agI++)
            fp << "utilitynode " << "Uh" << hI << "ag" << agI << " Ag" << agI << "\n{\n}\n\n";
    }
    
    //specify the probabilities...
    for(Index hI=0; hI < _m_nrHouses; hI++)
    {
        fp << "probability ( FL" << hI << " )\n{\n";
        for(Index flI=0; flI < _m_nrFireLevels; flI++)
        {
            //fp << "(" << flI << ") = " << Prior(hI, flI) << endl;
            fp << Prior(hI, flI);
            if (flI < _m_nrFireLevels - 1)
                fp << ", ";
            else
                fp << ";" << endl;
        }
        fp << "}\n\n";
    }

    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        fp << "probability ( T" << agI << " | ";
        const vector<Index> & houses_observed_by_this_ag = _m_houseIndices_obs[agI];
        size_t nr_houses_obs = houses_observed_by_this_ag.size();
        vector<Index>::const_iterator it = houses_observed_by_this_ag.begin();
        vector<Index>::const_iterator last = houses_observed_by_this_ag.end();
        while(it != last)
        {
            fp << "FL" << *it;
            it++;
            if( it != last )
                fp << ", ";
            else
                fp << ")\n{\n";
        }
        vector<size_t> max_FL_vec = vector<size_t>( nr_houses_obs, _m_nrFireLevels );
        vector<Index> FL_vec =  vector<Index>( nr_houses_obs, 0 );
        do
        {
            //cout << SoftPrintVector(FL_vec) << endl;
            fp << "(";
            for(Index i=0; i < nr_houses_obs; i++)
            {
                //house_index = houses_observed_by_this_ag[i]
                fp << FL_vec[i];
                if(i != nr_houses_obs - 1)
                    fp << ", ";
                else
                    fp << ") = ";
            }

            //now loop over all the types of this agent and 
            //determine their probability given that FL_vec is the 
            //vector of fire levels
            for(Index typeI=0; typeI < GetNrTypes(agI); typeI++)
            {
                double type_p = ComputeIndividualTypeProb(agI, typeI, FL_vec);
                fp << type_p;
                if(typeI < GetNrTypes(agI) - 1)
                    fp << ", ";
                else
                    fp << ";\n";
            }
        } while( ! IndexTools::Increment(FL_vec, max_FL_vec) );
        fp << "}\n\n";
    }

    //specify the utilities...
    for(Index hI=0; hI < _m_nrHouses; hI++)
    {
        if( _m_agentsForHouse_action.at(hI).size() == 0)
            continue; //<- no agents can fight fire at this house, omit util node (otherwise segfault)

        //since the utility functions are the same for all the agents, 
        //we write the body to a string stream such that it can be reused for
        //all the agents
        stringstream util_body;
        stringstream action_string;
        size_t nr_agents_for_this_house = _m_agentsForHouse_action.at(hI).size();
        for(Index a = 0 ; a < nr_agents_for_this_house; a++)
        {
            Index agentI = _m_agentsForHouse_action.at(hI).at(a);
            action_string << ", Ac_" << agentI;
            //if(a != nr_agents_for_this_house - 1)
               //action_string << ", ";
            //else
        }
        action_string << " )";

        vector<size_t> max_a_vec = vector<size_t>( nr_agents_for_this_house, 0 );
        for(Index agI=0; agI < nr_agents_for_this_house; agI++)
        {
            Index agentI = _m_agentsForHouse_action.at(hI).at(agI);
            max_a_vec.at(agI) = GetNrActions(agentI);
        }
        vector<Index> a_vec =  vector<Index>( nr_agents_for_this_house, 0 );        
        for(Index FL=0; FL < _m_nrFireLevels; FL++)
        {
            do
            {
                //write the 'key'
                util_body << "(" << FL;
                for(Index i=0; i < nr_agents_for_this_house; i++)
                {
                    util_body << ", " << a_vec[i];
                    //if(i != nr_agents_for_this_house - 1)
                        //util_body << ", ";
                    //else
                }
                util_body << ") = ";
                //write the utility
                size_t nr_present = ComputeNumberOfAgentsPresentAtHouse(hI, a_vec);
                double payoff = GetHouseReward(FL, nr_present);
                util_body << payoff << ";\n";
                
            } while( ! IndexTools::Increment(a_vec, max_a_vec) );            
        }

        
        for(Index agI=0; agI < GetNrAgents(); agI++)
        {
            fp << "utility ( " << "Uh" << hI << "ag" << agI << " | FL" << hI 
                << action_string.str() << "\n";
            fp << "{\n" << util_body.str() << "}\n\n";
        }
    }
}

double Problem_CGBG_FF::ComputeIndividualTypeProb(Index agI, 
        Index typeI, std::vector<Index> & FL_vec) const
{
    std::vector<Index> obsIndices = TypeIndexToObservationIndices(agI, typeI);
    size_t nr_houses = obsIndices.size();
    //const vector<Index> & houses_observed_by_this_ag = _m_houseIndices_obs[agI];
    double p = 1.0;
    for(Index h = 0; h < nr_houses; h++)
        p *= FLObservationProb(FL_vec.at(h), obsIndices.at(h));
    
    return p;
}

string Problem_CGBG_FF::GetUnixName() const
{
    stringstream pn;
    pn << "FFCGBG_"
       << "_Ag" << GetNrAgents()
       << "_AcHs" << _m_nrActionsPerAgent << "_ObHs"  << _m_nrObservedHousesPerAgents 
       << "_k"<< _m_k
       <<"_Hs" << _m_nrHouses << "_FL" << _m_nrFireLevels;
    return(pn.str());
}

std::string Problem_CGBG_FF::SoftPrintReadableType(Index agI, Index typeI) const
{
    //vector<Index> _m_houseIndices_obs
    vector<Index> obsIndices = TypeIndexToObservationIndices(agI, typeI);
    stringstream s;
    s << "Ag"<< agI << "T" << typeI << ":";
    for(Index i=0; i < obsIndices.size(); i++)
    {
        s << "h" << _m_houseIndices_obs.at(agI).at(i);
        s << "o" << obsIndices.at(i);
    }
    return s.str();
}
