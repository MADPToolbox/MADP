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
#include <algorithm>

#include "ProblemFireFighting.h"
#include "IndexTools.h"


#define DEBUG_PFF 0
#define DEBUG_CTM 0 //Create Trans Model
#define DEBUG_COM 0 //Create Obs Model
#define DEBUG_CJA 0
#define DEBUG_CJO 0
#define DEBUG_CA 0
#define DEBUG_CO 0


using namespace std;

//Default constructor
ProblemFireFighting::ProblemFireFighting(
        size_t nrAgents, size_t nrHouses, size_t nrFLs,
        double costOfMove, bool forcePositionRepres)
    :
        DecPOMDPDiscrete(
                SoftPrintBriefDescription(nrAgents, nrHouses, nrFLs),
                SoftPrintDescription(nrAgents, nrHouses, nrFLs),
                SoftPrintBriefDescription(nrAgents, nrHouses, nrFLs)
                )
        ,_m_nrAgents(nrAgents)
        ,_m_nrHouses(nrHouses)
        ,_m_nrFireLevels(nrFLs)
{
    SetSparse(true);
    SetNrAgents(nrAgents);


    double S = pow((double)nrFLs, (double)nrHouses);
    _m_nrJointFirelevels = (size_t) S;
    _m_nrFLs_vec =  vector<size_t>(nrHouses, nrFLs);

    //Create states, and ISD
    if(costOfMove > 0.0 || forcePositionRepres)
    {
        cout << "Including position"<<endl;
        _m_includePositions = true;
        //we're duplicating the generation of states as the matlab code does
        //(this is not the best thing as it generates states where one agent 
        //is in a start state and the other is not...)


        //the s_vec has the form <fl...fl_h, pos...pos_n>
        _m_nrStateFeatures = nrHouses+nrAgents;
        size_t nrPos = nrHouses+1; //include the 'start' position.

        _m_nrFLs_vec = vector<size_t>(nrHouses, nrFLs);
        vector< size_t > nrPos_vec = vector<size_t>(nrAgents, nrPos);
        _m_nrPerStateFeatureVec =  _m_nrFLs_vec;
        _m_nrPerStateFeatureVec.insert(
                _m_nrPerStateFeatureVec.end(), //insert position
                nrPos_vec.begin(),  //the stuff that is 
                nrPos_vec.end()     //appended...
                );
        if(DEBUG_PFF)
            cout << "_m_nrPerStateFeatureVec=" <<
                SoftPrintVector(_m_nrPerStateFeatureVec) << endl;
        size_t nrS=1;
        for(vector<size_t>::const_iterator it = _m_nrPerStateFeatureVec.begin();
                it < _m_nrPerStateFeatureVec.end();
                it++)
            nrS *= *it;

        vector<double> isd(nrS, 0.0);
        double nrStartStates = pow((double)nrFLs, (double)nrHouses);
        double ssprob = 1.0 / nrStartStates;

        for(Index sI = 0; sI < nrS; sI++)
        {
            vector< Index> s_vec = GetStateVector(sI);
            stringstream ss;
            ss << "S_";
            Index t1;
            for(t1 = 0; t1 < nrHouses; t1++)
                ss << "f" << s_vec.at(t1);
            bool isStartState = true;
            for(; t1 < _m_nrStateFeatures; t1++)
            {
                Index hI =  s_vec.at(t1);
                ss << "h";
                if(hI == nrHouses) //start position
                    ss << "S";
                else
                {
                    ss << hI;
                    isStartState = false;
                }
            }
            this->AddState(ss.str());
            if(isStartState)
                isd.at(sI) = ssprob;

            if(DEBUG_PFF)
            {
                cout << "added " << this->GetState(sI)->SoftPrintBrief()<<endl;
            }
        }
        StateDistributionVector *isdv=new StateDistributionVector(isd);
        this->SetISD(isdv);

    }
    else
    {
        cout << "Not including positions" << endl;
        _m_includePositions = false;
        //the s_vec has the form <fl...fl_h>
        _m_nrStateFeatures = nrHouses;        
        _m_nrPerStateFeatureVec = _m_nrFLs_vec;

        if(DEBUG_PFF)
            cout << "_m_nrPerStateFeatureVec=" <<
                SoftPrintVector(_m_nrPerStateFeatureVec) << endl;

        size_t nrS = _m_nrJointFirelevels;
        if(DEBUG_PFF)
            cout << "nrStates="<<nrS<<endl;
        for(Index sI = 0; sI < nrS; sI++)
        {
            vector< Index> s_vec = GetStateVector(sI);
            stringstream ss;
            ss << "S_" << SoftPrintVector(s_vec);
            this->AddState(ss.str());
            if(DEBUG_PFF)
            {
                cout << "added " << this->GetState(sI)->SoftPrintBrief()<<endl;
            }
        }
        SetUniformISD();
    }

    SetStatesInitialized(true);
    SetDiscount(1);
    
    // add actions:
    ConstructActions();
    if(DEBUG_CJA) cout << ">>>Creating joint actions and set..."<<endl;
    // add joint actions
    size_t testNRJA = ConstructJointActions();
    if(DEBUG_CA) cout << "testNRJA="<<testNRJA<<endl;    

    SetActionsInitialized(true);

    // add observations:
    ConstructObservations();
    size_t testNRJO = ConstructJointObservations();
    if(DEBUG_CO) cout << "testNRJO="<<testNRJO<<endl;

    SetObservationsInitialized(true);

    // add the transition model
    if(DEBUG_PFF) cout << ">>>Adding Transition model..."<<endl;
    CreateNewTransitionModel();
    FillTransitionModel();    

    // add observation model
    if(DEBUG_PFF) cout << ">>>Adding Observation model..."<<endl;
    CreateNewObservationModel();
    FillObservationModel();
    MultiAgentDecisionProcessDiscrete::SetInitialized(true);

    // add rewards
    CreateNewRewardModel();
    FillRewardModel();
    if(DEBUG_PFF)     cout << "Model created..."<<endl; 
    DecPOMDPDiscrete::SetInitialized(true);
}

/*
//Copy constructor.    
ProblemFireFighting::ProblemFireFighting(const ProblemFireFighting& o) 
{
}
//Destructor
ProblemFireFighting::~ProblemFireFighting()
{
}
//Copy assignment operator
ProblemFireFighting& ProblemFireFighting::operator= (const ProblemFireFighting& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/

std::string ProblemFireFighting::SoftPrintBriefDescription(
        size_t nrAgents, size_t nrHouses, size_t nrFLs)
{
    stringstream ss;
    ss << "FireFighting_" << nrAgents << 
        "_" << nrHouses <<
        "_" << nrFLs;
    return ss.str();
}

std::string ProblemFireFighting::SoftPrintDescription(size_t nrAgents,
        size_t nrHouses, size_t nrFLs)
{
    stringstream ss;
    ss << "The regular (non-factored) FireFighting problem with" << nrAgents << 
        " Agents, " << nrHouses << " houses and "
        << nrFLs << " fire levels for each house";
    return ss.str();

}

void ProblemFireFighting::ConstructActions()
{
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        size_t nrActionsThisAgent = _m_nrHouses;
//         _m_nrActions.push_back(nrActionsThisAgent);
//         _m_actionVecs.push_back( vector<ActionDiscrete>() );
        for(Index actionI=0; actionI < nrActionsThisAgent; actionI++)
        {
            stringstream ss;
            //ss << "Ag" <<agentIndex << ":House" << actionI;
            ss << "go" << actionI;
            string name = ss.str();
            ss.str("");
            ss << "Action " << actionI << " of agent " << agentIndex 
                << ": go to and fight fire at house "<<actionI;
            string descr = ss.str();

//             ActionDiscrete ad_temp =   ActionDiscrete(actionI,name,descr);
//             _m_actionVecs[agentIndex].push_back( ad_temp );
            AddAction(agentIndex,name,descr);
        }
    }
}

void ProblemFireFighting::ConstructObservations()
{
    /// add observations:
    for(Index agentIndex=0; agentIndex < _m_nrAgents; agentIndex++)
    {
        size_t nrObservationsThisAgent = 2; //flames or no flames
//         _m_nrObservations.push_back(nrObservationsThisAgent);
//         _m_observationVecs.push_back( vector<ObservationDiscrete>() );
        for(Index obsI=0; obsI < nrObservationsThisAgent; obsI++)
        {
            string whatObs;
            switch(obsI){
            case(FLAMES):    whatObs = "Flames__";
                break;
            case(NOFLAMES):    whatObs = "NoFlames";
                break;
            }
            stringstream ss;
            //ss << "Ag" <<agentIndex << ":" << whatObs;
            ss << whatObs;
            string name = ss.str();
            ss.str("");
            ss << "Observation " << obsI << " of agent " << agentIndex << ": " << whatObs;
            string descr = ss.str();
//             ObservationDiscrete od_temp = ObservationDiscrete(obsI,name,descr);
//             _m_observationVecs[agentIndex].push_back( od_temp );
            AddObservation(agentIndex,name,descr);
        }
    }
}

size_t ProblemFireFighting::
NumberOfContainedStartPositions(const vector<Index>& state) 
    const
{
    if(!_m_includePositions)
        return(0);

    size_t nrSPs = 0;
    //loop over the position features:
    for(Index i = _m_nrHouses; i < _m_nrStateFeatures; i++)
        if(state.at(i) == _m_nrHouses) //the 'start' postion index = nrHouses
            nrSPs++;
    return nrSPs;
    
}

void ProblemFireFighting::FillTransitionModel()
{
    // add transitions:
    for(Index ja=0; ja<GetNrJointActions(); ja++)
        for(Index s1=0; s1<GetNrStates();s1++) 
        {
#if DEBUG_CTM
            cout << "Transitions from s1="<<s1<<endl;
#endif
            vector< Index > s1_vec = GetStateVector(s1);
/* skip check for matlab generation compatibility            
            size_t nrSPs1 = NumberOfContainedStartPositions(s1_vec);
            if(nrSPs1 > 0 && nrSPs1 != GetNrAgents())
            {
                //illegal state, the prob. is zero except for transition
                //to self:
                SetTransitionProbability(s1, ja, s1, 1.0);
                //else 0, but we do not need to add that
                continue;
            }
*/
            //Since movements are deterministic, we can simply fill out
            //the position elements of the state factor
            //therefore we only need to loop over joint-firelevel vectors 
            //here
            for(Index s2=0; s2<_m_nrJointFirelevels;s2++) 
            {
                vector< Index > s2_vec = IndexTools::JointToIndividualIndices
                    (s2, _m_nrFLs_vec);
                //no longer necessary since we fill in the position components
                //if(NumberOfContainedStartPositions(s2_vec) > 0)
                    //continue;

                vector< Index > s1_vec_stripped(&s1_vec[0], 
                        &s1_vec[_m_nrHouses] );
                vector< Index > ja_vec = JointToIndividualActionIndices(ja);
                double p = ComputeTransitionProb(s1_vec_stripped, 
                        ja_vec, s2_vec);
#if DEBUG_CTM
                cout << "Trans from s="
                    << SoftPrintVector(s1_vec) 
                    << ", a=" 
                    << SoftPrintVector(ja_vec) 
                    << " to s'="
                    << SoftPrintVector(s2_vec) 
                    << " Prob=" << p << endl;
#endif
                if(p > 0.0)
                {
                    //compute full s2 index
                    vector< Index >& full_s2 = s2_vec;
                    full_s2.insert(full_s2.end(), ja_vec.begin(), ja_vec.end());
                    Index fs2I = IndexTools::IndividualToJointIndices(full_s2, 
                            _m_nrPerStateFeatureVec);
                    SetTransitionProbability(s1, ja, fs2I, p);
                }
            }
        }
}

double ProblemFireFighting::ComputeTransitionProb(
                const std::vector< Index>& s1,
                const std::vector< Index>& ja,
                const std::vector< Index>& s2
                ) const
{
    double p = 1.0;
    for(Index hI=0; hI < s1.size(); hI++)
    {
        Index curLevel = s1.at(hI);
        Index nextLevel = s2.at(hI);
        Index nrAgentsAtLocation = 0;
        for(Index aI=0; aI < ja.size(); aI++)
            if(ja.at(aI) == hI)
                nrAgentsAtLocation++;
#if 0 && DEBUG_CTM
        cout << "hI="<<hI<<" #agents="<<nrAgentsAtLocation << " ";
#endif
        //this is dependent on s1 right?:
        bool neighborIsBurning = isNeighborBurning(s1, hI);

        Index sameLevel = curLevel;
        Index higherLevel = min((size_t)sameLevel+1, _m_nrFireLevels-1);
        Index lowerLevel = (curLevel==0) ? 0 : (curLevel-1);
        double p2=0.0;// the prob. factor of ThisHouseFirelevel;
        switch(nrAgentsAtLocation)
        {
            case(0): 
            {
                //this is kind of strange: when a house is not burning, but
                //its neigbhor is, it will increase its FL with p=0.8
                //but when it is already burning (and its neighbor is not), it 
                //increase with p=0.6...

                //fire is likely to increase
                if(neighborIsBurning)
                {
                    if(nextLevel == sameLevel)
                        p2+=0.2;
                    if(nextLevel == higherLevel)
                        p2+=0.8;
                }
                else if (curLevel == 0) //fire won't get ignited
                {
                    if(0 == nextLevel)
                        p2=1.0;
                    else //not possible so we can quit...
                        p2=0.0;
                }
                else //normal burning house
                {
                    if(nextLevel == sameLevel)
                        p2+=0.6;
                    if(nextLevel == higherLevel)
                        p2+=0.4;
                }
                break;
            }
            case(1): 
            {
                //fire is likely to decrease
                if(neighborIsBurning)
                {
                    if(nextLevel == sameLevel)
                        p2+=0.4;
                    if(nextLevel == lowerLevel) 
                        p2+=0.6; //.6 prob of extuinguishing 1 fl
                }
                else if (curLevel == 0) //fire won't get ignited
                {
                    if(0 == nextLevel)
                        p2=1.0;
                    else //not possible so we can quit...
                        p2=0.0;
                }
                else //normal burning house
                {
                    if(nextLevel == sameLevel)
                        p2+=0.0;
                    if(nextLevel == lowerLevel)
                        p2+=1.0;
                }
                break;
            }
            default: 
            {
                //more than 1 agent: fire is extinguished
                if(0 == nextLevel)
                    p2=1.0;
                else //not possible so we can quit...
                    p2=0.0;
            }

            
        }
#if 0 && DEBUG_CTM
        cout << "p=" << p << ", p2=" << p2;
#endif
        p *= p2;
#if 0 && DEBUG_CTM
        cout << ", new p=" << p << " - ";
#endif

    }
#if DEBUG_CTM
    cout << "returning p=" << p << endl;
#endif
    return p;
}

bool ProblemFireFighting::isNeighborBurning( const std::vector< Index>& s, 
        Index hI) 
{
    bool b = false;
    if(hI > 0) //check 'lower' neighbor
        if(s.at(hI-1) > 0)
            b = true;
    if(hI+1 < s.size()) //check 'higher' neighbor
        if(s.at(hI+1) > 0)
            b = true;

    return(b);

}

void ProblemFireFighting::FillObservationModel()
{
    for(Index  ja=0; ja< GetNrJointActions(); ja++)
        for(Index s1=0; s1<GetNrStates();s1++) 
            for(Index jo=0; jo<GetNrJointObservations();jo++) 
            {
                vector< Index > ja_vec = JointToIndividualActionIndices(ja);
                vector< Index > s1_vec = GetStateVector(s1);
                vector< Index > jo_vec = 
                    JointToIndividualObservationIndices(jo);

                double prob = ComputeObservationProb(ja_vec, s1_vec, jo_vec);
                
                SetObservationProbability(ja, s1, jo, prob);
            }
}
double ProblemFireFighting::ComputeObservationProb(
        const std::vector< Index>& ja,
        const std::vector< Index>& s1,
        const std::vector< Index>& jo
                ) const
{
    double p_jo = 1.0;
    for(Index agI=0; agI < ja.size(); agI++)
    {
        double p_o_thisAgent = 0.0;
        Index hI, FL;
        if(_m_includePositions)
        {
            hI = s1.at(_m_nrHouses + agI);
            if(hI == _m_nrHouses) // agent at start position
                FL = 0;
            else
                FL =  s1.at(hI);
        }
        else
        {
            hI=ja.at(agI);    //the position (house) of agI
            FL = s1.at(hI);   //the firelevel at that house
        }
        //we compute P(FLAMES)
        double pFlames = 0.0;
        switch(FL)
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
        observation_t obsAgI = (observation_t) jo.at(agI);
        switch(obsAgI)
        {
            case(FLAMES):
                p_o_thisAgent = pFlames;
                break;
            case(NOFLAMES):
                p_o_thisAgent = pNoFlames;
                break;
        }
        p_jo *= p_o_thisAgent;
    }
    return p_jo;
}
void ProblemFireFighting::FillRewardModel()
{    

    for(Index s1=0; s1<GetNrStates();s1++) 
        for(Index ja=0; ja<GetNrJointActions(); ja++)
            for(Index s2=0; s2<GetNrStates();s2++) 
            {
                double r = ComputeReward(s2);
                SetReward(s1, ja, s2, r);
            }
}

double ProblemFireFighting::ComputeReward(Index sI) const
{
    vector<Index> fl_vec = GetStateVector(sI);
    double r = 0.0;
    //for( vector<Index>::const_iterator it = fl_vec.begin();
            //it != fl_vec.end();
            //it++)
        //r -= (double) *it;
    for(Index hI = 0; hI < _m_nrHouses; hI++)
        r -= (double) fl_vec.at(hI);

    return(r);
}

