/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "StringTools.h"
#include "ProblemFireFightingFactored.h"
#include "DecPOMDPDiscrete.h"
#include "JointBeliefInterface.h"
#include "RewardModelMapping.h"

using namespace std;

#define DEBUG_PFFF 0
#define DEBUG_CTM 0 //Create Trans Model
#define DEBUG_COM 0 //Create Obs Model
#define DEBUG_CJA 0
#define DEBUG_CJO 0
#define DEBUG_CA 0
#define DEBUG_CO 0


ProblemFireFightingFactored::ProblemFireFightingFactored(
        size_t nrAgents, size_t nrHouses, size_t nrFLs,
        double costOfMove, bool forcePositionRepres, bool initialize)
    :
        FactoredDecPOMDPDiscrete(
                SoftPrintBriefDescription(nrAgents, nrHouses, nrFLs),
                SoftPrintDescription(nrAgents, nrHouses, nrFLs),
                "none"),
        _m_nrAgents(nrAgents),
        _m_nrHouses(nrHouses),
        _m_nrFireLevels(nrFLs),
        _m_costOfMove(costOfMove),
        _m_forcePositionRepres(forcePositionRepres)
{
    if(initialize)
        InitializePFFF();
}

void ProblemFireFightingFactored::InitializePFFF()
{
    SetName(SoftPrintBriefDescription(_m_nrAgents, _m_nrHouses,
                                      _m_nrFireLevels));
    SetDescription(SoftPrintDescription(_m_nrAgents, _m_nrHouses,
                                        _m_nrFireLevels));
    SetUnixName(SoftPrintBriefDescription(_m_nrAgents, _m_nrHouses,
                                          _m_nrFireLevels));
    SetSparse(true);
    SetNrAgents(_m_nrAgents);

    double S = pow((double)_m_nrFireLevels, (double)_m_nrHouses);
    _m_nrJointFirelevels = (size_t) S;
    _m_nrFLs_vec =  vector<size_t>(_m_nrHouses, _m_nrFireLevels);

//Create states, and ISD
//
//first add the state features describing the (firelevels of) the houses)
    for(Index hI = 0; hI < _m_nrHouses; hI++)
    {

        Index sfI = AddStateFactor( 
                StringTools::Append("FL",hI) , 
                StringTools::Append("The fire level of house ",hI));
        for(Index flI=0; flI < _m_nrFireLevels; flI++)
            AddStateFactorValue(sfI, 
                    StringTools::Append("",flI));
    }
    
    if(_m_costOfMove > 0.0 || _m_forcePositionRepres)
    {
#if DEBUG_PFFF
        cout << "Including position"<<endl;
#endif
        _m_includePositions = true;
        
        for(Index agI=0; agI<_m_nrAgents; agI++)
        {
            Index sfI = AddStateFactor("Pos"+agI, "Position of agent "+agI);
            for(Index hI = 0; hI < _m_nrHouses; hI++)
                AddStateFactorValue(sfI, "House"+hI);
            AddStateFactorValue(sfI, "Start");
        }

        //the s_vec has the form <fl...fl_h, pos...pos_n>
        _m_nrStateFeatures = _m_nrHouses+_m_nrAgents;
        size_t nrPos = _m_nrHouses+1; //include the 'start' position.

        _m_nrFLs_vec = vector<size_t>(_m_nrHouses, _m_nrFireLevels);
        vector< size_t > nrPos_vec = vector<size_t>(_m_nrAgents, nrPos);
        _m_nrPerStateFeatureVec =  _m_nrFLs_vec;
        _m_nrPerStateFeatureVec.insert(
                _m_nrPerStateFeatureVec.end(), //insert position
                nrPos_vec.begin(),  //the stuff that is 
                nrPos_vec.end()     //appended...
                );
        if(DEBUG_PFFF)
            cout << "_m_nrPerStateFeatureVec=" <<
                SoftPrintVector(_m_nrPerStateFeatureVec) << endl;
    }
    else
    {
        cout << "Not including positions" << endl;
        _m_includePositions = false;
        //the s_vec has the form <fl...fl_h>
        _m_nrStateFeatures = _m_nrHouses;        
        _m_nrPerStateFeatureVec = _m_nrFLs_vec;

        if(DEBUG_PFFF)
            cout << "_m_nrPerStateFeatureVec=" <<
                SoftPrintVector(_m_nrPerStateFeatureVec) << endl;
    }

    SetStatesInitialized(true);
    //size_t nrS = GetNrStates();

//after initialization we can add the ISD:
    if(_m_includePositions)
    {
        /*
        vector<double> isd(nrS, 0.0);
        double nrStartStates = pow((double)_m_nrFireLevels, (double)_m_nrHouses);
        double ssprob = 1.0 / nrStartStates;
        for(Index sI = 0; sI < nrS; sI++)
        {
            vector< Index> s_vec = GetStateVector(sI);
            Index t1 = _m_nrHouses; //we start with the first position feature
            bool isStartState = true; 
            for(; t1 < _m_nrStateFeatures; t1++)
            {
                Index hI =  s_vec.at(t1);
                if(hI == _m_nrHouses) //start position
                    ; 
                else
                    isStartState = false;
            }
            if(isStartState)
                isd.at(sI) = ssprob;
        }
        this->SetISD(isd);
        */
        this->SetUniformISD();
        //still to be done: set probability of position factors to 1 for all start positions
        throw E("ProblemFireFightingFactored - initialization of ISD with positions not implemented yet");
    }
    else
        this->SetUniformISD();

#if DEBUG_PFFF  
    cout << "states initialized: " << endl ;
    cout << MADPComponentFactoredStates::SoftPrint() << endl;
#endif
    SetDiscount(1);    
#if DEBUG_PFFF
    cout << _m_2dbn.SoftPrint();
#endif

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

#if DEBUG_PFFF
    cout << ">>>Trans./obs. models created"<<endl;
    cout << _m_2dbn.SoftPrint();
#endif

//add rewards
    SetNrLRFs(_m_nrHouses);
    //first add scope for each reward function
    for(Index e=0; e < _m_nrHouses; e++)
    {
        //Add scope
        Scope emptySc;
        Scope ySc;
        ySc.Insert(e);//reward e depends on firelevel of house e at NS
        SetScopeForLRF(e, emptySc, emptySc, ySc, emptySc);
    }
    // compute some bookkeeping from the scopes
    InitializeInstantiationInformation();

    // now we add the reward functions themselves
    for(Index e=0; e < _m_nrHouses; e++)
    {
        Scope emptySc;
        Scope ySc;
        ySc.Insert(e);//reward e depends on firelevel of house e at NS
        //now get the (back-projected) X and A scope
        const Scope& sfSC = GetStateFactorScopeForLRF(e);
        const Scope& agSC = GetAgentScopeForLRF(e);
        string sf_descr = sfSC.SoftPrint();
        string ag_descr = agSC.SoftPrint();
        //the number of X instantiations (the size of the 'local' state space)
        size_t nrXIs = GetNrXIs(e);
        size_t nrAIs = GetNrAIs(e);

        RewardModelMapping* RMe = 
            new RewardModelMapping(nrXIs, nrAIs, sf_descr, ag_descr);
        SetRM(e, RMe);
        
        for(Index flI=0; flI < _m_nrFireLevels; flI++)
        {
            vector<Index> emptyVec;
            vector<Index> yVals(1, flI);// vector containing onyl flI
            SetRewardForLRF(  e,
                        emptySc, emptyVec, //X scope + value
                        emptySc, emptyVec, //A scope + value
                        ySc, yVals,
                        emptySc, emptyVec, //O scope + value
                        -(double)flI //the reward
                    );
        }
    }
 
#if DEBUG_PFFF
    cout << "Model created..."<<endl; 
#endif
    FactoredDecPOMDPDiscrete::SetInitialized(true);
}

std::string ProblemFireFightingFactored::SoftPrintBriefDescription(
        size_t nrAgents, size_t nrHouses, size_t nrFLs) const
{
    stringstream ss;
    ss << "FireFightingFactored_" << nrAgents << 
        "_" << nrHouses <<
        "_" << nrFLs;
    return ss.str();
}

std::string ProblemFireFightingFactored::SoftPrintDescription(size_t nrAgents,
        size_t nrHouses, size_t nrFLs) const
{
    stringstream ss;
    ss << "The factored (but non-graph) FireFighting problem with " 
        << nrAgents << 
        " Agents, " << nrHouses << " houses and "
        << nrFLs << " fire levels for each house.\n" <<
"Factored means that the state space is factored, and thus that the \
transition, observation and reward models are represented in a factored way \
(by a 2DBN and a collection of reward functions).\n\
Non-graph means that the actions of the agents are not restricted (i.e., they \
can go to any house)";
    return ss.str();
}

void ProblemFireFightingFactored::ConstructActions()
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

void ProblemFireFightingFactored::ConstructObservations()
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

size_t
ProblemFireFightingFactored::GetNrAgentsAtHouse(const std::vector< Index>& As,
                                                Index hI) const
{
    size_t nrAgentsAtLocation = 0;
    for(Index aI=0; aI < As.size(); aI++)
        if(GetAgentLocation(As.at(aI),aI) == hI)
            nrAgentsAtLocation++;
    return(nrAgentsAtLocation);
}


/*
size_t ProblemFireFightingFactored::
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
*/

/*
void ProblemFireFightingFactored::FillTransitionModel()
{
    // add transitions:
    for(Index ja=0; ja<GetNrJointActions(); ja++)
        for(Index s1=0; s1<GetNrStates();s1++) 
        {
#if DEBUG_CTM
            cout << "Transitions from s1="<<s1<<endl;
#endif
            vector< Index > s1_vec = GetStateVector(s1);

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
                    //SetTransitionProbability(s1, ja, fs2I, p);
                }
            }
        }
}
*/

double ProblemFireFightingFactored::ComputeTransitionProb(
            Index y,
            Index yVal,
            const std::vector< Index>& Xs,
            const std::vector< Index>& As,
            const std::vector< Index>& Ys
        ) const
{
    if(y >= _m_nrHouses)
        throw E("position sfac transition probs NYI");

    //this computes the probability P( nextLevel | ...) for hI
    Index hI = y;

    const Scope& XSoI = GetXSoI_Y(y);
    Index Index_of_hI_within_scope = 0;
    while(XSoI[Index_of_hI_within_scope] != hI &&
            Index_of_hI_within_scope < XSoI.size() )
        Index_of_hI_within_scope++;

    if(Index_of_hI_within_scope == XSoI.size())
        throw E( StringTools::Append("did not find this house (hI=",hI) +
                " in its own scope of influence XSoI)" );

    Index curLevel = Xs.at(Index_of_hI_within_scope);
    Index nextLevel = yVal;
    size_t nrAgentsAtLocation=GetNrAgentsAtHouse(As,hI);

#if 0 && DEBUG_CTM
    cout << "hI="<<hI<<" #agents="<<nrAgentsAtLocation << " ";
#endif
    bool neighborIsBurning = false;
    for(Index xI=0; xI < Xs.size(); xI++)
    {
        if(xI==Index_of_hI_within_scope)
            continue; //xI points not to a neighbor, but to this (hI) house
        if(Xs.at(xI) > 0)
            neighborIsBurning = true;
    }
    //bool neighborIsBurning = isNeighborBurning(s1, hI);

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
            //increase with p=0.4...

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
    return(p2);
}

/*
bool ProblemFireFightingFactored::isNeighborBurning( const std::vector< Index>& s, 
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
*/
/*
void ProblemFireFightingFactored::FillObservationModel()
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
                
                //SetObservationProbability(ja, s1, jo, prob);
            }
}
*/


double ProblemFireFightingFactored::ComputeObservationProb(
            Index o,
            Index oVal,
            const std::vector< Index>& As,
            const std::vector< Index>& Ys,
            const std::vector< Index>& Os
                ) const
{
    //Index agI = o;
    observation_t obsAgI = (observation_t) oVal;

    double p_o_thisAgent = 0.0;
    Index hI, FL;
    if(_m_includePositions)
    {
        //the agents position should be the last relevant
        //state variable (y):
        hI = Ys.at(Ys.size()-1); 
        if(hI == _m_nrHouses) // agent at start position
            FL = 0;
        else
            FL =  Ys.at(hI);
    }
    else
    {
        //the only relevant action gives the position (house) of agI

        // We don't use GetAgentLocation() because Ys will already
        // contain the scope of each's agent houses that it can reach
        hI=As.at(0);//GetAgentLocation(As.at(0),o);
        FL = Ys.at(hI);   //the firelevel at that house
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

    switch(obsAgI)
    {
        case(FLAMES):
            p_o_thisAgent = pFlames;
            break;
        case(NOFLAMES):
            p_o_thisAgent = pNoFlames;
            break;
    }
    return p_o_thisAgent;
}

size_t
ProblemFireFightingFactored::GetAgentLocation(Index action,
                                              Index agI) const
{
    return(action);
}

Scope ProblemFireFightingFactored::GetHousesAgentInfluences(Index agI) const
{
    Scope allHouses;
    for(Index hI=0; hI < _m_nrHouses; hI++)
        allHouses.Insert(hI);
    return(allHouses);
}

void ProblemFireFightingFactored::SetYScopes()
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
        if(yI > 0)
            x.Insert(yI-1);
        x.Insert(yI); // firelevel of house yI influenced by its PS firelevel
        if(yI < (_m_nrHouses-1) )
            x.Insert(yI+1);

        SetSoI_Y( yI, //sfacI = 0 -> house 0
                  x,
                  allAgents,
                  Scope("<>") // no interdependencies between next-stage FLs
                );
    }
}

void ProblemFireFightingFactored::SetOScopes()
{
#if DEBUG_PFFF
    cout << "About to set the SoIs for all observations O..."<<endl;
#endif
    for(Index oI=0; oI < _m_nrAgents; oI++)
    {
        Scope agSC;
        agSC.Insert(oI); //only agents' own action influences its obseration
        SetSoI_O( oI, //sfacI = 0 -> house 0
                  agSC,
                  GetHousesAgentInfluences(oI),
                  Scope("<>") // no interdependencies between next-stage Os
            );
    }
}
