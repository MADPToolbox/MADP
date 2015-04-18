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

#include "StringTools.h"
#include "ProblemFOBSFireFightingFactored.h"
#include "RewardModelMapping.h"

using namespace std;

#define DEBUG_PFFF 0
#define DEBUG_CTM 0 //Create Trans Model


ProblemFOBSFireFightingFactored::ProblemFOBSFireFightingFactored(
    size_t nrAgents, size_t nrHouses, size_t nrFLs,
    double costOfMove, bool forcePositionRepres, bool initialize)
        :
        FactoredMMDPDiscrete(
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

void ProblemFOBSFireFightingFactored::InitializePFFF()
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
                                StringTools::Append("fl",flI));
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
        throw E("ProblemFOBSFireFightingFactored - initialization of ISD with positions not implemented yet");
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

    // Initialize the 2DBN in FactoredMMDPDiscrete
    Initialize2DBN();

#if DEBUG_PFFF
    cout << ">>>Trans model created"<<endl;
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

std::string ProblemFOBSFireFightingFactored::SoftPrintBriefDescription(
    size_t nrAgents, size_t nrHouses, size_t nrFLs) const
{
    stringstream ss;
    ss << "_FireFightingFactored_" << nrAgents << 
            "_" << nrHouses <<
            "_" << nrFLs;
    return ss.str();
}

std::string ProblemFOBSFireFightingFactored::SoftPrintDescription(size_t nrAgents,
                                                               size_t nrHouses, size_t nrFLs) const
{
    stringstream ss;
    ss << "The fully-observable factored (but non-graph) FireFighting problem with " 
       << nrAgents << 
            " Agents, " << nrHouses << " houses and "
       << nrFLs << " fire levels for each house.\n" <<
            "Factored means that the state space is factored, and thus that \
             the transition and reward models are represented in a factored \
             way (by a 2DBN and a collection of reward functions).\n    \
Non-graph means that the actions of the agents are not restricted (i.e., they \
can go to any house)";
    return ss.str();
}

void ProblemFOBSFireFightingFactored::ConstructActions()
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

size_t ProblemFOBSFireFightingFactored::GetNrAgentsAtHouse(const std::vector< Index>& As, Index hI) const
{
    size_t nrAgentsAtLocation = 0;
    for(Index aI=0; aI < As.size(); aI++)
        if(GetAgentLocation(As.at(aI),aI) == hI)
            nrAgentsAtLocation++;
    return(nrAgentsAtLocation);
}

double ProblemFOBSFireFightingFactored::ComputeTransitionProb(
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

size_t ProblemFOBSFireFightingFactored::GetAgentLocation(Index action,
                                                     Index agI) const
{
    return(action);
}

Scope ProblemFOBSFireFightingFactored::GetHousesAgentInfluences(Index agI) const
{
    Scope allHouses;
    for(Index hI=0; hI < _m_nrHouses; hI++)
        allHouses.Insert(hI);
    return(allHouses);
}

void ProblemFOBSFireFightingFactored::SetYScopes()
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
