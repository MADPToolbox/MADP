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

#include "PlanningUnitMADPDiscrete.h"
#include <fstream>

#include "IndexTools.h"

#include "Action.h" 
#include "ActionDiscrete.h" 
#include "JointActionDiscrete.h" 
#include "Observation.h" 
#include "ObservationDiscrete.h" 
#include "JointObservationDiscrete.h" 

#include "JointBelief.h"
#include "JointBeliefSparse.h"
#include "JointPolicyDiscrete.h"
#include "PolicyDiscretePure.h"

#include "ActionHistory.h" 
#include "ActionHistoryTree.h" 
#include "JointActionHistory.h" 
#include "JointActionHistoryTree.h" 
#include "ObservationHistoryTree.h"
#include "JointObservationHistoryTree.h"
#include "ActionObservationHistoryTree.h"
#include "JointActionObservationHistoryTree.h"

#include "ENotCached.h"
#include "EOverflow.h"

#define DEBUG_PUD 0

// debug individual observation/action/actionobservation histories generation
#define DEBUG_PUDCOHT 0
#define DEBUG_PUDCAHT 0
#define DEBUG_PUDCAOHT 0

// debug joint observation/action/actionobservation histories generation
#define DEBUG_PUDCJOHT 0
#define DEBUG_PUDCJAHT 0
#define DEBUG_PUDCJAOHT 0

#define DEBUG_PUD_JAOHPROBS 0
#define DEBUG_PUDCGETJB 0

using namespace std;

//Default constructor
PlanningUnitMADPDiscrete::
PlanningUnitMADPDiscrete(
                         size_t horizon,
                         MultiAgentDecisionProcessDiscreteInterface* p,
                         const PlanningUnitMADPDiscreteParameters* params
    ) : 
    PlanningUnit(horizon,p),
    Interface_ProblemToPolicyDiscretePure()
    ,_m_madp(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitMADPDiscrete(PlanningUnitMADPDiscreteParameters params, size_t horizon, MultiAgentDecisionProcessDiscreteInterface* p)  called" << endl;
    _m_initialized = false;
    if(params != 0)
    {
        params->SanityCheck();
        _m_params=*params;
    }
    //else _m_params stores the default PlanningUnitMADPDiscreteParameters 
    //      object. Could also consider: _m_params.SetComputeAll(true);

    if(p != 0)
        Initialize();
}

///DEPRECATED?:
PlanningUnitMADPDiscrete::
PlanningUnitMADPDiscrete(size_t horizon,
                         MultiAgentDecisionProcessDiscreteInterface* p
    ) : 
    PlanningUnit(horizon,p),
    Interface_ProblemToPolicyDiscretePure()
    ,_m_madp(p)
{
    if(DEBUG_PU_CONSTRUCTORS) cout << "PlanningUnitMADPDiscrete( size_t horizon, MultiAgentDecisionProcessDiscreteInterface* p)  called" << endl;
    _m_params.SetComputeAll(true);
    _m_initialized = false;
    if(p != 0)
        Initialize();
}

//Destructor
PlanningUnitMADPDiscrete::~PlanningUnitMADPDiscrete()
{
    Deinitialize();
}

void PlanningUnitMADPDiscrete::SetProblem(MultiAgentDecisionProcessDiscreteInterface* m)
{
    if(DEBUG_PUD){
        cout << "\n>>>PlanningUnitMADPDiscrete::SetProblem - setting the following problem\n" 
             << m->SoftPrint() << endl;
    }
    _m_madp = m; 
    PlanningUnit::SetProblem(m);
    Deinitialize();
    Initialize();
}

void PlanningUnitMADPDiscrete::SetHorizon(size_t h)
{
    Deinitialize();
    PlanningUnit::SetHorizon(h);
    Initialize();
}

void
PlanningUnitMADPDiscrete::
SetParams(const PlanningUnitMADPDiscreteParameters &params)
{
    Deinitialize();
    _m_params=params;
    Initialize();
}

bool PlanningUnitMADPDiscrete::SanityCheck() const
{
    return(true);
}

//call methods to initialize the Planning unit
void PlanningUnitMADPDiscrete::Initialize()
{
    if(DEBUG_PUD)     cout << "PlanningUnitMADPDiscrete::Initialize()" <<endl;

    // We cannot initialize before the problem is set, so return
    // (SetProblem() will call Initialize() again)
    if(_m_madp == 0)
        return;

    if(!SanityCheck())
        throw(E("PlanningUnitMADPDiscrete::Initialize sanity check failed"));

    if(_m_initialized)
        Deinitialize();

    try
    {
        // calculating this stuff does not make sense for
        // infinite-horizon problems
        if(GetHorizon()!=MAXHORIZON)
        {
            InitializeObservationHistories();
            InitializeActionHistories();
            InitializeActionObservationHistories();
            InitializeJointObservationHistories();
            InitializeJointActionHistories();
            InitializeJointActionObservationHistories();
        }
    }
    catch(E& e)
    {
        e.Print();
        return;
    }
    _m_initialized = true;
if(DEBUG_PUD){    cout << "PlanningUnitMADPDiscrete::Initialize() done - info:" <<endl;     Print(); }
}

void PlanningUnitMADPDiscrete::Deinitialize()
{
    if(_m_initialized)
    {
        if(GetHorizon()!=MAXHORIZON)
        {
            // delete old stuff if necessary:
            DeInitializeObservationHistories();
            DeInitializeActionHistories();
            DeInitializeActionObservationHistories();
            DeInitializeJointObservationHistories();
            DeInitializeJointActionHistories();
            DeInitializeJointActionObservationHistories();
        }
        _m_initialized=false;
    }
}

void PlanningUnitMADPDiscrete::InitializeObservationHistories()
{
    if(_m_params.GetComputeIndividualObservationHistories())
    {
        //constructs the observation histories for all agents and puts them in trees.
        size_t nrAgents=GetNrAgents();
        for(Index i = 0; i < nrAgents; i++)
        {
            vector<ObservationHistoryTree*> * temp = new 
                vector<ObservationHistoryTree*>;
            _m_observationHistoryTreeVectors.push_back(* temp );
            _m_nrObservationHistoriesT.push_back(vector<size_t>());
            _m_firstOHIforT.push_back(vector<LIndex>());
            delete temp; // temp is copied into the vector - so delete this imm.
            ObservationHistoryTree* treeThiAg = CreateObservationHistoryTree(i);
            _m_observationHistoryTreeRootPointers.push_back(treeThiAg);
        }
    }
    else
    {
        /*Calculates the number of individual observation histories
         * and stores this in _m_nrObservationHistories (and
         * _m_nrObservationHistoriesT).  Only used when the observation
         * histories are not created.
         */
        _m_nrObservationHistories.clear();
        _m_nrObservationHistoriesT.clear();
        _m_firstOHIforT.clear();
        vector<size_t> oh_per_t;
        vector<LIndex> firstOHindex_per_t;
        size_t total = 0;
        for(Index i=0; i < GetNrAgents(); i++)
        {
            size_t nrOi = GetMADPDI()->GetNrObservations(i);
            oh_per_t.clear();
            firstOHindex_per_t.clear();
            total = 0;
            for(Index t=0; t < GetHorizon(); t++)
            {
                firstOHindex_per_t.push_back(total);
                size_t oh_this_t = (size_t) pow( (double)nrOi, (double)t );
                oh_per_t.push_back(oh_this_t);
                total += oh_this_t;
            }
            _m_firstOHIforT.push_back(firstOHindex_per_t);
            _m_nrObservationHistories.push_back(total);
            _m_nrObservationHistoriesT.push_back(oh_per_t);
        }
    }
}

//should only be called from CreateObservationHistories() 
//(should be called in the order of the agents)
ObservationHistoryTree*
PlanningUnitMADPDiscrete::CreateObservationHistoryTree(Index agentI)
{
    //we want to number the Observation Histories breadth-first, so we 
    //use a queue:
    queue<ObservationHistoryTree*> ohtQueue;
    
    Index ObservationHistoryIndex = 0;
    Index maxLength = GetHorizon()-1;//max. observation hist length = h-1
    size_t nrO = GetNrObservations(agentI);
    ObservationHistory* next_oh = 0;
    ObservationHistoryTree* next_oht = 0;
    
    // return a new initial (=empty) observation history

    ObservationHistory* oh = new ObservationHistory(*this, agentI);
    ObservationHistoryTree* root = new ObservationHistoryTree(oh);//, nrO);
    ohtQueue.push(root);

    Index ts = 0;
    _m_firstOHIforT[agentI].push_back(ObservationHistoryIndex);
    while(ohtQueue.size() > 0)
    {
        ObservationHistoryTree* oht = ohtQueue.front();
        oht->SetIndex(ObservationHistoryIndex);
        _m_observationHistoryTreeVectors[agentI].push_back(oht);

        size_t thisLength = oht->GetContainedElement()->GetLength(); 
        if(ts != (Index) thisLength) //length 1 <-> ts 1 
        {
            // we've now started processing OHs for the next ts
            Index nr_OHI_for_ts = ObservationHistoryIndex - 
                CastLIndexToIndex(_m_firstOHIforT[agentI][ts]);
#if DEBUG_PUDCOHT
            cout << "First oh for next ts+1: store nr of ts-OH..."<<endl<<
                "_m_firstOHIforT[agentI="<<agentI<<"][ts="<<ts<<"] = "<<
                _m_firstOHIforT[agentI][ts] << ", current OHI = " <<
                ObservationHistoryIndex << endl;
            Index tempIndex = _m_nrObservationHistoriesT[agentI].size();
#endif
            _m_nrObservationHistoriesT[agentI].push_back(nr_OHI_for_ts); 
#if DEBUG_PUDCOHT
            cout << "Set _m_nrObservationHistoriesT[agentI="<<agentI<<"][t="<<
                tempIndex <<" (ts="<<ts<<")] = "<< nr_OHI_for_ts << endl;
#endif
            if(++ts != (Index) thisLength )
                cout << "ERROR: CreateObservationHistories:  ++ts != "<<
                    "(Index) thisLength !!! ts (now) is: "<<ts<<
                    "thisLength =" << thisLength <<endl;
            _m_firstOHIforT[agentI].push_back(ObservationHistoryIndex);            
        }
        
        ObservationHistoryIndex++; //index of an Observation HISTORY
        
        if(thisLength < maxLength)
            for(Index obsI = 0; obsI < nrO; obsI++) //obsI = observation index
            {
                next_oh = new ObservationHistory(obsI, 
                    oht->GetObservationHistory());
                next_oht = new ObservationHistoryTree(next_oh);//, nrO);
                oht->SetSuccessor(obsI, next_oht);
                ohtQueue.push(next_oht);
            }
        ohtQueue.pop();
    }    

    //there is no next time step, so _m_nrObservationHistoriesT[agentI][lastTs]
    //have not been stored yet: do this now
    Index nr_OHI_for_ts = (ObservationHistoryIndex) 
        - CastLIndexToIndex(_m_firstOHIforT[agentI][ts]);
#if DEBUG_PUDCOHT
    cout << "Last ts end: store nr of ts=h-1 OH..."<<endl<<
        "_m_firstOHIforT[agentI="<<agentI<<"][ts="<<ts<<"] = "<<
        _m_firstOHIforT[agentI][ts] << ", current OHI = " <<
        ObservationHistoryIndex << endl;
    Index tempIndex = _m_nrObservationHistoriesT[agentI].size();
#endif
    _m_nrObservationHistoriesT[agentI].push_back(nr_OHI_for_ts);
#if DEBUG_PUDCOHT
    cout << "Set _m_nrObservationHistoriesT[agentI="<<agentI<<"][t="<<tempIndex
         <<" (last time step ts="<<ts<<")] = "<< nr_OHI_for_ts<<endl;
#endif
    //in a horizon h MADP without initial observation, the max. observation 
    //sequence length is h-1 observations (and the minimum is 0).
    size_t nrObsHist;
    size_t length = GetHorizon() - 1; // = maxLength - 1
    nrObsHist = IndexTools::CalculateNumberOfSequences(nrO, length);

    if( ObservationHistoryIndex != nrObsHist || 
        nrObsHist != _m_observationHistoryTreeVectors[agentI].size() )
        cerr << " WARNING:ObservationHistoryIndex="<< ObservationHistoryIndex <<
        " nrObsHist=" << nrObsHist <<" _m_observationHistoryTreeVectors[agentI="
        << agentI<<"].size()=" << _m_observationHistoryTreeVectors[agentI].size()
        << endl;
    
    _m_nrObservationHistories.push_back(ObservationHistoryIndex);

    return(root);
}

//constructs the observation histories for all agents and puts them in trees.
void PlanningUnitMADPDiscrete::DeInitializeObservationHistories()
{
    vector<ObservationHistoryTree *>::iterator it = 
    _m_observationHistoryTreeRootPointers.begin();
    vector<ObservationHistoryTree *>::iterator last = 
    _m_observationHistoryTreeRootPointers.end();
    while(it != last)
    {
        if( *it != 0 ) 
            delete *it;    //this recursively deletes the 
                //whole ObservationHistoryTree
        it++;
    }
/*  this is done automatically by compiler:  
    while(int s = _m_observationHistoryTreeVectors.size() > 0 )
    {
    _m_observationHistoryTreeVectors[s-1].clear();
    _m_observationHistoryTreeVectors.pop_back();
    }    
*/    
    _m_observationHistoryTreeRootPointers.clear();
    _m_observationHistoryTreeVectors.clear();
    _m_nrObservationHistories.clear();
    _m_nrObservationHistoriesT.clear(); 
    _m_firstOHIforT.clear();
}

void PlanningUnitMADPDiscrete::InitializeJointObservationHistories()
{
    if(_m_params.GetComputeJointObservationHistories())
    {
        queue<JointObservationHistoryTree*> johtQueue;
        
        Index JointObservationHistoryIndex = 0;
        Index maxLength = GetHorizon() -1 ;
        //max length of observation history o1...oh-1
        size_t nrJO = GetNrJointObservations();
        JointObservationHistory* next_joh = 0;
        JointObservationHistoryTree* next_joht = 0;
        
        //start with the initial (=empty) joint obs. hist.
        JointObservationHistory* joh_init = new JointObservationHistory(*this);
        JointObservationHistoryTree* root = new JointObservationHistoryTree(
            joh_init);
        johtQueue.push(root);
        
        Index ts = 0;
        _m_firstJOHIforT.push_back(JointObservationHistoryIndex);
        while(johtQueue.size() > 0)
        {
            JointObservationHistoryTree* joht = johtQueue.front();
            joht->SetIndex(JointObservationHistoryIndex);
            _m_jointObservationHistoryTreeVector.push_back(joht);
            
            Index thisLength = joht->GetContainedElement()->GetLength();
            if(ts !=  thisLength ) //length 0 <-> ts 0 
            {
                // we've now started processing JOHs for the next ts
                _m_nrJointObservationHistoriesT.push_back(
                    JointObservationHistoryIndex - CastLIndexToIndex(_m_firstJOHIforT[ts]));
                if(++ts !=  thisLength )
                    cerr << "ERROR: CreateJointObservationHistories: ++ts != "<<
                        "(Index) thisLength  !!! ts (now) is: "<<ts<<
                        "thisLength  =" << thisLength <<endl;
                _m_firstJOHIforT.push_back(JointObservationHistoryIndex);            
            }

            JointObservationHistoryIndex++; //index of an joint Observation history
#if DEBUG_PUDCJOHT
            cout << "PlanningUnitMADPDiscrete::"<<
                "CreateJointObservationHistoryTree - New joh tree from queue:"<<
                endl<<"\t"; joht->Print();
#endif
            
            if(thisLength < maxLength)
            {
#if DEBUG_PUDCJOHT
                cout << "PlanningUnitMADPDiscrete::"<<
                    "CreateJointObservationHistoryTree - Extending this joint "<<
                    "observation history tree..."<<endl;
#endif
                for(Index jObsI = 0; jObsI < nrJO; jObsI++) 
                {
#if DEBUG_PUDCJOHT
                    cout << "PlanningUnitMADPDiscrete::Create"
                         << "JointObservationHistoryTree - creating successor for "
                         << "joint observation "<< jObsI << endl;
#endif
                    JointObservationHistory* temp = joht->
                        GetJointObservationHistory();
#if DEBUG_PUDCJOHT
                    cout << "( history of pred was:";
                    temp->Print(); cout << ") " << endl;
#endif
                    next_joh = new JointObservationHistory(jObsI,temp);
                    next_joht = new JointObservationHistoryTree(next_joh);
                    joht->SetSuccessor(jObsI, next_joht);
                    johtQueue.push(next_joht);//this is safe - only the pointer is
                    //copied
                }
            }
            johtQueue.pop();   
        
        }
        //for the last time step, the nrJointObs are not yet added, do it now:
        _m_nrJointObservationHistoriesT.push_back
            (JointObservationHistoryIndex - CastLIndexToIndex(_m_firstJOHIforT[ts]));
        _m_nrJointObservationHistories =_m_jointObservationHistoryTreeVector.size();
        
        _m_jointObservationHistoryTreeRoot = root;

    }
    else
    {
        /*Calculates the number of joint observation histories and
         * stores this in _m_nrJointObservationHistories (and 
         * _m_nrJointObservationHistoriesT).
         * Only used when the joint observation histories are not created.
         */
        size_t nr=0;
        for (Index t=0; t<GetHorizon() ;t++ )
        {
            _m_firstJOHIforT.push_back(nr);
            size_t nrT=1;
            for (Index aI=0 ; aI<GetNrAgents(); aI++)
            {
                nrT *= _m_nrObservationHistoriesT[aI][t];
            }
            _m_nrJointObservationHistoriesT.push_back(nrT);
            nr += nrT;
        }
        _m_nrJointObservationHistories = nr;
        _m_jointObservationHistoryTreeRoot = 0;
    }
}

void PlanningUnitMADPDiscrete::DeInitializeJointObservationHistories()
{
    delete _m_jointObservationHistoryTreeRoot;//should recursively delet
    _m_jointObservationHistoryTreeVector.clear();
    _m_nrJointObservationHistories=0;
    _m_nrJointObservationHistoriesT.clear();
    _m_firstJOHIforT.clear();
}


//create / clear action histories:

void PlanningUnitMADPDiscrete::InitializeActionHistories()
{
    if(_m_params.GetComputeIndividualActionHistories())
    {
        size_t nrAgents=GetNrAgents();
        for(Index i = 0; i < nrAgents; i++)
        {
            vector<ActionHistoryTree*> * temp = new 
                vector<ActionHistoryTree*>;
            _m_actionHistoryTreeVectors.push_back(* temp );
            _m_nrActionHistoriesT.push_back(vector<size_t>());
            _m_firstAHIforT.push_back(vector<LIndex>());
            delete temp; // temp is copied into the vector - so delete this imm.
            ActionHistoryTree* treeThiAg = CreateActionHistoryTree(i);
            _m_actionHistoryTreeRootPointers.push_back(treeThiAg);
        }
    }
    else
    {
        /*Calculates the number of individual action histories and
         * stores this in _m_nrActionHistories (and 
         * _m_nrActionHistoriesT).
         * Only used when the Action histories are not created.
         */
        _m_nrActionHistories.clear();
        _m_nrActionHistoriesT.clear();
        _m_firstAHIforT.clear();
        vector<size_t> oh_per_t;
        vector<LIndex> firstAHindex_per_t;
        size_t total = 0;
        for(Index i=0; i < GetNrAgents(); i++)
        {
            size_t nrAi = GetMADPDI()->GetNrActions(i);
            oh_per_t.clear();
            firstAHindex_per_t.clear();
            total = 0;
            for(Index t=0; t < GetHorizon(); t++)
            {
                firstAHindex_per_t.push_back(total);
                size_t oh_this_t = (size_t) pow( (double)nrAi, (double)t );
                oh_per_t.push_back(oh_this_t);
                total += oh_this_t;
            }
            _m_firstAHIforT.push_back(firstAHindex_per_t);
            _m_nrActionHistories.push_back(total);
            _m_nrActionHistoriesT.push_back(oh_per_t);
        }
    }
}

//should only be called from CreateActionHistories() 
//(should be called in the order of the agents)
ActionHistoryTree* PlanningUnitMADPDiscrete::CreateActionHistoryTree(Index agentI)
{
    //we want to number the Action Histories breadth-first, so we 
    //use a queue:
    queue<ActionHistoryTree*> ohtQueue;
    
    Index ActionHistoryIndex = 0;
    int maxLength = GetHorizon() - 1;//max. action hist length = h - 1
    size_t nrO = GetNrActions(agentI);
    ActionHistory* next_oh = 0;
    ActionHistoryTree* next_oht = 0;
    
    // return a new initial (=empty) action history

    //action history never has an initial action
    //if(!GetMADPDI()->IsInitialActionEmpty())
    //    throw E("this MADP issues a non-empty initial action, but this is not yet supported by PlanningUnitMADPDiscrete::CreateActionHistoryTree");

    ActionHistory* oh = new ActionHistory(*this, agentI);
    ActionHistoryTree* root = new ActionHistoryTree(oh);//, nrO);
    ohtQueue.push(root);
    Index ts = 0;
    _m_firstAHIforT[agentI].push_back(ActionHistoryIndex);
    while(ohtQueue.size() > 0)
    {
        ActionHistoryTree* oht = ohtQueue.front();
        oht->SetIndex(ActionHistoryIndex);
        _m_actionHistoryTreeVectors[agentI].push_back(oht);
        int thisLength = oht->GetContainedElement()->GetLength(); 
        if(ts != (Index) thisLength ) //length 0 <-> ts 0 
        {
            // we've now started processing AHs for the next ts
            Index nr_AHI_for_ts = ActionHistoryIndex - 
                CastLIndexToIndex(_m_firstAHIforT[agentI][ts]);
#if DEBUG_PUDCAHT
            cout << "First ah for next ts+1: store nr of ts-AH..."<<endl<<
                "_m_firstAHIforT[agentI="<<agentI<<"][ts="<<ts<<"] = "<<
                _m_firstAHIforT[agentI][ts] << ", current AHI = " <<
                ActionHistoryIndex << endl;
            Index tempIndex = _m_nrActionHistoriesT[agentI].size();
#endif                
            _m_nrActionHistoriesT[agentI].push_back(nr_AHI_for_ts); 
#if DEBUG_PUDCAHT
            cout << "Set  _m_nrActionHistoriesT[agentI="<<agentI<<"][t="<<
                tempIndex <<" (ts="<<ts<<")] = "<< nr_AHI_for_ts <<endl;
#endif
            if(++ts != (Index) thisLength)
                cerr << "ERROR: CreateActionHistories:  ++ts != "<<
                    "(Index) thisLength !!! ts (now) is: "<<ts<<
                    "thisLength  =" << thisLength <<endl;            
            _m_firstAHIforT[agentI].push_back(ActionHistoryIndex);            
        }
        
        ActionHistoryIndex++; //index of an Action HISTORY
        
        if(thisLength < maxLength)
            for(Index obsI = 0; obsI < nrO; obsI++) //obsI = action index
            {
                next_oh = new ActionHistory(obsI, 
                    oht->GetActionHistory());
                next_oht = new ActionHistoryTree(next_oh);//, nrO);
                oht->SetSuccessor(obsI, next_oht);
                ohtQueue.push(next_oht);
            }
        ohtQueue.pop();
    }    
    //there is no next time step, so _m_nrObservationHistoriesT[agentI][lastTs]
    //have not been stored yet: do this now
    Index nr_AHI_for_ts = ActionHistoryIndex
        - CastLIndexToIndex(_m_firstAHIforT[agentI][ts]);
#if DEBUG_PUDCAHT
    cout << "Last ts end: store nr of ts=h-1 AH..."<<endl<<
        "_m_firstAHIforT[agentI="<<agentI<<"][ts="<<ts<<"] = "<<
        _m_firstAHIforT[agentI][ts] << ", current AHI = " <<
        ActionHistoryIndex << endl;
    Index tempIndex = _m_nrActionHistoriesT[agentI].size();
#endif
    _m_nrActionHistoriesT[agentI].push_back(nr_AHI_for_ts);
#if DEBUG_PUDCAHT
    cout << "Set  _m_nrActionHistoriesT[agentI="<<agentI<<"][t="<<tempIndex
         <<" (last time step ts="<<ts<<")] = "<< nr_AHI_for_ts<<endl;
#endif

    //in a horizon h MADP without initial observation, the max. action
    //sequence length is h-1 actions (and the minimum is 0).
    //(after h actions the problem is ended already - length-h action sequences
    //are not considered.)
    size_t nrActionHist;
    size_t length = GetHorizon() - 1; // = maxLength 
    nrActionHist = IndexTools::CalculateNumberOfSequences(nrO, length);
    
    if( ActionHistoryIndex != nrActionHist || 
        nrActionHist != _m_actionHistoryTreeVectors[agentI].size() )
        cerr << " WARNING:ActionHistoryIndex="<< ActionHistoryIndex <<
        " nrActionHistories=" << nrActionHist << 
        " _m_actionHistoryTreeVectors[agentI="<<agentI<<"].size()="<<
        _m_actionHistoryTreeVectors[agentI].size() << endl;
    
    _m_nrActionHistories.push_back(ActionHistoryIndex);
    
    return(root);
}

void PlanningUnitMADPDiscrete::InitializeJointActionHistories()
{
    if(_m_params.GetComputeJointActionHistories())
    {
        queue<JointActionHistoryTree*> jahtQueue;
    
        Index JointActionHistoryIndex = 0;
        int maxLength = GetHorizon() - 1 ;//max length of action history o0...oh-1
        size_t nrJA = GetNrJointActions();
        JointActionHistory* next_jah = 0;
        JointActionHistoryTree* next_jaht = 0;
        
        //start with the initial (=empty) joint obs. hist.
        JointActionHistory* jah_init = new JointActionHistory(*this);
        JointActionHistoryTree* root = new JointActionHistoryTree(
            jah_init);
        jahtQueue.push(root);
        while(jahtQueue.size() > 0)
        {
            JointActionHistoryTree* jaht = jahtQueue.front();
            jaht->SetIndex(JointActionHistoryIndex);
            _m_jointActionHistoryTreeVector.push_back(jaht);
            JointActionHistoryIndex++; //index of an joint Action history
            int thisLength = jaht->GetContainedElement()->GetLength();
            
#if DEBUG_PUDCJAHT
            cout << "PlanningUnitMADPDiscrete::CreateJointActionHistoryTree "
                 << "- New jah tree from queue:"<<endl<<"\t"; jaht->Print();
#endif
            if(thisLength < maxLength)
            {
#if DEBUG_PUDCJAHT
                cout << "PlanningUnitMADPDiscrete::CreateJointActionHistoryTree "
                     << "- Extending this joint action history tree..."<<endl;
#endif
                for(Index jAI = 0; jAI < nrJA; jAI++) //jAI = action index
                {
#if DEBUG_PUDCJAHT
                    cout << "PlanningUnitMADPDiscrete::CreateJointActionHistoryTree"
                         << " - creating successor for joint action "
                         << jAI << endl;
#endif
                    JointActionHistory* temp = jaht->
                        GetJointActionHistory();
#if DEBUG_PUDCJAHT
                    cout << "( history of pred was:";
                    temp->Print(); cout << ") " << endl;
#endif
                    next_jah = new JointActionHistory(jAI,temp);
                    next_jaht = new JointActionHistoryTree(next_jah);
                    jaht->SetSuccessor(jAI, next_jaht);
                    jahtQueue.push(next_jaht);//this is safe - only the pointer is
                    //copied
                }
            }
            jahtQueue.pop();   
            
        }
        _m_nrJointActionHistories =_m_jointActionHistoryTreeVector.size();

        _m_jointActionHistoryTreeRoot = root;
    }
    else
    {
        /*Calculates the number of joint action histories and
         * stores this in _m_nrJointActionHistories (and 
         * _m_nrJointActionHistoriesT).
         * Only used when the joint action histories are not created.
         */
        size_t nr=0;
        for (Index t=0; t<GetHorizon() ;t++ )
        {
            _m_firstJAHIforT.push_back(nr);
            size_t nrT=1;
            for (Index aI=0 ; aI<GetNrAgents(); aI++)
            {
                nrT *= _m_nrActionHistoriesT[aI][t];
            }
            _m_nrJointActionHistoriesT.push_back(nrT);
            nr += nrT;
        }
        _m_nrJointActionHistories = nr;
        _m_jointActionHistoryTreeRoot = 0;
    }
}

void PlanningUnitMADPDiscrete::DeInitializeActionHistories()
{
    vector<ActionHistoryTree *>::iterator it = 
    _m_actionHistoryTreeRootPointers.begin();
    vector<ActionHistoryTree *>::iterator last = 
    _m_actionHistoryTreeRootPointers.end();
    while(it != last)
    {
        if( *it != 0 ) 
            delete *it;    //this recursively deletes the 
                //whole ActionHistoryTree
        it++;
    }
/*  this is done automatically by compiler:  
    while(int s = _m_actionHistoryTreeVectors.size() > 0 )
    {
    _m_actionHistoryTreeVectors[s-1].clear();
    _m_actionHistoryTreeVectors.pop_back();
    }    
*/    
    _m_actionHistoryTreeRootPointers.clear();
    _m_actionHistoryTreeVectors.clear();
    _m_nrActionHistories.clear();
    _m_nrActionHistoriesT.clear(); 
    _m_firstAHIforT.clear();

}

void PlanningUnitMADPDiscrete::DeInitializeJointActionHistories()
{
    delete _m_jointActionHistoryTreeRoot;//should recursively delet
    _m_jointActionHistoryTreeVector.clear();
    _m_nrJointActionHistories=0;
    _m_nrJointActionHistoriesT.clear();
    _m_firstJAHIforT.clear();    
}

void PlanningUnitMADPDiscrete::DeInitializeActionObservationHistories()
{
    vector<ActionObservationHistoryTree *>::iterator it = 
    _m_actionObservationHistoryTreeRootPointers.begin();
    vector<ActionObservationHistoryTree *>::iterator last = 
    _m_actionObservationHistoryTreeRootPointers.end();
    while(it != last)
    {
        if( *it != 0 ) 
            delete *it;    //this recursively deletes the 
                //whole ActionObservationHistoryTree
        it++;
    }
    _m_actionObservationHistoryTreeRootPointers.clear();
    _m_actionObservationHistoryTreeVectors.clear();
    _m_nrActionObservationHistories.clear();
    _m_nrActionObservationHistoriesT.clear(); 
    _m_firstAOHIforT.clear();
}

void PlanningUnitMADPDiscrete::DeInitializeJointActionObservationHistories()
{
    delete _m_jointActionObservationHistoryTreeRoot;//should recursively delete

    _m_jointActionObservationHistoryTreeVector.clear();

    // clear the map
    if(!_m_jointActionObservationHistoryTreeMap.empty())
        delete _m_jointActionObservationHistoryTreeMap[0];
    _m_jointActionObservationHistoryTreeMap.clear();
       
    if(_m_params.GetComputeJointBeliefs())
    {
        vector<const JointBeliefInterface*>::iterator it =
            _m_jBeliefCache.begin();
        vector<const JointBeliefInterface*>::iterator last =
            _m_jBeliefCache.end();
        
        while(it != last)
        {
            delete(*it);
            it++;
        }
        _m_jBeliefCache.clear();
        _m_jaohConditionalProbs.clear();
    }

    _m_nrJointActionObservationHistories=0;
    _m_nrJointActionObservationHistoriesT.clear();
    _m_firstJAOHIforT.clear();
}

void PlanningUnitMADPDiscrete::InitializeActionObservationHistories()
{
    if(_m_params.GetComputeIndividualActionObservationHistories())
    {
        size_t nrAgents=GetNrAgents();
        for(Index i = 0; i < nrAgents; i++)
        {
            vector<ActionObservationHistoryTree*> * temp = new 
                vector<ActionObservationHistoryTree*>;
            _m_actionObservationHistoryTreeVectors.push_back(* temp );
            _m_nrActionObservationHistoriesT.push_back(vector<size_t>());
            _m_firstAOHIforT.push_back(vector<LIndex>());
            delete temp; // temp is copied into the vector - so delete this imm.
            ActionObservationHistoryTree* treeThiAg = 
                CreateActionObservationHistoryTree(i);
            _m_actionObservationHistoryTreeRootPointers.push_back(treeThiAg);
        }
    }
    else
    {
         /*Calculates the number of individual action-observation histories and
          * stores this in _m_nrActionObservationHistories (and 
          * _m_nrActionObservationHistoriesT).
          * Only used when the action-observation histories are not created.
          */
        _m_nrActionObservationHistories.clear();
        _m_nrActionObservationHistoriesT.clear();
        _m_firstAOHIforT.clear();
        vector<size_t> oh_per_t;
        vector<LIndex> firstAOHindex_per_t;
        size_t total = 0;
        for(Index i=0; i < GetNrAgents(); i++)
        {
            size_t nrAOi = GetMADPDI()->GetNrActions(i) * 
                GetMADPDI()->GetNrObservations(i);
            oh_per_t.clear();
            firstAOHindex_per_t.clear();
            total = 0;
            for(Index t=0; t < GetHorizon(); t++)
            {
                firstAOHindex_per_t.push_back(total);
                size_t oh_this_t = (size_t) pow( (double)nrAOi, (double)t );
                oh_per_t.push_back(oh_this_t);
                total += oh_this_t;
            }
            _m_firstAOHIforT.push_back(firstAOHindex_per_t);
            _m_nrActionObservationHistories.push_back(total);
            _m_nrActionObservationHistoriesT.push_back(oh_per_t);

            // need to some similar as for the JointActionObservationHistoryTree

//         vector<ActionObservationHistoryTree*> * temp = new 
//             vector<ActionObservationHistoryTree*>;
//         _m_actionObservationHistoryTreeVectors.push_back(* temp );
//         _m_nrActionObservationHistoriesT.push_back(vector<size_t>());
//         _m_firstAOHIforT.push_back(vector<size_t>());
//         delete temp; // temp is copied into the vector - so delete this imm.
//         ActionObservationHistoryTree* treeThiAg = 
//             CreateActionObservationHistoryTree(i);
//         _m_actionObservationHistoryTreeRootPointers.push_back(treeThiAg);



//         ActionObservationHistory* aoh_0 =
//             new ActionObservationHistory(*this);
//         ActionObservationHistoryTree* root =
//             new ActionObservationHistoryTree(aoh_0);
//         vector<ActionObservationHistoryTree*> *temp = new 
//             vector<ActionObservationHistoryTree*>;
//         temp
//         root->SetIndex(0);
//         _m_ActionObservationHistoryTreeVector.push_back(root);
            
        }
    }
}

ActionObservationHistoryTree* PlanningUnitMADPDiscrete::
    CreateActionObservationHistoryTree(Index agentI)
{
    queue<ActionObservationHistoryTree*> aohtQueue;
    
    Index aohI = 0;
    size_t maxLength = GetHorizon() - 1;
    size_t nrA = GetNrActions(agentI);
    size_t nrO = GetNrObservations(agentI);

    //initial ActionObservationHistory aoh_0
    ActionObservationHistory* aoh_0 =new ActionObservationHistory(*this,agentI);
    ActionObservationHistoryTree* root =new ActionObservationHistoryTree(aoh_0);

    aohtQueue.push(root);

    Index ts = 0;
    _m_firstAOHIforT[agentI].push_back(aohI);
    while(aohtQueue.size() >= 1)
    {
        //  aoh = queue.pop
        ActionObservationHistoryTree* aoht = aohtQueue.front();
        ActionObservationHistory* aoh = aoht->GetActionObservationHistory();
        aohtQueue.pop();

        int thisLength = aoh->GetLength(); 
        if(ts != (Index) thisLength  ) //length 0 <-> ts 0 
        {
            // we've now started processing AOHs for the next ts
            Index nr_AOHI_for_ts = aohI - 
                CastLIndexToIndex(_m_firstAOHIforT[agentI][ts]);
#if DEBUG_PUDCAOHT
            cout << "First aoh for next ts+1: store nr of ts-OH..."<<endl<<
                "_m_firstAOHIforT[agentI="<<agentI<<"][ts="<<ts<<"] = "<<
                _m_firstAOHIforT[agentI][ts] << ", current AOHI = " <<
                aohI << endl;
            Index tempIndex = _m_nrActionHistoriesT[agentI].size();
#endif                
            _m_nrActionObservationHistoriesT[agentI].push_back(nr_AOHI_for_ts); 
#if DEBUG_PUDCAOHT
            cout << 
                "Set _m_nrActionObservationHistoriesT[agentI="<<agentI<<"][t="<<
                tempIndex <<" (ts="<<ts<<")] = "<< nr_AOHI_for_ts <<endl;
#endif
            if(++ts != (Index) thisLength)
                cerr << "ERROR: CreateActionObservationHistories:  ++ts != "<<
                    "(Index) thisLength !!! ts (now) is: "<<ts<<
                    "thisLength-1  =" << thisLength-1 <<endl;            
            _m_firstAOHIforT[agentI].push_back(aohI);            
        }
        //  aoh.SetIndex(index++)
        aoht->SetIndex(aohI++);        
        _m_actionObservationHistoryTreeVectors[agentI].push_back(aoht);
        
        if(aoh->GetLength() >= maxLength )
            //this is a final action-obs history:
            continue;
        //else

        for(Index aI = 0; aI < nrA; aI++)
            for(Index oI = 0; oI < nrO; oI++)
            {
                //Constructor gets act. and obs. history and extends them
                //with aI, oI. to find new ones,
                ActionObservationHistory* next_aoh =
                    new ActionObservationHistory(aI, oI, aoh);
#if DEBUG_PUDCAOHT
                cout << "created new aoh (for aI="<<aI<<", oI="<<oI<<":"<<
                    endl; 
                next_aoh->Print();
                cout << endl;
#endif

                ActionObservationHistoryTree* next_aoht =
                    new ActionObservationHistoryTree(next_aoh);
#if DEBUG_PUDCAOHT
                cout << "\ncreated new aoh tree:"<<endl; 
                next_aoht->Print();
                cout << endl;
#endif

                aoht->SetSuccessor(aI, oI, next_aoht);
                aohtQueue.push(next_aoht);
            }

    }
    Index nr_AOHI_for_ts = aohI - CastLIndexToIndex(_m_firstAOHIforT[agentI][ts]);
    _m_nrActionObservationHistoriesT[agentI].push_back(nr_AOHI_for_ts);
    _m_nrActionObservationHistories.push_back(aohI);

    return(root);
}

void PlanningUnitMADPDiscrete::InitializeJointActionObservationHistories()
{
    if(_m_params.GetComputeJointActionObservationHistories())
    {
        queue<JointActionObservationHistoryTree*> jaohtQueue;    
        LIndex jaohI = 0;
        size_t maxLength = GetHorizon() - 1;
        size_t nrJA = GetNrJointActions();
        size_t nrJO = GetNrJointObservations();   
        
        //initial JointActionObservationHistory aoh_0
        JointActionObservationHistory* aoh_0 =
            new JointActionObservationHistory(*this);
        JointActionObservationHistoryTree* root =
            new JointActionObservationHistoryTree(aoh_0);
        jaohtQueue.push(root);
        
        queue<JointBeliefInterface*> jBeliefQueue;
        queue<double> cpQueue;
        queue<double> pQueue;
        const JointBeliefInterface* jb;
        bool cacheJBs=_m_params.GetComputeJointBeliefs();

        //put the initial belief and probs in the queues
        JointBeliefInterface* b0 = GetNewJointBeliefInterface(); 
        const StateDistribution* sd = GetMADPDI()->GetISD();
        b0->Set ( *sd  ); // b0 is not yet in the cache!
        jBeliefQueue.push(b0);
        cpQueue.push(1.0);
        pQueue.push(1.0);

        //while(! empty(queue) )
        Index ts = 0;
        _m_firstJAOHIforT.push_back(0);//ts=0 starts at index 0
        while(jaohtQueue.size() >= 1)
        {
            //  jaoh = queue.pop
            JointActionObservationHistoryTree* jaoht = jaohtQueue.front();
            JointActionObservationHistory* jaoh = jaoht->
                GetJointActionObservationHistory();
            jaohtQueue.pop();
        
            Index thisLength = jaoh//t->GetContainedElement()
                ->GetLength();
            if(ts !=  thisLength ) //length 0 <-> ts 0 
            {
                // we've now started processing JOHs for the next ts
                _m_nrJointActionObservationHistoriesT.push_back(
                    CastLIndexToIndex(jaohI - _m_firstJAOHIforT[ts]));
                if(++ts !=  thisLength )
                    cerr << "ERROR: CreateJointObservationHistories: ++ts != "<<
                        "thisLength  !!! ts (now) is: "<<ts<<
                        "thisLength =" << thisLength <<endl;
                _m_firstJAOHIforT.push_back(jaohI);            
            }

            //  jaoh.setIndex(index++)
            jaoht->SetIndex(jaohI++);        
            _m_jointActionObservationHistoryTreeVector.push_back(jaoht);
            
            double prob = pQueue.front();
            _m_jaohProbs.push_back(prob);
            pQueue.pop();
            
            double cprob = cpQueue.front();
            _m_jaohConditionalProbs.push_back(cprob);
            cpQueue.pop();
            jb = jBeliefQueue.front();
            jBeliefQueue.pop();
            if(cacheJBs) //if we cache the joint beliefs... cache it!
                _m_jBeliefCache.push_back(jb);
        
            if(jaoh->GetLength() >= maxLength )
            {
                //this is a final action-obs history: clean up...
                if(!cacheJBs) //we no longer need jb
                    delete jb;
                //... and
                continue;
            }
            //else

            for(Index jaI = 0; jaI < nrJA; jaI++)
                for(Index joI = 0; joI < nrJO; joI++)
                {
                    //Constructor gets act. and obs. history and extends them
                    //with jaI, joI. to find new ones,
                    JointActionObservationHistory* next_jaoh =
                        new JointActionObservationHistory(jaI, joI, jaoh);
                    JointActionObservationHistoryTree* next_jaoht =
                        new JointActionObservationHistoryTree(next_jaoh);
#if DEBUG_PUDCJAOHT
                    cout << "created new jaoh (for jaI="<<jaI<<", joI="<<joI<<
                        ":"<< endl; next_jaoh->Print(); cout << endl;
                    cout << "\ncreated new jaoh tree:"<<endl; next_jaoht->Print();
                    cout << endl;
#endif

                    jaoht->SetSuccessor(jaI, joI, next_jaoht);
                    jaohtQueue.push(next_jaoht);
                    
                    JointBeliefInterface* new_jb = GetNewJointBeliefInterface();
                    *new_jb = *jb;
                    double new_cond_p = new_jb->Update(*GetMADPDI(),jaI, joI);
                    jBeliefQueue.push(new_jb);
                    cpQueue.push(new_cond_p);
                    double new_p = prob * new_cond_p;
                    pQueue.push(new_p);
                }

            if(!cacheJBs) //we no longer need jb
                delete jb;
            
        }
        _m_nrJointActionObservationHistories = CastLIndexToIndex(jaohI);
        
        _m_jointActionObservationHistoryTreeRoot = root;
    }
    else
    {
        /*Calculates the number of joint action-observation histories and
         * stores this in _m_nrJointActionObservationHistories (and
         * _m_nrJointActionObservationHistoriesT).  Only used when the joint
         * act-observation histories are not created. 
         *
         * !Also initializes the
         * root of JointActionObservationHistoryTree vector, for on-the-fly
         * generation of successors.
         */
        size_t nr=0;
        for (Index t=0; t<GetHorizon() ;t++ )
        {
            LIndex firstIndex = nr;
            _m_firstJAOHIforT.push_back(firstIndex);
            size_t nrT=1; //the number of histories at 
            for (Index aI=0 ; aI<GetNrAgents(); aI++)
            {
                size_t historiesThisAgent =  
                    _m_nrObservationHistoriesT[aI][t] *  
                        _m_nrActionHistoriesT[aI][t];
                nrT *= historiesThisAgent;
            }
            _m_nrJointActionObservationHistoriesT.push_back(nrT);
            nr += nrT;
        }
        _m_nrJointActionObservationHistories = nr;
        
        JointActionObservationHistory* aoh_0 =
            new JointActionObservationHistory(*this);
        JointActionObservationHistoryTree* root =
            new JointActionObservationHistoryTree(aoh_0);

        root->SetIndex(0);
        _m_jointActionObservationHistoryTreeMap[0]=root;

        _m_jointActionObservationHistoryTreeRoot = 0;
    }
}


// GET functions:
//

size_t PlanningUnitMADPDiscrete::GetNrStates() const 
{
    const MultiAgentDecisionProcessDiscreteInterface* p = GetMADPDI();
    size_t nrs = p->GetNrStates();
    return (nrs);
}

LIndex PlanningUnitMADPDiscrete::GetNrPolicies(Index agentI) const
{
#if 0 // don't use powl(), we want to detect overflow
    return( static_cast<LIndex>(powl( GetNrActions(agentI), 
                                      GetNrObservationHistories(agentI))));
#endif
    LIndex nrPols=1;
    for(size_t k=0;k!=GetNrObservationHistories(agentI);++k)
    {
        LIndex nrPolsOld=nrPols;
        nrPols*=GetNrActions(agentI);
        if(nrPols<nrPolsOld)
            throw(EOverflow("PlanningUnitMADPDiscrete::GetNrPolicies() overflow detected"));
    }
    return(nrPols);
}

LIndex PlanningUnitMADPDiscrete::GetNrJointPolicies() const
{
    LIndex n=1;
    for(Index ag=0; ag < GetNrAgents(); ag++)
    {
        LIndex nOld=n;
        n *= GetNrPolicies(ag);
        if(n<nOld)
            throw(EOverflow("PlanningUnitMADPDiscrete::GetNrJointPolicies() overflow detected"));
    }
    return(n);
}

size_t PlanningUnitMADPDiscrete::GetNrObservationHistories(Index agentI) const
{
    if(agentI < _m_nrObservationHistories.size() )
        return(_m_nrObservationHistories.at(agentI));
    //else
    throw(E("ERROR PlanningUnitMADPDiscrete::GetNrObservationHistories(Index agentI) index out of bounds or _m_nrObservationHistories not yet initialized?!\n"));
} 

const vector<size_t> PlanningUnitMADPDiscrete::GetNrObservationHistoriesVector() const
{
    vector<size_t> nrOHts; 
    for(Index agentI = 0; agentI < GetNrAgents(); agentI++)
        nrOHts.push_back( GetNrObservationHistories(agentI) );
    return nrOHts;
}

const vector<size_t> PlanningUnitMADPDiscrete::GetNrObservationHistoriesVector(Index ts) const
{
    vector<size_t> nrOHts; 
    for(Index agentI = 0; agentI < GetNrAgents(); agentI++)
        nrOHts.push_back( GetNrObservationHistories(agentI, ts) );
    return nrOHts;
}

ObservationHistoryTree* PlanningUnitMADPDiscrete::GetObservationHistoryTree(Index agentI, Index ohI) const
{
    if(!_m_params.GetComputeIndividualObservationHistories())
        throw ENotCached("PlanningUnitMADPDiscrete::GetObservationHistoryTree IndividualObservationHistories are not cached!");

    size_t nrA = GetNrAgents();
    if(_m_observationHistoryTreeVectors.size() != nrA)
        throw E("_m_observationHistoryTreeVectors.size() != nrA)");

    if(agentI < nrA )
    {
    size_t nrOH = GetNrObservationHistories(agentI);
if(DEBUG_PUD){cout << "_m_observationHistoryTreeVectors["<<agentI<<"].size() = " <<_m_observationHistoryTreeVectors[agentI].size()<<endl;}
    if(_m_observationHistoryTreeVectors[agentI].size() != nrOH)
        throw E("_m_observationHistoryTreeVectors[agentI].size() != nrOH)");

    if(ohI < nrOH )
    {
if(DEBUG_PUD){ cout <<"accessing _m_observationHistoryTreeVectors["<<    agentI<<"]["<<ohI<<"]"<<endl;}

        ObservationHistoryTree* oht = 
            _m_observationHistoryTreeVectors.at(agentI).at(ohI);
if(DEBUG_PUD){ cout <<"...done - returning "<<oht<<endl;}
        return (oht);
    }
    else
        cerr << "WARNING PlanningUnitMADPDiscrete::GetObservationHistory(Index agentI, Index a) index ohI out of bounds"<<endl;
    }
    else
    cerr << "WARNING PlanningUnitMADPDiscrete::GetObservationHistory(Index agentI, Index ohI) index agentI out of bounds"<<endl;   
    
    return(0);
   
}
JointObservationHistoryTree* PlanningUnitMADPDiscrete::
    GetJointObservationHistoryTree(Index johI) const
{
    if(johI < _m_nrJointObservationHistories)
        return(_m_jointObservationHistoryTreeVector.at(johI));
    else
    {
    cerr << "PlanningUnitMADPDiscrete::GetJointObservationHistoryTree - Warning: "<<
        "index (" << johI <<")out of bound!" << endl;
    return(0);
    }
}




//-------------------------------------------------
//END of "Get...HistoryArrays/Vectors()" functions
//-------------------------------------------------

void 
PlanningUnitMADPDiscrete::ComputeHistoryArrays
    (
        Index hI, //index of the history
        Index t,  //stage of hI
        Index t_offset, //the offset within t
        Index Indices[], //the computed individual indices
        size_t indexDomainSize //the indices range from 0...(indexDomainSize-1)

    ) const
{
    //let's hope the compiler optimizes out the obvious inefficientcies here
    //(I left them for readability)

    //substract the offset to find out what is the index within the 
    //time-step-t-histories.
    Index hI_withinT = hI - t_offset;   
    Index remainder = hI_withinT;
    size_t vec_size = t; // the number of indices  we are searching
    size_t nrO = indexDomainSize;
    for(Index t2=0; t2 < vec_size ; t2++)
    {
        // suppose we have a oh = < oI[3], oI[2], o[1], oI[0] >
        // (the numbers represent the stage of the oI)
        // ->then this history is for stage 4 (we have that t=4).
        //
        // the ohI is generated by: 
        //      ohI= oI[3] * nrO^3 + ... +oI[0] * nrO^0
        //
        // we reconstruct this by performing
        // oI[3] = ohI /  nrO^3
        // remainder = ohI - (oI^3)
        // oI[2] = remainder / nrO^2
        // etc.
        //
        // we start with stage
        size_t stage = (t-1) - t2;
        //we are going to calculate the joa that took place at time stage
        size_t b = (size_t) pow((double)nrO, (double)(stage));
        Index oI_t2 = remainder / b;
        Indices[t2] = oI_t2;
        remainder -= oI_t2 * b;
    }
}

void
PlanningUnitMADPDiscrete::GetJointActionObservationHistoryVectors
    (LIndex jaohI, vector<Index> &jaIs, vector<Index> &joIs) const
{
    jaIs.clear();
    joIs.clear();

    JointActionObservationHistoryTree* jaoht = 
        GetJointActionObservationHistoryTree(jaohI);
    // if it is not cached, jaoht will be set to 0
    if(jaoht)
    {
        jaoht->GetJointActionObservationHistory()->
            GetJointActionObservationHistoryVectors(jaIs, joIs);
    }
    else
    {    
        Index t = GetTimeStepForJAOHI(jaohI);
        Index jaIs_arr[t];
        Index joIs_arr[t];
        GetJointActionObservationHistoryArrays(jaohI, t, jaIs_arr, joIs_arr );
        for(size_t t2 = 0; t2 < t; t2++)
        {
            jaIs.push_back(jaIs_arr[t2]);
            joIs.push_back(joIs_arr[t2]);
        }
    }
}

void 
PlanningUnitMADPDiscrete::GetJointActionObservationHistoryArrays
    (LIndex jaohI, Index t,  Index jaIs[], Index joIs[]) const
{
    //substract the offset to find out what is the index within the time-step-t-
    //observation-histories.
    LIndex jaohIwithinThisT = jaohI - _m_firstJAOHIforT.at(t);
   
    LIndex remainder = jaohIwithinThisT;
    size_t vec_size = t; // the number of 'JAO' we are searching
        //note: this was t+1, but that is incorrect(?)!
        
    size_t nrJO = GetNrJointObservations();
    size_t nrJA = GetNrJointActions();
    size_t nrJAO = nrJO*nrJA;
//    Index jao_array[vec_size];
    for(Index t2=0; t2 < vec_size ; t2++)
    {
        // suppose we have a jaoh = < jaoI[3], jaoI[2], jao[1], jaoI[0] >
        // (the numbers represent the stage of the jaoI)
        // ->then this history is for stage 4 (we have that t=4).
        //
        // the jaohI is generated by: 
        //      jaohI= jaoI[3] * nrJAO^3 + ... +jaoI[0] * nrJAO^0
        //
        // we reconstruct this by performing
        // jaoI[3] = jaohI /  nrJAO^3
        // remainder = jaohI - (jaoI^3)
        // jaoI[2] = remainder / nrJAO^2
        // etc.
        //
        // we start with stage
        size_t stage = (t-1) - t2;
        //we are going to calculate the joa that took place at time stage
        size_t b = (size_t) pow((double)nrJAO, (double)(stage));

        LIndex jaoI_t2 = remainder / b;
//        jao_array[t2] = jaoI_t2;
        remainder -= jaoI_t2 * b;

        jaIs[t2] = IndexTools::
            ActionObservation_to_ActionIndex(CastLIndexToIndex(jaoI_t2), nrJA, nrJO);
        joIs[t2] = IndexTools::
            ActionObservation_to_ObservationIndex(CastLIndexToIndex(jaoI_t2), nrJA, nrJO);
    }

}

void 
PlanningUnitMADPDiscrete::GetJointObservationHistoryArrays
    (Index johI, Index t,  Index joIs[]) const
{
    //substract the offset to find out what is the index within the time-step-t-
    //observation-histories.
    Index johIwithinThisT = johI - CastLIndexToIndex(_m_firstJOHIforT.at(t));
   
    Index remainder = johIwithinThisT;
    size_t vec_size = t; // the number of joint observations we are searching
    size_t nrJO = GetNrJointObservations();
    //size_t nrJA = GetNrJointActions();
    for(Index t2=0; t2 < vec_size ; t2++)
    {
        // for explanation see GetJointActionObservationHistoryArrays()
        //
        // we start with stage
        size_t stage = (t-1) - t2;
        //we are going to calculate the joa that took place at time stage
        size_t b = (size_t) pow((double)nrJO, (double)(stage));
        Index joI_t2 = remainder / b;
        joIs[t2] = joI_t2; 
        remainder -= joI_t2 * b;
    }
}

void 
PlanningUnitMADPDiscrete::GetActionObservationHistoryArrays
    (Index agentI, Index aohI, Index t,  Index aIs[], Index oIs[]) const
{
    //substract the offset to find out what is the index within the time-step-t-
    //observation-histories.
    Index aohIwithinThisT = aohI - CastLIndexToIndex(_m_firstAOHIforT.at(agentI).at(t));
   
    Index remainder = aohIwithinThisT;
    size_t vec_size = t; // the number of 'JAO' we are searching
        
    size_t nrO = GetNrObservations(agentI);
    size_t nrA = GetNrActions(agentI);
    size_t nrAO = nrO*nrA;
    for(Index t2=0; t2 < vec_size ; t2++)
    {
        // for explanation see GetJointActionObservationHistoryArrays()
        //
        // we start with stage
        size_t stage = (t-1) - t2;
        //we are going to calculate the joa that took place at time stage
        size_t b = (size_t) pow((double)nrAO, (double)(stage));

        Index aoI_t2 = remainder / b;
        remainder -= aoI_t2 * b;

        aIs[t2] = IndexTools::
            ActionObservation_to_ActionIndex(aoI_t2, nrA, nrO);
        oIs[t2] = IndexTools::
            ActionObservation_to_ObservationIndex(aoI_t2, nrA, nrO);
    }

}

void 
PlanningUnitMADPDiscrete::GetObservationHistoryArrays
    (Index agentI, Index ohI, Index t,  Index oIs[]) const
{
    //substract the offset to find out what is the index within the time-step-t-
    //observation-histories.
    Index ohIwithinThisT = ohI - CastLIndexToIndex(_m_firstOHIforT.at(agentI).at(t));
   
    Index remainder = ohIwithinThisT;
    size_t vec_size = t; // the number of joint observations we are searching
    size_t nrO = GetNrObservations(agentI);
    //size_t nrJA = GetNrJointActions();
    for(Index t2=0; t2 < vec_size ; t2++)
    {
        // for explanation see GetJointActionObservationHistoryArrays()
        //
        // we start with stage
        size_t stage = (t-1) - t2;
        //we are going to calculate the joa that took place at time stage
        size_t b = (size_t) pow((double)nrO, (double)(stage));
        Index oI_t2 = remainder / b;
        oIs[t2] = oI_t2; 
        remainder -= oI_t2 * b;
    }
}

void 
PlanningUnitMADPDiscrete::GetActionHistoryArrays
    (Index agentI, Index ahI, Index t,  Index aIs[]) const
{
    //substract the offset to find out what is the index within the time-step-t-
    //action-histories.
    Index ahIwithinThisT = ahI - CastLIndexToIndex(_m_firstAHIforT.at(agentI).at(t));
   
    Index remainder = ahIwithinThisT;
    size_t vec_size = t; // the number of joint actions we are searching
    size_t nrA = GetNrActions(agentI);
    for(Index t2=0; t2 < vec_size ; t2++)
    {
        // for explanation see GetJointActionActionHistoryArrays()
        //
        // we start with stage
        size_t stage = (t-1) - t2;
        //we are going to calculate the joa that took place at time stage
        size_t b = (size_t) pow((double)nrA, (double)(stage));
        Index aI_t2 = remainder / b;
        aIs[t2] = aI_t2; 
        remainder -= aI_t2 * b;
    }
}

//-------------------------------------------------
//END of "Get...HistoryArrays/Vectors()" functions
//-------------------------------------------------






//-----------------------------------------
//BEGIN of "Get...HistoryIndex()" functions
//-----------------------------------------


Index PlanningUnitMADPDiscrete::GetJointObservationHistoryIndex(
        JointObservationHistoryTree* joh) const
{
    throw E("GetJointObservationHistoryIndex(JointObservationHistoryTree* joh) seems like a dumb-ass function? I mean looping over exponentially many histories does not seem like a nice thing to do?");

    for(Index i=0;i<_m_nrJointObservationHistories;i++)
    {
        if(_m_jointObservationHistoryTreeVector.at(i)==joh)
            return(i);
    }
    // we should not get here
    throw E("GetJointObservationHistoryIndex index not found");
    return(0);
}

LIndex PlanningUnitMADPDiscrete::GetJointActionObservationHistoryIndex(
        JointActionObservationHistoryTree* jaoh) const
{
    throw E("GetJointActionObservationHistoryIndex(JointActionObservationHistoryTree* joh) seems like a dumb-ass function? I mean looping over exponentially many histories does not seem like a nice thing to do?");
    for(Index i=0;i<_m_nrJointActionObservationHistories;i++)
    {
        if(_m_jointActionObservationHistoryTreeVector.at(i)==jaoh)
            return(i);
    }
    // we should not get here
    throw E("GetJointObservationHistoryIndex index not found");
    return(0);
}


Index PlanningUnitMADPDiscrete::ComputeHistoryIndex(
        Index t,    //the stage
        Index t_offset, //for each stage t, there is an offset
        //const vector<Index>& indices, //the indices of the (J)(A/O)s
        const Index indices [], //the indices of the (J)(A/O)s
        size_t indexDomainSize //the indices range from 0...(indexDomainSize-1)
        ) const
{
    //This example assumes we are computing the index of an observation history:
    //
    //indices is a vector with the observation indices for stage 0...t-1
    //<o^0, o^1, ... , o^(t-1) >
    //from this we want to find the history observation index, which is given
    //by:
    //      o^0 * |O|^(t-1) + ... + o^(t-1) * |O|^0     + stage offset
    //
    //where |O| is the `base' of the number as specified by the tuple of 
    //indices
    //
    size_t HI=t_offset;
    for (Index ts=0; ts<t; ts++)
    {
        int exponent = t-1 - ts;
        size_t b = (size_t) pow((double)indexDomainSize, (double)(exponent));
        HI += indices[ts] * b;
    }
    return(HI);
}


Index PlanningUnitMADPDiscrete::GetObservationHistoryIndex(Index agentI, 
        Index t, const vector<Index>& observations) const
{
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(_m_firstOHIforT.at(agentI).at(t)), 
                &observations[0],
                GetNrObservations(agentI)
                )
          );

    //observations is a vector with the observation indices for stage 0...t-1
    //<o^0, o^1, ... , o^(t-1) >
    //from this we want to find the history observation index, which is given
    //by:
    //      o^0 * |O|^(t-1) + ... + o^(t-1) * |O|^0     + stage offset
    //
    //where |O| is the `base' of the number as specified by the tuple of 
    //indices
    //
    //
    //size_t this_OHI=_m_firstOHIforT.at(agentI).at(t);
    //size_t nrO = GetNrObservations(agentI);
    //for (Index ts=0; ts<t; ts++)
    //{
        //int exponent = t-1 - ts;
        //size_t b = (size_t) pow((double)nrO, (double)(exponent));
        //this_OHI += observations.at(ts) * b;
    //}
    //return(this_OHI);
}

Index PlanningUnitMADPDiscrete::GetJointObservationHistoryIndex(
        Index t, const vector<Index>& jointObservations) const
{
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(_m_firstJOHIforT.at(t)), 
                &jointObservations[0],
                GetNrJointObservations()
                )
          );    
/*
    size_t this_JOHI=_m_firstJOHIforT.at(t);
    size_t nrJO = GetNrJointObservations();
    for (Index ts=0; ts<t; ts++)
    {
        int exponent = t-1 - ts;
        size_t b = (size_t) pow((double)nrJO, (double)(exponent));
        this_JOHI += jointObservations.at(ts) * b;
    }
    return(this_JOHI);*/
}
Index PlanningUnitMADPDiscrete::GetJointObservationHistoryIndex(
        Index t, const Index jointObservations[] ) const
{
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(_m_firstJOHIforT.at(t)), 
                jointObservations,
                GetNrJointObservations()
                )
          );    
/*
    size_t this_JOHI=_m_firstJOHIforT.at(t);
    size_t nrJO = GetNrJointObservations();
    for (Index ts=0; ts<t; ts++)
    {
        int exponent = t-1 - ts;
        size_t b = (size_t) pow((double)nrJO, (double)(exponent));
        this_JOHI += jointObservations[ts] * b;
    }
    return(this_JOHI);*/
}

Index PlanningUnitMADPDiscrete::GetJointActionHistoryIndex(
        Index t, const vector<Index>& jointActions) const
{
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(_m_firstJAHIforT.at(t)), 
                &jointActions[0],
                GetNrJointActions()
                )
          );    
/*
    size_t this_JAHI=_m_firstJAHIforT.at(t);
    size_t nrJA = GetNrJointActions();
    for (Index ts=0; ts<t; ts++)
    {
        int exponent = t-1 - ts;
        size_t b = (size_t) pow((double)nrJA, (double)(exponent));
        this_JAHI += jointActions.at(ts) * b;
    }
    return(this_JAHI);*/
}
Index PlanningUnitMADPDiscrete::GetJointActionHistoryIndex(
        Index t, const Index jointActions[] ) const
{
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(_m_firstJAHIforT.at(t)), 
                jointActions,
                GetNrJointActions()
                )
          );    
    /*size_t this_JAHI=_m_firstJAHIforT.at(t);
    size_t nrJA = GetNrJointActions();
    for (Index ts=0; ts<t; ts++)
    {
        int exponent = t-1 - ts;
        size_t b = (size_t) pow((double)nrJA, (double)(exponent));
        this_JAHI += jointActions[ts] * b;
    }
    return(this_JAHI);*/
}

Index PlanningUnitMADPDiscrete::GetActionObservationHistoryIndex(
        Index agentI, 
        Index t, 
        const vector<Index>& actions,
        const vector<Index>& observations
        ) const
{
    //functioning is the same as for observation histories: only now the 
    //base is the nr AO.

    //-> first convert to AO indices:
    size_t first_AOHI=CastLIndexToIndex(_m_firstAOHIforT.at(agentI).at(t));
    size_t nrA = GetNrActions(agentI);
    size_t nrO = GetNrObservations(agentI);
    size_t nrAO = nrA * nrO;
    vector<Index> AO_Is(t, 0);
    for (Index ts=0; ts<t; ts++)
        AO_Is[ts] =  IndexTools::ActionAndObservation_to_ActionObservationIndex(
                    actions.at(ts), observations.at(ts), nrA, nrO);
    return( ComputeHistoryIndex(
                t, 
                first_AOHI,
                &AO_Is[0],
                nrAO
                )
          );    

    /*for (Index ts=0; ts<t; ts++)
    {
        int exponent = t-1 - ts;
        size_t b = (size_t) pow((double)nrAO, (double)(exponent));
        Index this_t_ao = IndexTools::
            ActionAndObservation_to_ActionObservationIndex(
                    actions.at(ts), observations.at(ts), nrA, nrO);
        this_AOHI += this_t_ao * b;
    }
    return(this_AOHI);*/
}

Index PlanningUnitMADPDiscrete::GetJointActionObservationHistoryIndex(
        Index t, 
        const vector<Index>& Jactions,
        const vector<Index>& Jobservations
        ) const
{
    //functioning is the same as for observation histories:
    //(only now the base is the nr AO)

    LIndex first_JAOHI=_m_firstJAOHIforT.at(t);
    size_t nrJA = GetNrJointActions();
    size_t nrJO = GetNrJointObservations();
    size_t nrJAO = nrJA * nrJO;
 
    vector<Index> JAO_Is(t, 0);
    for (Index ts=0; ts<t; ts++)
        JAO_Is[ts] = IndexTools::ActionAndObservation_to_ActionObservationIndex(
                    Jactions.at(ts), Jobservations.at(ts), nrJA, nrJO);
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(first_JAOHI),
                &JAO_Is[0],
                nrJAO
                )
          );    
}

Index PlanningUnitMADPDiscrete::GetJointActionObservationHistoryIndex(
        Index t, 
        const vector<Index>& Jactions,
        const vector<Index>& Jobservations,
        const Scope& agentSope
        ) const
{
    //functioning is the same as for observation histories:
    //(only now the base is the nr AO)

    LIndex first_JAOHI=_m_firstJAOHIforT.at(t);
    size_t nrJA = GetNrJointActions();
    size_t nrJO = GetNrJointObservations();
    size_t nrJAO = nrJA * nrJO;
 
    vector<Index> JAO_Is(t, 0);
    for (Index ts=0; ts<t; ts++)
        JAO_Is[ts] = IndexTools::ActionAndObservation_to_ActionObservationIndex(
                    Jactions.at(ts), Jobservations.at(ts), nrJA, nrJO);
    return( ComputeHistoryIndex(
                t, 
                CastLIndexToIndex(first_JAOHI),
                &JAO_Is[0],
                nrJAO
                )
          );    
}
        
//-----------------------------------------
//END of "Get...HistoryIndex()" functions
//-----------------------------------------



//-----------------------------------------
//BEGIN of "JointToIndividual...HistoryIndices...()" functions
//-----------------------------------------

vector<Index>
PlanningUnitMADPDiscrete::JointToIndividualObservationHistoryIndices(Index
        johI) const
{
    Index t = GetTimeStepForJOHI(johI);
    Index jo_array[t];
    GetJointObservationHistoryArrays(johI, t, jo_array );

    //convert joint observations -> indiv. observations
    vector< vector<Index> > indivO_vec;
    
    for (Index agentI=0; agentI<GetNrAgents(); agentI++)
        indivO_vec.push_back( vector<Index>() );

    for (Index ts=0; ts<t; ts++)
    {
        vector<Index> indivIndicesThisT = 
            JointToIndividualObservationIndices(jo_array[ts]);
        for (Index agentI=0; agentI<GetNrAgents(); agentI++)
            indivO_vec.at(agentI).push_back( indivIndicesThisT.at(agentI) );
    }
            
    //calculate indiv observation history index from seq. of indiv. observations
    vector<Index> ohI_vec;
    for (Index agentI=0; agentI<GetNrAgents(); agentI++)
    {
        Index this_agentOHI = GetObservationHistoryIndex(agentI, t, 
                                                     indivO_vec.at(agentI) );
        ohI_vec.push_back(this_agentOHI);
    }
    return(ohI_vec);
}

const vector<Index>&
PlanningUnitMADPDiscrete::JointToIndividualObservationHistoryIndicesRef(Index
        jObsHistI) const
{
    if(jObsHistI < _m_nrJointObservationHistories)
    {
        JointObservationHistoryTree* joht = 
            _m_jointObservationHistoryTreeVector.at(jObsHistI);
        return(joht->GetJointObservationHistory()->
                GetIndividualObservationHistoryIndices());
    }
    else
    {
        stringstream ss;
        ss << "PlanningUnitMADPDiscrete::JointToIndividualObservationHistory"<<
           "Indices ERROR: index ("<< jObsHistI <<") out of bounds!"<<endl;
        throw E(ss); 
    }
}

const vector<Index>&
PlanningUnitMADPDiscrete::JointToIndividualActionObservationHistoryIndicesRef(
        LIndex jaohI) const
{    
    if(jaohI < _m_nrJointActionObservationHistories)
    {
        JointActionObservationHistoryTree* jaoht = 
            _m_jointActionObservationHistoryTreeVector[CastLIndexToIndex(jaohI)];
        return(jaoht->GetJointActionObservationHistory()->
                GetIndividualActionObservationHistoryIndices());
    }
    else
    {
        stringstream ss;
        ss << "PlanningUnitMADPDiscrete::JointToIndividualActionActObserva"<<
            "tionHistory Indices ERROR: index ("<< jaohI <<
            ") out of bounds!"<<endl;
        throw E(ss); 
    }
}

void
PlanningUnitMADPDiscrete::JointAOHIndexToIndividualActionObservationVectors(
        LIndex jaohI,
        vector< vector<Index> >& indivAO_vec
        ) const
{
    throw E("PlanningUnitMADPDiscrete::JointAOHIndexToIndividualActionObservationVectors(Index jaohI, vector< vector<Index> >& indivAO_vec ) const is not yet implemented.");

}

void
PlanningUnitMADPDiscrete::JointAOHIndexToIndividualActionObservationVectors(
        LIndex jaohI,
        vector< vector<Index> >& indivO_vec,
        vector< vector<Index> >& indivA_vec
        ) const
{
    Index t = GetTimeStepForJAOHI(jaohI);
    Index jo_array[t];
    Index ja_array[t];

    GetJointActionObservationHistoryArrays(jaohI, t, ja_array, jo_array);      

    size_t nrAgents = GetNrAgents();
    //convert joint observations -> indiv. observations
    //indivO_vec[agentI][t] = oI
    indivO_vec = vector< vector<Index> >(nrAgents, vector<Index>(t, 0));
    indivA_vec = vector< vector<Index> >(nrAgents, vector<Index>(t, 0));
    for (Index ts=0; ts<t; ts++)
    {
        vector<Index> indivA_IndicesThisT = 
            JointToIndividualActionIndices(ja_array[ts]);
        vector<Index> indivO_IndicesThisT = 
            JointToIndividualObservationIndices(jo_array[ts]);
        for (Index agentI=0; agentI<GetNrAgents(); agentI++)
        {
            indivO_vec[agentI][ts] = indivO_IndicesThisT[agentI];
            indivA_vec[agentI][ts] = indivA_IndicesThisT[agentI];
        }
    }
}
            
vector<Index> 
PlanningUnitMADPDiscrete::JointToIndividualActionObservationHistoryIndices(
        LIndex jaohI) const
{
    Index t = GetTimeStepForJAOHI(jaohI);
    //indivO_vec[agentI][t] = oI
    size_t nrAgents = GetNrAgents();
    vector< vector<Index> > indivO_vec(nrAgents, vector<Index>(t, 0));
    vector< vector<Index> > indivA_vec(nrAgents, vector<Index>(t, 0));
    JointAOHIndexToIndividualActionObservationVectors(
            jaohI, indivO_vec, indivA_vec );
    //calculate indiv observation history index from seq. of indiv. observations
    vector<Index> aohI_vec;
    for (Index agentI=0; agentI<GetNrAgents(); agentI++)
    {
        Index this_agentAOHI = GetActionObservationHistoryIndex(agentI, t, 
                            indivA_vec.at(agentI), indivO_vec.at(agentI) );
        aohI_vec.push_back(this_agentAOHI);
    }
    return(aohI_vec);
}

vector<Index> PlanningUnitMADPDiscrete::JointToIndividualActionHistoryIndices(
        Index jObsHistI) const
{
    throw E("PlanningUnitMADPDiscrete::JointToIndividualActionHistoryIndices(Index jObsHistI) NOT YET IMPLEMENTED");

}

const vector<Index>& PlanningUnitMADPDiscrete::JointToIndividualActionHistoryIndicesRef(Index jObsHistI) const
{
    if(jObsHistI < _m_nrJointActionHistories)
    {
        JointActionHistoryTree* joht = 
            _m_jointActionHistoryTreeVector[jObsHistI];
        return(joht->GetJointActionHistory()->
                GetIndividualActionHistoryIndices());
    }
    else
    {
        stringstream ss;
        ss << "PlanningUnitMADPDiscrete::JointToIndividualActionHistoryIndices"
           <<" ERROR: index ("<< jObsHistI <<") out of bounds!"<<endl;
        throw E(ss);
    }
}


//-----------------------------------------
//END of "JointToIndividual...HistoryIndices...()" functions
//-----------------------------------------

vector<Index> 
PlanningUnitMADPDiscrete::
JointToIndividualPolicyDomainIndices(Index jdI,
                                     PolicyGlobals::PolicyDomainCategory cat) const
{
    switch ( cat )
    {
    case PolicyGlobals::OHIST_INDEX :
        return JointToIndividualObservationHistoryIndices(jdI);
        break;
    case PolicyGlobals::AOHIST_INDEX :
        return JointToIndividualActionObservationHistoryIndices(jdI);
        break;
    case PolicyGlobals::TYPE_INDEX : 
        throw(E("JointToIndividualPolicyDomainIndices: types are not a valid policy domain for a PlanningUnitMADPDiscrete"));
        break;
    default:
        throw(E("JointToIndividualPolicyDomainIndices: incorrect JointPolicyDiscrete::PolicyGlobals::PolicyDomainCategory"));
        break;
    }
}

const vector<Index>& PlanningUnitMADPDiscrete::JointToIndividualPolicyDomainIndicesRef( 
    Index jdI, PolicyGlobals::PolicyDomainCategory cat) const
{
    switch ( cat )
    {
    case PolicyGlobals::OHIST_INDEX :
        return JointToIndividualObservationHistoryIndicesRef(jdI);
        break;
    case PolicyGlobals::AOHIST_INDEX :
        return JointToIndividualActionObservationHistoryIndicesRef(jdI);
        break;
    case PolicyGlobals::TYPE_INDEX : 
        throw(E("JointToIndividualPolicyDomainIndicesRef: types are not a valid policy domain for a PlanningUnitMADPDiscrete"));
        break;
    default:
        throw(E("JointToIndividualPolicyDomainIndicesRef: incorrect JointPolicyDiscrete::PolicyGlobals::PolicyDomainCategory"));
        break;
    }
}




//-----------------------------------------
//BEGIN of "IndividualToJoint...HistoryIndex()" functions
//-----------------------------------------


Index 
PlanningUnitMADPDiscrete::IndividualToJointObservationHistoryIndex(Index t, 
        const vector<Index>& indivIs) 
const
{
    //individual observation history index -> individual observation sequences
    size_t nrAg = GetNrAgents();
    vector< Index* > observSeqs(nrAg);
    for (Index agentI = 0; agentI < nrAg; agentI++)
    {
        observSeqs[agentI] = new Index[t];
        Index ohI_thisAgent = indivIs.at(agentI);
        GetObservationHistoryArrays (agentI, ohI_thisAgent, t, 
                observSeqs[agentI] );
    }

    // observSeqs[agent][ts] -> stageObserv[ts][agent]
    vector< vector<Index> > stageObserv(t, vector<Index>(nrAg) );
    for (Index agentI = 0; agentI < nrAg; agentI++)
        for (Index ts = 0; ts < t; ts++)
            stageObserv[ts][agentI] = observSeqs[agentI][ts];    
    
    for (Index agentI = 0; agentI < nrAg; agentI++)
        delete [] observSeqs[agentI];
    
    //individual observation sequences -> joint observation sequences.
    vector<Index> jobs(t);
    for (Index ts = 0; ts < t; ts++)
        jobs[ts] = IndividualToJointObservationIndices(stageObserv[ts]);

    //joint observation sequences -> joint history index.
    Index johI = GetJointObservationHistoryIndex(t, jobs);
    return(johI);
    
}
Index 
PlanningUnitMADPDiscrete::IndividualToJointActionHistoryIndex(Index t, 
        const vector<Index>& indivIs) 
const
{
    //individual action history index -> individual action sequences
    size_t nrAg = GetNrAgents();
    vector< Index* > observSeqs(nrAg);
    for (Index agentI = 0; agentI < nrAg; agentI++)
    {
        observSeqs[agentI] = new Index[t];
        Index ahI_thisAgent = indivIs.at(agentI);
        GetActionHistoryArrays (agentI, ahI_thisAgent, t, 
                observSeqs[agentI] );
    }

    // observSeqs[agent][ts] -> stageObserv[ts][agent]
    vector< vector<Index> > stageObserv(t, vector<Index>(nrAg) );
    for (Index agentI = 0; agentI < nrAg; agentI++)
        for (Index ts = 0; ts < t; ts++)
            stageObserv[ts][agentI] = observSeqs[agentI][ts];    
    
    for (Index agentI = 0; agentI < nrAg; agentI++)
        delete [] observSeqs[agentI];
    
    //individual action sequences -> jaint action sequences.
    vector<Index> jabs(t);
    for (Index ts = 0; ts < t; ts++)
        jabs[ts] = IndividualToJointActionIndices(stageObserv[ts]);

    //jaint action sequences -> jaint history index.
    Index jahI = GetJointActionHistoryIndex(t, jabs);
    return(jahI);
    
}

LIndex 
PlanningUnitMADPDiscrete::IndividualToJointActionObservationHistoryIndex(
        Index t, const vector<Index>& indivIs) const
{
    //individual observation history index -> individual observation sequences
    size_t nrAg = GetNrAgents();
    vector< Index* > observSeqs(nrAg);
    vector< Index* > actionSeqs(nrAg);
    for (Index agentI = 0; agentI < nrAg; agentI++)
    {
        observSeqs[agentI] = new Index[t];
        actionSeqs[agentI] = new Index[t];
        Index aohI_thisAgent = indivIs.at(agentI);
        GetActionObservationHistoryArrays (agentI, aohI_thisAgent, t, 
                actionSeqs[agentI], observSeqs[agentI]  );
    }

    // observSeqs[agent][ts] -> stageObserv[ts][agent]
    vector< vector<Index> > stageObserv(t, vector<Index>(nrAg) );
    vector< vector<Index> > stageAction(t, vector<Index>(nrAg) );
    for (Index agentI = 0; agentI < nrAg; agentI++)
        for (Index ts = 0; ts < t; ts++)
        {
            stageObserv[ts][agentI] = observSeqs[agentI][ts];    
            stageAction[ts][agentI] = actionSeqs[agentI][ts];    
        }
    
    for (Index agentI = 0; agentI < nrAg; agentI++)
    {
        delete [] observSeqs[agentI];
        delete [] actionSeqs[agentI];
    }
    
    //individual observation sequences -> joint observation sequences.
    vector<Index> jobs(t);
    vector<Index> jacs(t);
    
    for (Index ts = 0; ts < t; ts++)
    {
        jobs[ts] = IndividualToJointObservationIndices(stageObserv[ts]);
        jacs[ts] = IndividualToJointActionIndices(stageAction[ts]);
        
    }

    //joint observation sequences -> joint history index.
    LIndex jaohI = GetJointActionObservationHistoryIndex(t, jacs, jobs);
    return(jaohI);
    
}

//-----------------------------------------
//END of "IndividualToJoint...HistoryIndex()" functions
//-----------------------------------------













//start action hist functions...

size_t PlanningUnitMADPDiscrete::GetNrActionHistories(Index agentI) const
{
    if(agentI < _m_nrActionHistories.size() )
        return(_m_nrActionHistories.at(agentI));
    //else
    throw(E("ERROR PlanningUnitMADPDiscrete::GetNrActionHistories(Index agentI) index out of bounds or _m_nrActionHistories not yet initialized?!\n"));
} 
size_t PlanningUnitMADPDiscrete::GetNrJointActionHistories() const
{
/* incorrect calculation (takes cartesian product over action histories
 * of different time-steps...)
 * size_t nrJointActionHistories = 1;
    size_t nrAgents = GetNrAgents();
    for(Index agentI=0; agentI < nrAgents; agentI++)
    nrJointActionHistories *= GetNrActionHistories(agentI);

    if( nrJointActionHistories != _m_jointActionHistoryTreeVector.
        size() )
    cerr << "WARNING: PlanningUnitMADPDiscrete::GetNrJointActionHistories() - nrJointActionHistories (= "<< nrJointActionHistories <<" ) != _m_jointActionHistoryTreeVector.size() (= "<< _m_jointActionHistoryTreeVector.size() <<" ) !!"<<endl; */
    size_t nr = _m_jointActionHistoryTreeVector.size();
//    cerr << " _m_jointActionHistoryTreeVector.size() = "<< _m_jointActionHistoryTreeVector.size() << " - nr = " << nr << endl;
    return nr;    
}
ActionHistoryTree* PlanningUnitMADPDiscrete::GetActionHistoryTree(Index agentI, Index ohI) const
{
    size_t nrA = GetNrAgents();
    if(_m_actionHistoryTreeVectors.size() != nrA)
       throw E("PlanningUnitMADPDiscrete::GetActionHistoryTree  _m_actionHistoryTreeVectors.size() != nrA)");

    if(agentI < nrA )
    {
        size_t nrOH = GetNrActionHistories(agentI);
        if(DEBUG_PUD){cout << "_m_actionHistoryTreeVectors["<<agentI<<
            "].size() = " <<_m_actionHistoryTreeVectors[agentI].size()<<endl;}
        if(_m_actionHistoryTreeVectors[agentI].size() != nrOH)
            throw E("_m_actionHistoryTreeVectors[agentI].size() != nrOH)");

        if(ohI < nrOH )
        {
            if(DEBUG_PUD){ cout <<"accessing _m_actionHistoryTreeVectors["<<    
                agentI<<"]["<<ohI<<"]"<<endl;}

            ActionHistoryTree* oht = 
            _m_actionHistoryTreeVectors[agentI][ohI];
            if(DEBUG_PUD){ cout <<"...done - returning "<<oht<<endl;}
            return (oht);
        }
        else
            cerr << "WARNING PlanningUnitMADPDiscrete::GetActionHistory(Index"<<
               " agentI, Index a) index ohI out of bounds"<<endl;
    }
    else
    cerr << "WARNING PlanningUnitMADPDiscrete::GetActionHistory(Index agentI, Index ohI) index agentI out of bounds"<<endl;   
    
    return(0);
   
}
JointActionHistoryTree* PlanningUnitMADPDiscrete::
    GetJointActionHistoryTree(Index johI) const
{
    if(johI < _m_nrJointActionHistories)
        return(_m_jointActionHistoryTreeVector.at(johI));
    else
    {
    cerr << "PlanningUnitMADPDiscrete::GetJointActionHistoryTree - Warning: "<<
        "index (" << johI <<")out of bound!" << endl;
    return(0);
    }
}

Index PlanningUnitMADPDiscrete::
GetJointActionHistoryIndex(JointActionHistoryTree* joh) const
{
    for(Index i=0;i<_m_nrJointActionHistories;i++)
    {
        if(_m_jointActionHistoryTreeVector.at(i)==joh)
            return(i);
    }

    // we should not get here
    throw E("GetJointActionHistoryIndex index not found");
    return(0);
}

    
vector<Index> PlanningUnitMADPDiscrete::JointToIndividualActionIndices(Index joI) const
{
    return(GetMADPDI()->JointToIndividualActionIndices(joI));
}


JointBeliefInterface* 
PlanningUnitMADPDiscrete::GetJointBeliefInterface(LIndex jaohI) const
{
    // NOTE: this function so that it always returns a _new_
    // joint belief. This means we also do not have to worry about invalidating
    // the beliefs cache.
    
    if(_m_params.GetComputeJointBeliefs())
    {
        JointBeliefInterface* jbi = GetNewJointBeliefInterface();
        *jbi = *(_m_jBeliefCache.at(CastLIndexToIndex(jaohI))); //make a copy
        return jbi;
    }
    else
    {
        vector<Index> jaIs;
        vector<Index> joIs;
        GetJointActionObservationHistoryVectors(jaohI,jaIs,joIs);

        JointBeliefInterface* JB = GetNewJointBeliefFromISD();
        for(unsigned i=0;i!=jaIs.size();++i)
            JB->Update(*GetMADPDI(),jaIs[i],joIs[i]);
#if DEBUG_PUDCGETJB
        if(!JB->SanityCheck())
        {
            JB->Print();
            abort();
        }
#endif
        return(JB);
    }
}



double 
PlanningUnitMADPDiscrete::GetJAOHProbs(
    //output args:
    JointBeliefInterface* jb, 
    //input args:
    LIndex jaohI, // the jaohI for which we want to know the probs
    LIndex p_jaohI, //=0 the GIVEN predecessor
    const JointBeliefInterface* p_jb,// = NULL, // the corresponding GIVEN jb of p
    const JointPolicyDiscrete * jpol// = NULL // the policy followed in
    ) const
{
    if(p_jaohI == 0 && p_jb == NULL && jpol==NULL) //the cacheable case
        if(_m_params.GetComputeJointBeliefs() )
        {
            //return the cached results...
            //
            //we have to make a copy (can't do jb = ..., because that would 
            //not be reflected in the calling function)
            *jb = *_m_jBeliefCache[CastLIndexToIndex(jaohI)]; 
            return (_m_jaohProbs[CastLIndexToIndex(jaohI)]);
        }
    //we can't get the cached result, so we have to compute.

    Index t = GetTimeStepForJAOHI(jaohI);
    Index t_p = GetTimeStepForJAOHI(p_jaohI);
    
    Index jaIs[t];
    Index joIs[t];
    GetJointActionObservationHistoryArrays(jaohI, t, jaIs, joIs);      
#if DEBUG_PUD_JAOHPROBS
    vector<Index> jaIsv,joIsv;
    JointActionObservationHistoryTree* jaoht = 
        GetJointActionObservationHistoryTree(jaohI);
    jaoht->GetJointActionObservationHistory()->
        GetJointActionObservationHistoryVectors(jaIsv, joIsv);
    
    cout << "t " << t << " " << jaohI  
         << "\njaIsv:" << SoftPrintVector(jaIsv)
         << ", joIsv:" << SoftPrintVector(joIsv)
         << ", jaoht:" << jaoht->GetJointActionObservationHistory()->SoftPrint()
         << endl;

    for(Index i=0;i!=t;++i)
        cout << "jaIs["<<i<<"]=" << jaIs[i] << " ";
    cout << endl;
#endif
    Index p_jaIs[t_p];
    Index p_joIs[t_p];
    
    if(p_jaohI != 0)
    {
        //check that p_jaohI is indeed a predecessor of jaohI, this consists of
        //two checks 1) the (quick) stage check::
        if (t_p >= t)
            return (0.0);

        if(t_p == 0)
            throw E(" p_jaohI != 0 but t_p == 0 ");

        //and 2) the consistency check
        GetJointActionObservationHistoryArrays(p_jaohI, t_p, p_jaIs, p_joIs); 
        for(Index tI=0; tI < t_p; tI++)
        {
            if ( jaIs[tI] == p_jaIs[tI] && 
                 joIs[tI] == p_joIs[tI]    )  
                ; //consistent
            else
            {
                cerr << "GetJAOHProbs:: Warning pred. inconsistent with requested jaohI" << endl;
                return (0.0); // p_jaohI is inconsistent with jaohI.
            }
        }
    }
    //p_jaIs[ t_p ... (t-1) ] contains the joint actions `still to be taken'
    //to get from p_jaohI to jaohI.

    if(p_jaohI != 0 && p_jb == NULL)
    {
        // a predecessor is specified, but without accompanying joint belief.
        // In this case we assume that a pure (deterministic) joint policy
        // that is consistent with p_jaohI has been followed.
        if(_m_params.GetComputeJointBeliefs() )
        {
            //if joint beliefs are cache use them:
            *jb = *_m_jBeliefCache[CastLIndexToIndex(jaohI)]; 
            double pr = _m_jaohProbs[CastLIndexToIndex(jaohI)];
            double p_pr = _m_jaohProbs[CastLIndexToIndex(p_jaohI)];
            double pr_cond = pr / p_pr;
            return pr_cond;
        }
        else
        {
            // Here we compute the joint belief corresponding to p_jaohI for
            // such a pure joint policy. by performing the belief update
            // starting from the first time step
            jb->Set( *GetMADPDI()->GetISD() );
            //*jb = *GetNewJointBeliefFromISD();
            GetJAOHProbsRecursively(jb, p_jaIs, p_joIs, 0, t_p, 
                    Globals::INITIAL_JAOHI );        
            //GetJAOHProbsRecursively(jb, p_jaIs, p_joIs, t0, t_p, jaoh0 );
        }
    }
    else if ( p_jb == NULL ) // and p_jaohI == 0
        jb->Set( *GetMADPDI()->GetISD() );
        //*jb = *GetNewJointBeliefFromISD();
    else
        *jb = *p_jb; //copy by value, we don't want to alter p_jb!

    double p = GetJAOHProbsRecursively(jb, 
            jaIs, joIs, t_p, t,
            //jaohI, 
            p_jaohI, 
            jpol);

    return(p);
}

double 
PlanningUnitMADPDiscrete::GetJAOHProbsRecursively(
    //input/output arg:
    JointBeliefInterface* jb, 
    //input args:
    const Index jaIs[],
    const Index joIs[],
    Index t_p,
    Index t,
//    const Index jaohI, // the jaohI for which we want to know the probs
    LIndex p_jaohI,// = 0, // the GIVEN predecessor
//    const JointBelief* p_jb = NULL, // the corresponding GIVEN jb of p
    const JointPolicyDiscrete * jpol// = NULL // the policy followed in
    ) const
{
    //if(jaohI == p_jaohI) 
    if(t == t_p)
        return(1.0);

    Index jaI = jaIs[t_p];
    Index joI = joIs[t_p];
    double cond_P = jb->Update(*GetMADPDI(), jaI, joI);
//    Index next_p_jaohI = //???
    double P_remainder = GetJAOHProbsRecursively(
            jb, 
            jaIs,
            joIs,
            t_p + 1,
            t,
//            jaohI,
            GetSuccessorJAOHI(p_jaohI, jaI, joI),
            jpol
    );
    //jb should now be the jb corresponding to jaohI
    double polProb = 0.0;
    if(jpol != NULL)
    {
        LIndex jdI = 0;
        switch ( jpol->GetPolicyDomainCategory() )
        {
            case OHIST_INDEX :
                jdI = GetJointObservationHistoryIndex(t_p, joIs);
                break;
            case AOHIST_INDEX :
                jdI = p_jaohI;
                break;
            default:
                throw E("PlanningUnitMADPDiscrete::GetJAOHProbsRecursively --- policy has unsupported PolicyDomainCategory");
        }
        polProb = jpol->GetJointActionProb( jdI, jaI ) ;
    }
    else
        polProb = 1.0;

    //line 10 of Algorithm 1 of the manual documentation:
    double Prob_jaohI = P_remainder * cond_P * polProb;
    return (Prob_jaohI);
}



double PlanningUnitMADPDiscrete::GetJAOHProb( 
        LIndex jaohI, 
        Index p_jaohI, // the GIVEN predecessor
        const JointBeliefInterface* p_jb, 
        const JointPolicyDiscrete* jpol 
        ) const
{
    JointBeliefInterface* dummy = GetNewJointBeliefInterface();
    double p =  GetJAOHProbs(dummy, jaohI, p_jaohI, p_jb, jpol );
    delete dummy;
    return p;
    
    
}

double PlanningUnitMADPDiscrete::GetJAOHProbGivenPred(LIndex jaohI) const
{
    if(_m_params.GetComputeJointActionObservationHistories())
        return(_m_jaohConditionalProbs[CastLIndexToIndex(jaohI)]);
    //else

    vector<Index> jaIs;
    vector<Index> joIs;
    GetJointActionObservationHistoryVectors(jaohI,jaIs,joIs);
    Index previous = GetJointActionObservationHistoryIndex(jaIs.size()-1,
                                                           jaIs,joIs);

    JointBeliefInterface* dummy = GetNewJointBeliefInterface();
    double prob = GetJAOHProbs(dummy, jaohI, previous);
    delete dummy;
    return prob;
}

//print functions

void PlanningUnitMADPDiscrete::PrintObservationHistories()
{
    size_t nrAgents=GetNrAgents();
    for(Index i = 0; i < nrAgents; i++)
    {
    cout << "ObservationHistoryTree for agent "<<i<<endl;
    _m_observationHistoryTreeRootPointers[i]->Print();
    }
}

void PlanningUnitMADPDiscrete::PrintActionHistories()
{
    size_t nrAgents=GetNrAgents();
    for(Index i = 0; i < nrAgents; i++)
    {
        cout << "ActionHistoryTree for agent "<<i<<endl;
        _m_actionHistoryTreeRootPointers[i]->Print();
    }
}

void PlanningUnitMADPDiscrete::PrintActionObservationHistories()
{
    if(!_m_params.GetComputeIndividualActionObservationHistories())
    {
        cout<<"PlanningUnitMADPDiscrete::PrintActionObservationHistories()"<<
            " called on a P.U. that did not generate action-observation"<<
            " histories."<<endl;
        return;
    }
    size_t nrAgents=GetNrAgents();
    for(Index i = 0; i < nrAgents; i++)
    {
        cout << "ActionObservationHistoryTree for agent "<<i<<endl;
        _m_actionObservationHistoryTreeRootPointers[i]->Print();
    }
}

void PlanningUnitMADPDiscrete::Print()
{
    if(! _m_initialized)
    cerr << "This PlanningUnitMADPDiscrete is not initialized (yet)" <<
        endl;
    else
    {
        cout << "PlanningUnitMADPDiscrete parameters:" << endl;
        _m_params.Print();

        //observ.
        cout << "number of individual observation histories: ";
        PrintVectorCout(_m_nrObservationHistories);
        cout << endl;
        cout << "number of individual observation histories per time step: ";
        PrintVectorCout(_m_nrObservationHistoriesT);
        cout << endl;
        cout << "index of first indiv. observation histories per time step: ";
        PrintVectorCout(_m_firstOHIforT);
        cout << endl;

        if(_m_params.GetComputeIndividualObservationHistories())
            PrintObservationHistories();
        cout << "number of joint observation histories: "<<
            _m_nrJointObservationHistories << endl;
        if(_m_params.GetComputeJointObservationHistories())
            _m_jointObservationHistoryTreeRoot->Print();   
        //actions        
        cout << "number of individual action histories: ";
        PrintVectorCout(_m_nrActionHistories);
        cout << endl;
        
        cout << "number of individual action histories per time step: ";
        PrintVectorCout(_m_nrActionHistoriesT);
        cout << endl;
        
        cout << "index of first indiv. action histories per time step: ";
        PrintVectorCout(_m_firstAHIforT);
        cout << endl;

        if(_m_params.GetComputeIndividualActionHistories())
            PrintActionHistories();
        cout << "number of joint action histories: "<<
            _m_nrJointActionHistories << endl;
        if(_m_params.GetComputeJointActionHistories())
            _m_jointActionHistoryTreeRoot->Print();
        
        //action-observations
        cout << "number of individual action-observation histories: ";
        PrintVectorCout(_m_nrActionObservationHistories);
        cout << endl;
        cout << "number of individual action-observ. histories per time step: ";
        PrintVectorCout(_m_nrActionObservationHistoriesT);
        cout << endl;
        cout << "index of first indiv. action-obs. histories per time step: ";
        PrintVectorCout(_m_firstAOHIforT);
        cout << endl;
        if(_m_params.GetComputeIndividualActionObservationHistories())
            PrintActionObservationHistories();
        cout << "number of joint action observation histories: "<<
            _m_nrJointActionObservationHistories << endl;
        if(_m_params.GetComputeJointActionObservationHistories())
            _m_jointActionObservationHistoryTreeRoot->Print();   

    }
}

Index PlanningUnitMADPDiscrete::GetTimeStepForOHI(Index agI, Index ohI) const
{
    if(_m_params.GetComputeIndividualObservationHistories())
        return(GetObservationHistoryTree(agI, ohI)->GetContainedElement()->
                GetLength() );

    //else... find out what time step this is a joh for
    Index t = 0;
    size_t h=GetHorizon();
    while(  t < h && _m_firstOHIforT.at(agI).at(t) <= ohI )
        t++;

    //  _m_firstJOHIforT[t] > ohI  OR t==GetHorizon
    if(t == h)
        t--; //last time step is h-1
    else
        t--;//_m_nrJointObservationHistoriesT[t] > ohI so ohI belongs to 
            //previous time step.
    return(t);
}
Index PlanningUnitMADPDiscrete::GetTimeStepForJOHI(Index johI) const
{
    if(_m_params.GetComputeJointObservationHistories())
        return(GetJointObservationHistoryTree(johI)->GetContainedElement()->
                GetLength() );

    //else... find out what time step this is a joh for
    Index t = 0;
    size_t h=GetHorizon();
    while(  t < h && _m_firstJOHIforT[t] <= johI )
        t++;

    //  _m_firstJOHIforT[t] > johI  OR t==GetHorizon
    if(t == h)
        t--; //last time step is h-1
    else
        t--;//_m_nrJointObservationHistoriesT[t] > johI so johI belongs to 
            //previous time step.
    return(t);
}

Index PlanningUnitMADPDiscrete::GetSuccessorJOHI(Index johI, Index joI) const
{
    if(_m_params.GetComputeJointObservationHistories())
        return(CastLIndexToIndex(GetJointObservationHistoryTree(johI)->GetSuccessor(joI)->
                                 GetIndex()));

    Index t = GetTimeStepForJOHI(johI);
    if(t >= GetHorizon() - 1)
        throw E("taking successor of last time step joint observation history");
    //Index johI Without Offset:
    Index johI_wo = johI - CastLIndexToIndex(_m_firstJOHIforT[t]);
    //johI_wo   = |jo|^t * joI(0) +...+ |jo|^0 * joI(t)
    //johIsuc_wo= |jo^|^(t+1) *joI(0) +...+ |jo|^1 * joI(t) + |jo|^0 * joI(t+1)
    //therefore the index of the successor (without offset) is:
    Index johIsuc_wo = johI_wo * GetNrJointObservations() + joI;
    return(johIsuc_wo + CastLIndexToIndex(_m_firstJOHIforT[t+1]));
}

Index PlanningUnitMADPDiscrete::GetSuccessorOHI(Index agI, Index ohI, Index oI)
    const
{
    if(_m_params.GetComputeIndividualObservationHistories())
        return(CastLIndexToIndex(GetObservationHistoryTree(agI, ohI)->GetSuccessor(oI)->
                                 GetIndex()));

    Index t = GetTimeStepForOHI(agI, ohI);
    if(t >= GetHorizon() - 1)
        throw E("taking successor of last time step  observation history");
    //Index ohI Without Offset:
    Index ohI_wo = ohI - CastLIndexToIndex(_m_firstOHIforT.at(agI).at(t));
    //ohI_wo   = |o|^t * oI(0) +...+ |o|^0 * oI(t)
    //ohIsuc_wo= |o^|^(t+1) *oI(0) +...+ |o|^1 * oI(t) + |o|^0 * oI(t+1)
    //therefore the index of the successor (without offset) is:
    Index ohIsuc_wo = ohI_wo * GetNrObservations(agI) + oI;
    return(ohIsuc_wo + CastLIndexToIndex(_m_firstOHIforT.at(agI).at(t+1)));
}

Index PlanningUnitMADPDiscrete::GetTimeStepForAHI(Index agI, Index ahI) const
{
    if(_m_params.GetComputeIndividualActionHistories())
        return(GetActionHistoryTree(agI, ahI)->GetContainedElement()->
                GetLength() );

    //else... find out what time step this is a joh for
    Index t = 0;
    size_t h=GetHorizon();
    while(  t < h && _m_firstAHIforT.at(agI).at(t) <= ahI )
        t++;

    //  _m_firstJAHIforT[t] > ahI  OR t==GetHorizon
    if(t == h)
        t--; //last time step is h-1
    else
        t--;//_m_nrJointActionHistoriesT[t] > ahI so ahI belongs to 
            //previous time step.
    return(t);
}
Index PlanningUnitMADPDiscrete::GetTimeStepForJAHI(Index jahI) const
{
    if(_m_params.GetComputeJointActionHistories())
        return(GetJointActionHistoryTree(jahI)->GetContainedElement()->
                GetLength() );

    //else... find out what time step this is a joh for
    Index t = 0;
    size_t h=GetHorizon();
    while(  t < h && _m_firstJAHIforT[t] <= jahI )
        t++;

    //  _m_firstJAHIforT[t] > jahI  OR t==GetHorizon
    if(t == h)
        t--; //last time step is h-1
    else
        t--;//_m_nrJointActionHistoriesT[t] > jahI so jahI belongs to 
            //previous time step.
    return(t);
}
Index PlanningUnitMADPDiscrete::GetSuccessorJAHI(Index jahI, Index joI) const
{
    if(_m_params.GetComputeJointActionHistories())
        return(CastLIndexToIndex(GetJointActionHistoryTree(jahI)->GetSuccessor(joI)->
                                 GetIndex()));

    Index t = GetTimeStepForJAHI(jahI);
    if(t >= GetHorizon() - 1)
        throw E("taking successor of last time step joint action history");
    //Index jahI Without Offset:
    Index jahI_wo = jahI - CastLIndexToIndex(_m_firstJAHIforT[t]);
    //jahI_wo   = |jo|^t * joI(0) +...+ |jo|^0 * joI(t)
    //jahIsuc_wo= |jo^|^(t+1) *joI(0) +...+ |jo|^1 * joI(t) + |jo|^0 * joI(t+1)
    //therefore the index of the successor (without offset) is:
    Index jahIsuc_wo = jahI_wo * GetNrJointActions() + joI;
    return(jahIsuc_wo + CastLIndexToIndex(_m_firstJAHIforT[t+1]));
}

Index PlanningUnitMADPDiscrete::GetSuccessorAHI(Index agI, Index ahI, Index oI)
    const
{
    if(_m_params.GetComputeIndividualActionHistories())
        return(CastLIndexToIndex(GetActionHistoryTree(agI, ahI)->GetSuccessor(oI)->
                                 GetIndex()));

    Index t = GetTimeStepForAHI(agI, ahI);
    if(t >= GetHorizon() - 1)
        throw E("taking successor of last time step  action history");
    //Index ahI Without Offset:
    Index ahI_wo = ahI - CastLIndexToIndex(_m_firstAHIforT.at(agI).at(t));
    //ahI_wo   = |o|^t * oI(0) +...+ |o|^0 * oI(t)
    //ahIsuc_wo= |o^|^(t+1) *oI(0) +...+ |o|^1 * oI(t) + |o|^0 * oI(t+1)
    //therefore the index of the successor (without offset) is:
    Index ahIsuc_wo = ahI_wo * GetNrActions(agI) + oI;
    return(ahIsuc_wo + CastLIndexToIndex(_m_firstAHIforT.at(agI).at(t+1)));
}

Index PlanningUnitMADPDiscrete::GetTimeStepForAOHI(Index agI,Index aohI) const
{
    // copied from GetTimeStepForJOHI
    if(_m_params.GetComputeIndividualActionObservationHistories())
        return(CastLIndexToIndex(GetActionObservationHistoryTree(agI, aohI)->
                                 GetContainedElement()->GetLength())); 
    else
    {
        Index t = 0;
        while( t < GetHorizon() && _m_firstAOHIforT.at(agI).at(t) <= aohI )
            t++;
        //  _m_firstJAOHIforT[t] > johI  OR t==GetHorizon
        if(t == GetHorizon())
            t--; //last time step is h-1
        else
            t--;//_m_nrJointActionObservationHistoriesT[t] > aohI so
                //johI belongs to previous time step.
        return(t);
    }
}
Index PlanningUnitMADPDiscrete::GetTimeStepForJAOHI(LIndex jaohI) const
{
    // copied from GetTimeStepForJOHI
    if(_m_params.GetComputeJointActionObservationHistories())
        return(GetJointActionObservationHistoryTree(jaohI)->
               GetContainedElement()->GetLength() ); 
    else
    {
        Index t = 0;
        if(GetHorizon()==MAXHORIZON)
            throw(E("PlanningUnitMADPDiscrete::GetTimeStepForJAOHI does not work yet for infinite-horizon case"));

        while( t < GetHorizon() && _m_firstJAOHIforT[t] <= jaohI )
            t++;
        //  _m_firstJAOHIforT[t] > johI  OR t==GetHorizon
        if(t == GetHorizon())
            t--; //last time step is h-1
        else
            t--;//_m_nrJointActionObservationHistoriesT[t] > jaohI so
                //johI belongs to previous time step.
        return(t);
    }
}

Index PlanningUnitMADPDiscrete::GetSuccessorAOHI(Index agI,
        Index aohI, Index aI, Index oI) const
{
    // copied from GetSuccessorJOHI, look there for comments
    if(_m_params.GetComputeIndividualActionObservationHistories())
        return(CastLIndexToIndex(GetActionObservationHistoryTree(agI, aohI)->
                                 GetSuccessor(aI,oI)->GetIndex()));
    else
    {
        Index t = GetTimeStepForAOHI(agI, aohI);
        if(t >= GetHorizon() - 1)
            throw E("taking successor of last time step  action observation history");
        Index aohI_wo = aohI - CastLIndexToIndex(_m_firstAOHIforT.at(agI).at(t));
        Index aohIsuc_wo = aohI_wo * GetNrObservations(agI)*
            GetNrActions(agI) + aI * GetNrObservations(agI) +oI;
        return(aohIsuc_wo + CastLIndexToIndex(_m_firstAOHIforT.at(agI).at(t+1)));
    }
}

LIndex PlanningUnitMADPDiscrete::GetSuccessorJAOHI(LIndex jaohI, Index jaI,
                                                   Index joI) const
{
    LIndex jaohIsuc;
    // copied from GetSuccessorJOHI, look there for comments
    if(_m_params.GetComputeJointActionObservationHistories())
        jaohIsuc=GetJointActionObservationHistoryTree(jaohI)->
            GetSuccessor(jaI,joI)->GetIndex();
    else
    {
        Index t = GetTimeStepForJAOHI(jaohI);
        if(t >= GetHorizon() - 1)
            throw E("taking successor of last time step joint action observation history");
        LIndex jaohI_wo = jaohI - _m_firstJAOHIforT[t];
        LIndex jaohIsuc_wo = jaohI_wo * GetNrJointObservations()*
            GetNrJointActions() + jaI * GetNrJointObservations() +joI;
        jaohIsuc=jaohIsuc_wo + _m_firstJAOHIforT[t+1];
    }

    if(jaohIsuc<jaohI)
        throw(EOverflow("PlanningUnitMADPDiscrete::GetSuccessorJAOHI index overflow detected"));

    return(jaohIsuc);
}

void PlanningUnitMADPDiscrete::
RegisterJointActionObservationHistoryTree(JointActionObservationHistoryTree* 
                                          jaoht)
{
    throw(E("PlanningUnitMADPDiscrete::RegisterJointActionObservationHistoryTree TreeNode works with Index, not LIndex, should make two classes"));
    Index i=CastLIndexToIndex(jaoht->GetIndex());
    _m_jointActionObservationHistoryTreeMap[i]=jaoht;
}

JointBeliefInterface* PlanningUnitMADPDiscrete::GetNewJointBeliefFromISD() const
{
    JointBeliefInterface* b0 = GetNewJointBeliefInterface();
    const MultiAgentDecisionProcessDiscreteInterface* madpd = GetMADPDI();
    const StateDistribution* isd = madpd->GetISD();
    b0->Set( *isd );
    return(b0);
}

JointActionObservationHistoryTree* 
PlanningUnitMADPDiscrete::GetJointActionObservationHistoryTree(LIndex jaohI)
    const
{
    if(_m_params.GetComputeJointActionObservationHistories())
        return(_m_jointActionObservationHistoryTreeVector.at(CastLIndexToIndex(jaohI)));
    else
    {
        // cannot use m_jointActionObservationHistoryTreeMap[jaohI],
        // it violates the const, as it creates an empty object if
        // jaohI does not exist yet
        map<LIndex,JointActionObservationHistoryTree*>::const_iterator cit
            = _m_jointActionObservationHistoryTreeMap.find(jaohI);
        if(cit!=_m_jointActionObservationHistoryTreeMap.end())
            return(cit->second);
        else
        {
            return(0);
//            throw(E("GetJointActionObservationHistoryTree:: index not found in map"));
        }
    }
}

bool PlanningUnitMADPDiscrete::AreCachedJointToIndivIndices(
    PolicyGlobals::PolicyDomainCategory pdc) const 
{
    switch ( pdc )
    {
    case PolicyGlobals::OHIST_INDEX :
        return _m_params.GetComputeJointObservationHistories();
        break;
    case PolicyGlobals::AOHIST_INDEX :
        return _m_params.GetComputeJointActionObservationHistories();
        break;
    case PolicyGlobals::TYPE_INDEX :
        throw(E("PlanningUnitMADPDiscrete::AreCachedJointToIndivIndices: types are not defined in a PlanningUnitMADPDiscrete context"));
        return false;
        break;
    default:
        throw(E("Used a JointPolicyDiscrete::PolicyGlobals::PolicyDomainCategory unknown to PlanningUnitMADPDiscrete!"));
        break;
    }				/* -----  end switch  ----- */
}

string PlanningUnitMADPDiscrete::SoftPrintPolicyDomainElement(
    Index agentI, Index dI, PolicyGlobals::PolicyDomainCategory cat ) const
{
    switch ( cat )
    {
    case PolicyGlobals::OHIST_INDEX :
        {
            string s = SoftPrintObservationHistory(agentI, dI);
            return s;


            break;
        }
    case PolicyGlobals::AOHIST_INDEX :
        return GetActionObservationHistoryTree(agentI,dI)->
            GetActionObservationHistory()->SoftPrint();
        break;
    case PolicyGlobals::TYPE_INDEX : 
        throw(E("SoftPrintPolicyDomain: types are not a valid policy domain for a PlanningUnitMADPDiscrete"));
        break;
    default:
        throw(E("SoftPrintPolicyDomain: incorrect JointPolicyDiscrete::PolicyGlobals::PolicyDomainCategory"));
        break;
    }
    return("IMPOSSIBLE!");
}

size_t PlanningUnitMADPDiscrete::GetNrPolicyDomainElements(
    Index agentI, PolicyGlobals::PolicyDomainCategory cat,
    size_t depth ) const
{
    switch ( cat )
    {
    case PolicyGlobals::OHIST_INDEX:
        if(depth==MAXHORIZON || depth==GetHorizon())
            return GetNrObservationHistories(agentI);
        else
            return(CastLIndexToIndex(_m_firstOHIforT.at(agentI).at(depth)));
        break;
    case PolicyGlobals::AOHIST_INDEX:
        if(depth==MAXHORIZON || depth==GetHorizon())
            return GetNrActionObservationHistories(agentI);
        else
            return(CastLIndexToIndex(_m_firstAOHIforT.at(agentI).at(depth)));
        break;
    case PolicyGlobals::TYPE_INDEX : 
        throw(E(" GetNrPolicyDomainElements: types are not a valid policy domain for a PlanningUnitMADPDiscrete"));
        break;
    default:
        throw(E("incorrect JointPolicyDiscrete::PolicyGlobals::PolicyDomainCategory"));
        break;
    }
}

PolicyGlobals::PolicyDomainCategory PlanningUnitMADPDiscrete::GetDefaultIndexDomCat()
    const
{
    return PolicyGlobals::OHIST_INDEX;
}

JointBeliefInterface * PlanningUnitMADPDiscrete::GetNewJointBeliefInterface() const
{
    if( _m_params.GetUseSparseJointBeliefs() )
        return (new JointBeliefSparse(GetNrStates()) );
    else
        return (new JointBelief(GetNrStates()) );
}

JointBeliefInterface * PlanningUnitMADPDiscrete::GetNewJointBeliefInterface(size_t size) 
    const
{
    if( _m_params.GetUseSparseJointBeliefs() )
        return (new JointBeliefSparse(size) );
    else
        return (new JointBelief(size) );
}

string PlanningUnitMADPDiscrete::SoftPrintObservationHistory(Index agentI,
                                                             Index ohIndex) const
{

    string s;
    if(_m_params.GetComputeIndividualObservationHistories())
        s = GetObservationHistoryTree(agentI,ohIndex)->
            GetObservationHistory()->SoftPrint();                
    else
    {
        Index t = GetTimeStepForOHI(agentI, ohIndex);
        Index obs[t];
        GetObservationHistoryArrays(agentI, ohIndex, t, obs);
        stringstream ss;
        ss << "(";
        for(Index t2=0; t2<t; t2++)
        {
            Index oI = obs[t2];
            string oName = GetObservation(agentI, oI)->GetName();
            ss << oName;
            if(t2 + 1 < t)
                ss << ",";
        }
        ss << ")";
        s = ss.str();
    }
    return(s);
}

string PlanningUnitMADPDiscrete::SoftPrintAction(Index agentI, Index actionI) const 
{
    return(GetAction(agentI, actionI)->GetName());
}

void PlanningUnitMADPDiscrete::ExportDotGraph(const std::string &filename,
                                              const PolicyDiscretePure &policy,
                                              Index agentI,
                                              bool labelEdges) const
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "PlanningUnitMADPDiscrete::ExportDotGraph failed to open file " << filename << endl;
        throw E(ss);
    }

    fp << PolicyToDotGraph(policy,agentI,labelEdges);
}

string PlanningUnitMADPDiscrete::PolicyToDotGraph(const PolicyDiscretePure &policy,
                                                  Index agentI,
                                                  bool labelEdges) const
{
    stringstream ss;

    ss << "digraph policyAgent" << agentI << " {" << endl;
    ss << "edge [dir=none];" << endl;

    // first we label each node with its action
    for(Index i=0;
        i!=GetNrObservationHistories(agentI);
        ++i)
        ss << "node" << i << " [ label=\"" << SoftPrintAction(agentI,
                                                              policy.GetActionIndex(i))
           << "\" ];" << endl;

    // then we draw all the edges
    for(Index i=0;
        i!=GetNrObservationHistories(agentI);
        ++i)
        {
            Index t = GetTimeStepForOHI(agentI, i);
            if(t >= GetHorizon() - 1)
                continue;

            for(Index o=0;o!=GetNrObservations(agentI);++o)
            {
                ss << "node" << i << " -> "
                   << "node" << GetSuccessorOHI(agentI, i, o);
                if(labelEdges)
                    ss << " [label=\"" << GetObservation(agentI, o)->GetName() << "\"]";
                ss << ";" << endl;
            }
        }
    ss << "}" << endl;

    return(ss.str());
}
