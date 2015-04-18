/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

#include "BayesianGameCollaborativeGraphical.h"
#include "BayesianGameIdenticalPayoff.h"
#include "JointPolicyDiscretePure.h"
#include "PDDiscreteInterface.h"
using namespace std;

BayesianGameCollaborativeGraphical::BayesianGameCollaborativeGraphical() :
    BayesianGameIdenticalPayoffInterface(),
    _m_nrLRFs(0)
    ,_m_jt_pd(0)
{
}

BayesianGameCollaborativeGraphical::BayesianGameCollaborativeGraphical(
        const size_t nrAgents, 
        const vector<size_t>& nrActions,  
        const vector<size_t>& nrTypes)
    :
        BayesianGameIdenticalPayoffInterface(nrAgents, nrActions, nrTypes)
    ,_m_nrLRFs(0)
    ,_m_jt_pd(0)
{
}

//Copy constructor.    
BayesianGameCollaborativeGraphical::BayesianGameCollaborativeGraphical(
    const BayesianGameCollaborativeGraphical& o) :
        BayesianGameIdenticalPayoffInterface(o)
{
    _m_nrLRFs = o._m_nrLRFs;
    if( o._m_jt_pd )
        _m_jt_pd = o._m_jt_pd->Clone();
    _m_LRFs.clear();
    for(Index i=0;i!=o._m_LRFs.size();++i)
        _m_LRFs.push_back(new BayesianGameIdenticalPayoff(*o._m_LRFs.at(i)));
    _m_agentScopes=o._m_agentScopes;
}

//Destructor
BayesianGameCollaborativeGraphical::~BayesianGameCollaborativeGraphical()
{
    for(Index i=0;i!=_m_LRFs.size();++i)
        delete _m_LRFs.at(i);

    if(_m_jt_pd)
        delete _m_jt_pd;
}

//Copy assignment operator
BayesianGameCollaborativeGraphical& BayesianGameCollaborativeGraphical::operator= (const BayesianGameCollaborativeGraphical& o)
{
    throw E(" BayesianGameCollaborativeGraphical::operator= - nyi ");
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
        

void 
BayesianGameCollaborativeGraphical::AddLRF(const Scope &s)
{
    // check whether the scope is sorted, i.e., agent indices are in
    // ascending order
    Scope sSorted=s;
    sSorted.Sort();
    if(!sSorted.Equals(s))
        throw(E("BayesianGameCollaborativeGraphical::AddLRF() Scope should always be sorted"));

    if(! s.empty() )
    {
        Index highestAgentI = s.at(s.size()-1);
        if (highestAgentI >= GetNrAgents() )
            throw(E("BayesianGameCollaborativeGraphical::AddLRF() Scope contains non-existing agent!"));
    }

    _m_nrLRFs++;
    size_t nrAgentsThisLRF = s.size();
    vector<size_t> nrActions;
    vector<size_t> nrTypes;
    for(Index s_I = 0; s_I < nrAgentsThisLRF; s_I++)
    {
        Index agentI=s[s_I];
        nrActions.push_back( _m_nrActions[agentI] );
        nrTypes.push_back( _m_nrTypes[agentI] );
    }
    bool useSparseRewardModel=true;
    _m_LRFs.push_back( new BayesianGameIdenticalPayoff(nrAgentsThisLRF,
                                                       nrActions, nrTypes,
                                                       useSparseRewardModel ) );
    _m_agentScopes.push_back(s);
}
    
size_t 
BayesianGameCollaborativeGraphical::GetNrJointTypesForLRF(Index e)
{
    return _m_LRFs[e]->GetNrJointTypes();
}

size_t 
BayesianGameCollaborativeGraphical::GetNrJointActionsForLRF(Index e)
{
    return _m_LRFs[e]->GetNrJointActions();
}
        
void 
BayesianGameCollaborativeGraphical::DistributeProbability(Index jtI, double p)
{
    //set the probability of the joint type (represented by the 
    //BayesianGameBase )
    BayesianGameBase::SetProbability(jtI, p);

    //for each LRF add p to the consistent vars.
    const vector<Index> &indTypes = JointToIndividualTypeIndices(jtI);
    
    for(Index e = 0; e < _m_nrLRFs; e++)
    {    
        vector<Index> restrictedIndivTypes = RestrictIndividualIndicesToScope(
            indTypes, e);
        //now we have a smaller vector with the individual types restricted
        //to the agent in the scope of e
        _m_LRFs[e]->AddProbability(restrictedIndivTypes, p);
    }

}

void BayesianGameCollaborativeGraphical::
SetProbabilityDistribution(const PDDiscreteInterface* pd)
{
    //first create a clone of the pd
    _m_jt_pd = pd->Clone(); 

    _m_jt_pd->SanityCheck();
    //next make sure that all the edges use consistent probs

    for(Index e = 0; e < _m_nrLRFs; e++)
    {   
        BayesianGameIdenticalPayoff& edge(*(_m_LRFs.at(e)) );
        const Scope& sc_e = GetScope(e);

        for(Index jtI_e=0; jtI_e < edge.GetNrJointTypes(); jtI_e++)
        {
            const vector<Index>& indTypes_e = edge.JointToIndividualTypeIndices(jtI_e);
            double p = _m_jt_pd->Get( sc_e, indTypes_e);
            edge.SetProbability(jtI_e, p);
        }
        edge.SanityCheck();
    }
    
}



vector<double> BayesianGameCollaborativeGraphical::
GetRestrictedJointTypeProbabilities(Index jtI) const
{ 
    vector<double> restricted_probs;
    const vector<Index> &indTypes = JointToIndividualTypeIndices(jtI);
    for(Index e = 0; e < _m_nrLRFs; e++)
    {    
        vector<Index> restrictedIndivTypes = RestrictIndividualIndicesToScope(
            indTypes, e);
        //now we have a smaller vector with the individual types restricted
        //to the agent in the scope of e
        restricted_probs.push_back(
            _m_LRFs[e]->GetProbability(restrictedIndivTypes)
            );
    }
    return restricted_probs;
}

Index  BayesianGameCollaborativeGraphical::
JointToGroupTypeIndex(Index e, Index jtI) const
{
    const vector<Index> &indTypes = JointToIndividualTypeIndices(jtI);
    vector<Index> restrictedIndivTypes = RestrictIndividualIndicesToScope(
            indTypes, e);    
    Index jtGI = _m_LRFs[e]->
        IndividualToJointTypeIndices(restrictedIndivTypes);
    return(jtGI);
}

Index BayesianGameCollaborativeGraphical::
JointToGroupActionIndex(Index e, Index jaI) const
{
    const vector<Index> &indActions = JointToIndividualActionIndices(jaI);
    vector<Index> restrictedIndivActions = RestrictIndividualIndicesToScope(
            indActions, e);    
    Index jaGI = _m_LRFs[e]->
        IndividualToJointActionIndices(restrictedIndivActions);
    return(jaGI);
}

//GetProbability functions
double 
BayesianGameCollaborativeGraphical::
GetProbability(
        Index e, 
        const std::vector< Index >& indTypes) const
{ return _m_LRFs.at(e)->GetProbability(indTypes); }

double 
BayesianGameCollaborativeGraphical::
GetProbability(
        Index e, 
        Index jtI_e ) const
{ return _m_LRFs.at(e)->GetProbability(jtI_e); }

double 
BayesianGameCollaborativeGraphical::
GetProbability(Index jtype) const
{
    if(!_m_jt_pd)
    {
        //throw E("BayesianGameCollaborativeGraphical::GetProbability(Index jtype) - no pd!");
        //for now, fall back on BGBase...
        return BayesianGameBase::GetProbability(jtype);
    }
   
    double p =  _m_jt_pd->Get(jtype);
    return p;
}

//SetProbability functions
void 
BayesianGameCollaborativeGraphical::
SetProbability(
        Index e, 
        const std::vector< Index >& indTypes, 
        double p)
{ _m_LRFs.at(e)->SetProbability(indTypes, p); }

void 
BayesianGameCollaborativeGraphical::
SetProbability(
        Index e, 
        Index jtI_e, 
        double p)
{ _m_LRFs.at(e)->SetProbability(jtI_e, p); }

//GetUtility functions
double 
BayesianGameCollaborativeGraphical::
GetUtility(Index e, Index jtI_e, Index jaI_e) const
{ return _m_LRFs.at(e)->GetUtility(jtI_e, jaI_e); }

double 
BayesianGameCollaborativeGraphical::
GetUtility(Index e, 
        const std::vector< Index >& indTypes_e, 
        const std::vector< Index >& actions_e) const
{ return _m_LRFs.at(e)->GetUtility(indTypes_e, actions_e); }

double 
BayesianGameCollaborativeGraphical::
GetUtility(Index jtype, Index ja) const
{
    double r = 0.0;
    for(Index e=0; e < _m_nrLRFs; e++)
    {
        Index jtI_e = JointToGroupTypeIndex(e, jtype);
        Index jaI_e = JointToGroupActionIndex(e, ja);
        r += GetUtility(e, jtI_e, jaI_e);
    }
    return(r);
}
double BayesianGameCollaborativeGraphical::GetUtility(
        const std::vector<Index>& indTypeIndices, 
        const std::vector<Index>& indActionIndices ) const
{
    throw E("BayesianGameCollaborativeGraphical::GetUtility - nyi ");
}


//SetUtility functions
void 
BayesianGameCollaborativeGraphical::
SetUtility(Index e, Index jtI_e, Index jaI_e, double ut)
{ _m_LRFs.at(e)->SetUtility(jtI_e, jaI_e, ut); }

void 
BayesianGameCollaborativeGraphical::
SetUtility(Index e, 
        const std::vector< Index >& indTypes_e, 
        const std::vector< Index >& actions_e,
        double ut)
{ _m_LRFs.at(e)->SetUtility(indTypes_e, actions_e, ut); }




string BayesianGameCollaborativeGraphical::SoftPrint() const
{
    stringstream ss;
    //ss << BayesianGameBase::SoftPrint();    
    for(Index e=0; e < _m_nrLRFs; e++)
    {
        ss << "LRF function " << e << ":" << endl;
        ss << "Agent scope " << SoftPrintVector(_m_agentScopes.at(e)) << endl;
        ss << _m_LRFs.at(e)->SoftPrint();
        ss << endl;
    }
    return(ss.str());
}

void BayesianGameCollaborativeGraphical::SanityCheckBGCG() const
{
    for(Index e=0; e < _m_nrLRFs; e++)
        _m_LRFs.at(e)->SanityCheck();

    if(!IsFullyConnected())
        cout << "Warning: BayesianGameCollaborativeGraphical is not fully connected"  << endl;
}


bool BayesianGameCollaborativeGraphical::IsFullyConnected() const
{
    vector<bool> agentVisited(GetNrAgents(),false);
    RecurseOverAgents(0, agentVisited);
    bool fullyConnected=true;
    // if the graph is fully connected, the search should have visited
    // every agent
    for(Index i=0;i!=agentVisited.size();++i)
        if(!agentVisited.at(i))
            fullyConnected=false;

    return(fullyConnected);
}

void BayesianGameCollaborativeGraphical::RecurseOverAgents(Index currentAgent,
                                                 vector<bool> &agentVisited) const
{
    agentVisited.at(currentAgent)=true;
    Scope neighbors;
    for(Index e=0;e!=GetNrLRFs();++e)
    {
        Scope lrfScope=GetScope(e);
        if(lrfScope.Contains(currentAgent))
        {
            for(Index i=0;i!=lrfScope.size();++i)
            {
                Index ag=lrfScope.at(i);
                if(ag!=currentAgent &&
                   !neighbors.Contains(ag))
                    neighbors.Insert(ag);
            }
        }
    }
    for(Index i=0;i!=neighbors.size();++i)
        if(!agentVisited.at(neighbors.at(i)))
            RecurseOverAgents(neighbors.at(i),agentVisited);
}


double BayesianGameCollaborativeGraphical::
ComputeValueJPol(
    const JointPolicyDiscretePure & jpolBG
) const
{
    size_t nrR = GetNrLRFs();
    double r = 0.0; //immediate reward (exact)
    for(Index e=0; e < nrR; e++)
    {
        double r_e = 0.0;
        const Scope & agSc = GetScope(e);
        size_t nrAgents_e = agSc.size();
        vector<Index> typeIs_e(nrAgents_e, 0);
        vector<size_t> nrTypes_e(agSc.size());
        IndexTools::RestrictIndividualIndicesToScope(
            GetNrTypes(), agSc, nrTypes_e);

        do{
            //compute the action specified by jpolBG
            vector<Index> aIs_e(nrAgents_e, 0);
            for(Index i=0; i < nrAgents_e; i++)
            {
                Index agI = agSc.at(i);
                Index actionI = jpolBG.GetActionIndex(agI, typeIs_e.at(i));
                aIs_e[i] = actionI;
            }
            double p = GetProbability(e, typeIs_e);
            double this_r =GetUtility(e, typeIs_e, aIs_e);
            r_e += this_r * p;

        }while(!IndexTools::Increment(typeIs_e, nrTypes_e ) );
        r += r_e;

    }
    return r;    
}


