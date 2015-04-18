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

#include <set>
#include "BGCG_SolverNonserialDynamicProgramming.h"
#include "BayesianGameCollaborativeGraphical.h"
#include "LocalBGValueFunctionVector.h"
#include "LocalBGValueFunctionBGCGWrapper.h"
#include <float.h>

#define DEBUG_BGCG_SolverNonserialDynamicProgramming 0

using namespace std;

//Default constructor
BGCG_SolverNonserialDynamicProgramming::BGCG_SolverNonserialDynamicProgramming(
    const BGCG_constPtr &bgcg) :
        BGCG_Solver(bgcg)
{
}

double
BGCG_SolverNonserialDynamicProgramming::Solve()
{
    BGCG_constPtr cgbg = GetBGCG();
    size_t nrAgents = cgbg->GetNrAgents();

    //find an ordering of agents that minimizes the amount of work
    //(typical heuristic is start with agent with lowest degree)
    const vector<Index> agentOrdering = ComputeHeuristicAgentOrder();

    CreateInitialLocalBGFunctionsFromBGCG();

    // initializes _m_agentFunctions and _m_neighbors
    InitializeNeighborsAndAgentFuncs();

    //for each agent we maintain the best response policies:
    //bestResponses[agentI][jpol_neighbors] contains the index of
    //the policy of agentI, that is a best response against his neighbors
    //using (joint) policy jpol_neighbors.
    _m_bestResponses.clear();
    for(Index agI=0; agI < nrAgents; agI++)
        _m_bestResponses.push_back( vector<Index>() );
    
    //we also need to store the scopes of the best-response functions
    //(i.e., for each agent i, the neigborhood its neighborhood at the
    //time when it was eliminated)
    _m_bestResponseScopes.resize(nrAgents);
    
    //for each agent i (according to heuristic ordering)
    //  find G = all agents j, connected to i
    //  compute agent i's best response table and values:
    //  for each (joint) policy pol_G
    //      compute agent i's best response
    //      V(pol_G) = best-response value.
    //
    //  add an LRF representing V(pol_G) to ??? (the cgbg?)

#if DEBUG_BGCG_SolverNonserialDynamicProgramming
    {
    cout << "BGCG_SolverNonserialDynamicProgramming::Solve()  #LRFs=" << cgbg->GetNrLRFs() <<
        ", #local_Fs=" << _m_local_Fs.size()
        <<" summary: "<<endl;
    set<LocalBGValueFunctionInterface*>::iterator Func_it;
    for(Func_it=_m_local_Fs.begin();Func_it!=_m_local_Fs.end();Func_it++)
        cout << (*Func_it)->SoftPrint()<<endl;
        
    cout << "BGCG_SolverNonserialDynamicProgramming::Solve() using agent ordering " << 
        SoftPrintVector(agentOrdering) << endl;
    }
#endif

    // eliminate the agents one by one
    vector<Index>::const_iterator agent_it = agentOrdering.begin();
    while(agent_it != agentOrdering.end())
    {
        EliminateAgent(*agent_it);
        agent_it++;
    }

#if 1 && DEBUG_BGCG_SolverNonserialDynamicProgramming
    cout << "\n\n>>>Elimination phase finished<<<\n"<<endl;
    cout << "number of local functions left:"<<_m_local_Fs.size()<<
        "- a summary:"<<endl;
    set<LocalBGValueFunctionInterface*>::iterator lf_it;
    for(lf_it = _m_local_Fs.begin(); lf_it != _m_local_Fs.end();lf_it++)
        cout << (*lf_it)->SoftPrint() << endl;
#endif
    if(_m_local_Fs.size() != 1)
    {
        throw E("after elimination of all agents (_m_local_Fs.size() != 1) - unexpected");
        //I'm not really sure if this is possible?
    }

    // get the value we computed
    double value_this_bgcg = (*(_m_local_Fs.begin()))->GetValue(0);

    // construct the best responses for each agent, and combine it to
    // the best joint policy
    LIndex bestJointBGPolicyIndex = 
        GetJpolIndexForBestResponses(agentOrdering);

    //this is a BGCG_Solver which is a BGIPSolution, so let's set the solution
    AddSolution(bestJointBGPolicyIndex,value_this_bgcg);

#if DEBUG_BGCG_SolverNonserialDynamicProgramming
    cout << "BGCG_SolverNonserialDynamicProgramming::Solve returning value " << value_this_bgcg
         << " (jpol index " << bestJointBGPolicyIndex << ")" << endl;
#endif

    return(value_this_bgcg);
}

void BGCG_SolverNonserialDynamicProgramming::EliminateAgent(Index agToElim)
{
    size_t nrAgents=GetBGCG()->GetNrAgents();
    const set<Index> thisAgNeighbors = _m_neighbors[agToElim];
    _m_bestResponseScopes[agToElim] = vector<Index>(thisAgNeighbors.begin(), 
                                                    thisAgNeighbors.end());

    size_t nrNeighbors =  thisAgNeighbors.size();
#if DEBUG_BGCG_SolverNonserialDynamicProgramming
    cout  << "BGCG_SolverNonserialDynamicProgramming::Solve() starting on elimination of agent "<<agToElim
          <<" , with "<<nrNeighbors<< " neigbors, namely:"
          << SoftPrint(thisAgNeighbors) << endl;
#endif                
                
    //vector<Index> thisAgLRFs = AgentLRFs[agToElim];
    set<LocalBGValueFunctionInterface*> thisAgentFuncs = 
        _m_agentFunctions[agToElim];
    //the number of policies per neighbor.
    vector<size_t> nrNeighborsPolicies;
    // nr joint neigbor policies (excluding this agent)
    size_t nrJointNeighborPolicies = 1;
//I don't want to treat 0 neighbors as a special case (if not necessary)
    set<Index>::const_iterator tan_it;
//         if(nrNeighbors > 0)
//         {
    for(tan_it = thisAgNeighbors.begin(); 
        tan_it != thisAgNeighbors.end(); tan_it++)
    {
        size_t nrpols = CastLIndexToIndex(GetBGCG()->GetNrPolicies(*tan_it));
        nrNeighborsPolicies.push_back(nrpols);
        nrJointNeighborPolicies *= nrpols;
    }
//         }
//         else
//             nrJointNeighborPolicies = 0;


    //we now create the new local function that replace the functions
    //in which agent agToElim participates.
    //
    //first we convert thisAgNeighbors to a Scope
    Scope neighScope(thisAgNeighbors.begin(), thisAgNeighbors.end() );

    LocalBGValueFunctionInterface* newFunc =
        new LocalBGValueFunctionVector(GetBGCG(), neighScope);

#if DEBUG_BGCG_SolverNonserialDynamicProgramming
    {
        cout << "BGCG_SolverNonserialDynamicProgramming::Solve() eliminating agent " << agToElim 
             << " (nrJNPol " << nrJointNeighborPolicies 
             << ") (neighScope " << SoftPrint(neighScope) << ") ";
        Index it=0;
        for(tan_it = thisAgNeighbors.begin(); 
            tan_it != thisAgNeighbors.end(); tan_it++)
        {
            cout << "(agent " << *tan_it << " #pol "
                 << nrNeighborsPolicies.at(it++) << ") ";
        }
        cout << endl;
        cout << "This agent has "<< thisAgentFuncs.size() << " local"<<
            " value functions it participates in."<<endl;
            /*"- summary:"<<endl;
              set<LocalBGValueFunctionInterface*>::iterator thisAgentFunc_it;
              for(thisAgentFunc_it = thisAgentFuncs.begin();
              thisAgentFunc_it != thisAgentFuncs.end();
              thisAgentFunc_it++)
              cout << (*thisAgentFunc_it)->SoftPrint()<<endl;*/
    }
#endif

    //now we rotate through all possible joint policies for the neighbors
    for(Index jpolI=0; jpolI < nrJointNeighborPolicies; jpolI++)
    {
#if 0
        //print status
        size_t fleft = _m_local_Fs.size();
        stringstream ss;
        ss << "Current agent="<<agToElim
           << ",#local functions left="<<fleft<<
            " - evaluating joint neighbor policies for current agent";
        PrintProgress(ss.str(), jpolI, nrJointNeighborPolicies,25);
#endif
        vector<Index> indPols = 
            IndexTools::JointToIndividualIndices(jpolI,nrNeighborsPolicies);

        //NOTE: we are now rotating through agToElim's policies, however
        //this agent might be able to compute a best response for each of
        //its individual histories...

        double best_response_value = -DBL_MAX;
        Index best_response = 0;

        //rotate through agToElim's policies (in order to compute a best
        //response against jpolI)
        for(Index polA=0; polA < GetBGCG()->GetNrPolicies(agToElim); polA++)
        {
            //evaluate the value of polA:
            //for each of agToElim's functions (i.e., LRFs) we evaluate
            //the expected payoff
            double value = 0.0;

            set<LocalBGValueFunctionInterface*>::const_iterator
                thisAgentFunc_it;
            for(thisAgentFunc_it = thisAgentFuncs.begin();
                thisAgentFunc_it != thisAgentFuncs.end();
                thisAgentFunc_it++)
            {
                LocalBGValueFunctionInterface* f = *thisAgentFunc_it;

                Scope agSC_f = f->GetAgentScope();
#if 0 && DEBUG_BGCG_SolverNonserialDynamicProgramming
                cout << "BGCG_SolverNonserialDynamicProgramming::Solve() jpolI " << jpolI << " polA " << polA
                     << " agSC_f " << SoftPrint(agSC_f) 
                     << " f.SoftPrint= "
                     << f->SoftPrint() << endl;
#endif

                //construct a vector with the policy for each agent that
                //plays a role in this local function.
                //
                //the 'problem' is that we have to insert agToElim's policy
                //(polA)  at the right position.

                vector<Index> indPols_f;
                for(Index scI=0; scI < agSC_f.size(); scI++)
                {
                    Index agToAdd = agSC_f[scI];
                    //cout << "looking for policy for agent "<<agToAdd;
                    if(agToAdd == agToElim)
                        indPols_f.push_back(polA);
                    else
                    {
                        //we need to find the index of this agent within
                        //the neighbor hood.
                        //
                        //i.e., we have indPols, e.g. <1 3 1>
                        //which is a tuple of nrNeighbors policies. 
                        //
                        //now we search for the policy of agent i (say 5)
                        //if we know thisAgNeighbors = < 1 5 7>
                        //then we know that index_within_neighborhood=1
                        //and that its policy is 3
                        set<Index>::iterator neighbors_it = 
                            thisAgNeighbors.begin();
                        for(Index iwn=0; iwn < nrNeighbors; iwn++)
                        {
                            if(*neighbors_it == agToAdd)
                            {
                                indPols_f.push_back(indPols[iwn]);
                                break;
                            }
                            neighbors_it++;
                        }
                    }
                }
                if(indPols_f.size() != agSC_f.size() )
                    throw E("error, wasn't able to construct local policy for LRF");

                //indPols_f now contains the indices for the individual 
                //policies of the agents that participate in function f,
                //so we can get the expected value:
                value += f->GetValue(indPols_f);
            }
            if(value > best_response_value)
            {
                best_response_value = value;
                best_response = polA;
            }
        }
        newFunc->SetValue(jpolI, best_response_value);
            
        //_m_bestResponses[agToElim][jpolI] = polA; we do this in order so:
        _m_bestResponses[agToElim].push_back(best_response);
    }//end for policies of neighbors

#if DEBUG_BGCG_SolverNonserialDynamicProgramming
    cout << "BGCG_SolverNonserialDynamicProgramming::Solve() finished agent " << agToElim
//             << ", before maintenance " << newFunc->SoftPrint() 
         << endl;
#endif        
    //here we have to do maintanance on the _m_local_Fs.
    
    //we have to remove the references to all f's in thisAgentFuncs
    set<LocalBGValueFunctionInterface*>::iterator taf_it;
    for(taf_it = thisAgentFuncs.begin(); taf_it != thisAgentFuncs.end();
        taf_it++)
    {
        LocalBGValueFunctionInterface* function_to_remove = *taf_it;
        _m_local_Fs.erase( function_to_remove );
        //remove references from 
        //vector< set<LocalBGValueFunctionInterface*> > _m_agentFunctions;
        for(Index agI=0; agI < nrAgents; agI++)
            _m_agentFunctions.at(agI).erase( function_to_remove );
    }
    //now add newFunc to _m_local_Fs
    _m_local_Fs.insert(newFunc);
    // and to the functions of the neighbors
//        set<Index>::iterator tan_it; //already declared above
    for(tan_it = thisAgNeighbors.begin(); 
        tan_it != thisAgNeighbors.end(); tan_it++)
        _m_agentFunctions.at(*tan_it).insert(newFunc);

    //remove agToElim from all neigborhoods 
    //(from vector< set<Index> > _m_neighbors)
    for(Index agI=0; agI < nrAgents; agI++)
    {
#if 0 && DEBUG_BGCG_SolverNonserialDynamicProgramming
        cout << "removing agent "<<agToElim<<" from neighbors of agent "
             << agI<<", which currently are:"
             << SoftPrint(_m_neighbors[agI]);
#endif
        _m_neighbors[agI].erase(agToElim);
#if 0 && DEBUG_BGCG_SolverNonserialDynamicProgramming
        cout << " (after `erase':"<< SoftPrint(_m_neighbors[agI])
             << ")"<<endl;
#endif
    }
#if DEBUG_BGCG_SolverNonserialDynamicProgramming
    cout << "BGCG_SolverNonserialDynamicProgramming::Solve() finished eliminating agent " << agToElim << endl;
#endif
}

LIndex 
BGCG_SolverNonserialDynamicProgramming::GetJpolIndexForBestResponses(const vector<Index> &agentOrdering) const
{
    size_t nrAgents = GetBGCG()->GetNrAgents();

    //Get the nr of policies for all agents
    vector<size_t> nrPols;
    for(Index agI=0; agI < nrAgents; agI++)
        nrPols.push_back(CastLIndexToIndex(GetBGCG()->GetNrPolicies(agI)));

//we store the index, not the policy itelf
//JointPolicyPureVector(*cgbg, TYPE_INDEX ) bestJointBGPolicy;
    vector<Index > selectedBRIs(nrAgents);

    Index neighJointPol = 0;
    vector<Index>::const_reverse_iterator rit = agentOrdering.rbegin();
    while(rit != agentOrdering.rend())
    {
        Index agI=*rit;
        const Scope& brSc = _m_bestResponseScopes[agI];
#if DEBUG_BGCG_SolverNonserialDynamicProgramming
        cout << "selecting best response policy for agent "<< agI <<
            ". bestResponseScope(its neighbors) is " <<
            SoftPrint(brSc) << endl;
#endif        
        if(brSc.size() > 0)
        {
            //get the policies selected by the agents in brSc and convert
            //it to neighJointPol index
            vector<Index> restrictedBRIs(brSc.size());
            IndexTools::RestrictIndividualIndicesToScope(selectedBRIs, brSc,
                                                         restrictedBRIs);
            vector<size_t> nrNeighborsPolicies(brSc.size());
            IndexTools::RestrictIndividualIndicesToScope(nrPols, brSc,
                nrNeighborsPolicies);
/*            for(Scope::const_iterator brSc_it = brSc.begin(); 
                    brSc_it != brSc.end(); brSc_it++)
            {
                size_t nrpols = cgbg->GetNrPolicies(*brSc_it);
                nrNeighborsPolicies.push_back(nrpols);
//                nrJointNeighborPolicies *= nrpols;
            }
*/            
            Index brSc_jointIndex = CastLIndexToIndex(IndexTools::IndividualToJointIndices
                                                      (restrictedBRIs, nrNeighborsPolicies));
            neighJointPol = brSc_jointIndex;
        }
        else
            neighJointPol = 0;

        selectedBRIs[agI] = _m_bestResponses[agI][neighJointPol];

#if 0 && DEBUG_BGCG_SolverNonserialDynamicProgramming
        PolicyPureVector brPol(*GetBGCG(), agI, TYPE_INDEX);
        brPol.SetIndex(selectedBRIs[agI]);
        cout << "selecting best response for joint neighbor index "<< 
            neighJointPol <<endl;
        cout << "selected policy index selectedBRIs[agI]="<< selectedBRIs[agI]
             <<", corresponding to BG policy:"<<endl
             <<brPol.SoftPrint();
#endif        
        rit++;
    }
    
    //Get the joint policy index
    LIndex bestJointBGPolicyIndex = IndexTools::IndividualToJointIndices(
            selectedBRIs, nrPols);

    return(bestJointBGPolicyIndex);
}

vector<Index> BGCG_SolverNonserialDynamicProgramming::ComputeHeuristicAgentOrder() const
{
    vector<Index> order;
    for(Index aI=0; aI < GetBGCG()->GetNrAgents(); aI++)
        order.push_back(aI);

    return(order);
}

void BGCG_SolverNonserialDynamicProgramming::CreateInitialLocalBGFunctionsFromBGCG()
{
    _m_local_Fs.clear();

    //create the vector _m_local_Fs from the CGBG:
    //
    //we create 1 LocalBGValueFunctionBGCGWrapper for each LRF.
    for(Index e=0; e < GetBGCG()->GetNrLRFs(); e++)
        _m_local_Fs.insert(new LocalBGValueFunctionBGCGWrapper(GetBGCG(), e));
}

void BGCG_SolverNonserialDynamicProgramming::InitializeNeighborsAndAgentFuncs()
{
    _m_neighbors.clear();
    _m_agentFunctions.clear();

    size_t nrA = GetBGCG()->GetNrAgents();
    _m_neighbors = vector< set<Index> >( nrA );
    _m_agentFunctions = vector< set<LocalBGValueFunctionInterface* > >( nrA );

    set<LocalBGValueFunctionInterface*>::const_iterator lF_it=
        _m_local_Fs.begin();
    while(lF_it != _m_local_Fs.end())
    {
#if DEBUG_BGCG_SolverNonserialDynamicProgramming
        cout << "BGCG_SolverNonserialDynamicProgramming::InitializeNAAF() considering local_Fs "
             << (*lF_it)->SoftPrint() << endl;
#endif
         LocalBGValueFunctionInterface* f = *lF_it;
         Scope agSc = f->GetAgentScope();
         //for all agents in this scope, add f to _m_agentFunctions
         Scope::const_iterator agInSc = agSc.begin();
         while(agInSc != agSc.end() )
         {
             _m_agentFunctions.at(*agInSc).insert( f );
#if DEBUG_BGCG_SolverNonserialDynamicProgramming
             cout << "BGCG_SolverNonserialDynamicProgramming::InitializeNAAF() added to agent " << *agInSc
                  << " " << f->SoftPrint() << endl;
#endif
             agInSc++;
         }
         lF_it++;
    }
    //now we can use _m_agentFunctions to easily find the neighbors:
    for(Index agI=0; agI < nrA; agI++)
    {
        set<LocalBGValueFunctionInterface*>::const_iterator funcsOfAgI =
            _m_agentFunctions.at(agI).begin();
        //we don't  want to insert duplicate in the neigbor list, so we
        //put all neigbors in a set
        set<Index>& neighSet_this_a = _m_neighbors.at(agI);
        while(funcsOfAgI != _m_agentFunctions.at(agI).end())
        {
            Scope agSc = (*funcsOfAgI)->GetAgentScope();
            Scope::iterator agInSc = agSc.begin();
            while(agInSc != agSc.end() )
            {
                if(*agInSc != agI) //don't insert agI itself
                    neighSet_this_a.insert(*agInSc);
                agInSc++;
            }
            funcsOfAgI++;
        }
        
    }
}
