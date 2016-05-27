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

#include "LocalBGValueFunctionBGCGWrapper.h"
#include "BayesianGameCollaborativeGraphical.h"
#include "BayesianGameIdenticalPayoff.h"
#include "PolicyPureVector.h"

using namespace std;

//Default constructor
LocalBGValueFunctionBGCGWrapper::LocalBGValueFunctionBGCGWrapper(
         const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg, Index LRF)
{        
    _m_bgcg = bgcg;
    _m_LRF = LRF;
}
/*
//Copy constructor.    
LocalBGValueFunctionBGCGWrapper::LocalBGValueFunctionBGCGWrapper(const LocalBGValueFunctionBGCGWrapper& o) 
{
}
//Destructor
LocalBGValueFunctionBGCGWrapper::~LocalBGValueFunctionBGCGWrapper()
{
}
//Copy assignment operator
LocalBGValueFunctionBGCGWrapper& LocalBGValueFunctionBGCGWrapper::operator= (const LocalBGValueFunctionBGCGWrapper& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/

double LocalBGValueFunctionBGCGWrapper::GetValue(Index jpolI) const
{
    throw E(" LocalBGValueFunctionBGCGWrapper::GetValue(Index jpolI) not implemented");

    //convert to individual indices and call
    //return  GetValue(indPols);//(if this turns out practical...)
}
double LocalBGValueFunctionBGCGWrapper::GetValue(vector<Index> indPolsIs) const
{
//    throw E(" LocalBGValueFunctionBGCGWrapper::GetValue(vector<Index> indPols) not implemented");
    
    //Get the BGIP of the LRF that this   LocalBGValueFunctionBGCGWrapper
    //represents a LocalBGValueFunction for
    const BayesianGameIdenticalPayoff* bge = _m_bgcg->GetBGIPforLRF(_m_LRF);

    size_t nrAgents = indPolsIs.size();
    size_t nrAgents2 = bge->GetNrAgents();
    if(nrAgents != nrAgents2)
        throw E("nrAgents != nrAgents2)");
   
    //first we construct the individual policies
    vector<PolicyPureVector*> indPols;
    for(Index agI = 0; agI < nrAgents; agI++)
    {
        PolicyPureVector* polA = new PolicyPureVector(bge, agI, TYPE_INDEX);
        polA->SetIndex(indPolsIs.at(agI));
        indPols.push_back(polA);
    }

    //loop over joint types of LRF _m_LRF, i.e., resricted types jtGI
        //V += V(jtGI, indPols(jtGI) )
    double v = 0.0;
    for(Index JTe=0; JTe < bge->GetNrJointTypes(); JTe++)
    {
        const vector<Index>& indT = bge->JointToIndividualTypeIndices(JTe);

        vector<Index> actions = vector<Index>(nrAgents);
        for(Index agI = 0; agI < nrAgents; agI++)
            actions[agI] = indPols[agI]->GetActionIndex(indT[agI]);

        double ut = bge->GetUtility(indT, actions);        
        double p = bge->GetProbability(JTe);
        v += p * ut;
    }

    //free the individual policies.
    for(Index agI = 0; agI < nrAgents; agI++)
        delete indPols[agI];

    return(v);
}
        
Scope LocalBGValueFunctionBGCGWrapper::GetAgentScope() const
{return _m_bgcg->GetScope(_m_LRF);}

string LocalBGValueFunctionBGCGWrapper::SoftPrint() const
{
    stringstream ss;
    ss << "[LBGVFBGCGWrapper LRF " << _m_LRF << " Scope "
       << SoftPrintVector(GetAgentScope()) << "]";
    return(ss.str());
}
