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

#include "LocalBGValueFunctionVector.h"
#include "BayesianGameCollaborativeGraphical.h"
#include "Scope.h"

using namespace std;

//Default constructor
LocalBGValueFunctionVector::LocalBGValueFunctionVector(
        const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &cgbg, 
         Scope agentScope)
{
    _m_agentScope = agentScope;
    _m_bgcg = cgbg;

    //    we fill _m_nrIndivPols; and compute
    _m_nrJointPols = 1;
    Scope::iterator it = _m_agentScope.begin();
    Scope::iterator last = _m_agentScope.end();
    while(it != last)
    {
        Index agI = *it;
        size_t polsThisAg = CastLIndexToIndex(_m_bgcg->GetNrPolicies(agI));
        _m_nrJointPols *= polsThisAg;
        _m_nrIndivPols.push_back(polsThisAg);
        it++;
    }
    _m_values = vector<double>(_m_nrJointPols, 0.0);   
}
/*LocalBGValueFunctionVector::LocalBGValueFunctionVector(*/
        /*BayesianGameCollaborativeGraphical* cgbg, */
        /*vector< Index > thisAgNeighbors)*/
/*{*/
    /*cerr << "WARNING LocalBGValueFunctionVector constructor not implemented yet ";*/
/*}*/
//Copy constructor.    
//LocalBGValueFunctionVector::LocalBGValueFunctionVector(const LocalBGValueFunctionVector& o) 
//{
//}
//Destructor
LocalBGValueFunctionVector::~LocalBGValueFunctionVector()
{
}
//Copy assignment operator
//LocalBGValueFunctionVector& LocalBGValueFunctionVector::operator= (const LocalBGValueFunctionVector& o)
//{
//    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

//    return *this;
//}


/* the following functions are defined in the .h:
 *
void LocalBGValueFunctionVector::
SetValue(Index jpolI, double best_response_value)
{
    cerr<< "LocalBGValueFunctionVector::SetValue not implemented yet";
}
void LocalBGValueFunctionVector::
SetValue(vector<Index> indPols, double best_response_value)
{
    cerr<< "LocalBGValueFunctionVector::SetValue not implemented yet";
}
double LocalBGValueFunctionVector::
GetValue(Index jpolI) const
{
    cerr<< "LocalBGValueFunctionVector::GetValue(Index jpolI) not implemented yet";
}
double LocalBGValueFunctionVector::
GetValue(vector<Index> indPols) const
{
    cerr<< "LocalBGValueFunctionVector::GetValue(vector<Index> indPols)  not implemented yet";
}
Scope LocalBGValueFunctionVector::
GetAgentScope() const
{
    cerr<< "LocalBGValueFunctionVector::GetAgentScope()  not implemented yet";
}

*/

string LocalBGValueFunctionVector::SoftPrint() const
{
    stringstream ss;
    ss << "[LBGVFV AgentScope " << SoftPrintVector(_m_agentScope) << " v "
       << SoftPrintVector(_m_values) << " #pol " << SoftPrintVector(_m_nrIndivPols)
       << " #jpol " << _m_nrJointPols << "]";

    return(ss.str());
}
