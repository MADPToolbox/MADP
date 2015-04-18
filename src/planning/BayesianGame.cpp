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

#include "BayesianGame.h"
#include "E.h"

using namespace std;

BayesianGame::BayesianGame(size_t nrAgents, const vector<size_t> &nrActions,  
                           const vector<size_t> &nrTypes) :
    BayesianGameBase(nrAgents, nrActions, nrTypes)
{
    //initialize the utility functions
    RewardModelMapping r(_m_nrJTypes, _m_nrJA, "type", "ja");
    for(Index i = 0; i < _m_nrAgents; i++)
        _m_utilFuncs.push_back(r); //r is copied by value so we get independent
        //copies in _m_utilFuncs.

}
//Copy assignment constructor.    
BayesianGame::BayesianGame(const BayesianGame& o)  :
     BayesianGameBase(o)
{
    throw E("trying to copy construct a BayesianGame - not implemented. (is this necessary?)");
}
//Destructor

bool BayesianGame::SetInitialized(bool b)
{
    _m_initialized = b;
    return(true);
}

void BayesianGame::Print() const
{
    BayesianGameBase::Print();
    
    cout << "Utility functions:"<<endl;
    for(Index aI = 0; aI < _m_nrAgents; aI++)
    {
        cout << "Agent "<<aI<<":"<<endl;
        _m_utilFuncs[aI].Print();
    }
}

