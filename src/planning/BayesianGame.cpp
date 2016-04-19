/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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

