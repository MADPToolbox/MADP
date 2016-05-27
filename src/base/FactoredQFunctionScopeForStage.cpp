/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "FactoredQFunctionScopeForStage.h"

using namespace std;

//Default constructor
FactoredQFunctionScopeForStage::FactoredQFunctionScopeForStage()
    :
    _m_agentScopes(0),
    _m_sfacScopes(0)
{
}

void 
FactoredQFunctionScopeForStage::AddLocalQ( const Scope& sfacS, const Scope& agS)
{
    _m_sfacScopes.push_back(sfacS);
    _m_agentScopes.push_back(agS);
}

void 
FactoredQFunctionScopeForStage::RemoveLocalQ( Index j)
{
    if(j >= _m_sfacScopes.size())
        throw E("FactoredQFunctionScopeForStage::RemoveLocalQ( Index j), index j out of bounds");

    _m_sfacScopes.erase(_m_sfacScopes.begin() + j );
    _m_agentScopes.erase(_m_agentScopes.begin() + j );
}

string FactoredQFunctionScopeForStage::SoftPrint() const
{
    stringstream ss; 
    for(Index q=0; q < _m_sfacScopes.size(); q++)
    {
        ss << q << "-th local Q function, agentScope="<< _m_agentScopes.at(q) <<
            ", sfacScope=" << _m_sfacScopes.at(q) << endl;
    }
    return (ss.str());
}
