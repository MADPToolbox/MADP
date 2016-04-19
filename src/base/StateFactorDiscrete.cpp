/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "StateFactorDiscrete.h"

using namespace std;

//Default constructor
StateFactorDiscrete::StateFactorDiscrete(const string &n, 
                                         const string &d)
    : NamedDescribedEntity(n,d)
{
}
StateFactorDiscrete::StateFactorDiscrete(size_t nrVs, const string &n,
                                         const string &d)
    : NamedDescribedEntity(n,d)
{
    _m_domainSize = nrVs;
    //perhaps also add strings 0...nrVs-1 to _m_domainValues ?
    
}
//Copy constructor.    
StateFactorDiscrete::StateFactorDiscrete(const StateFactorDiscrete& o) 
{
}
//Destructor
StateFactorDiscrete::~StateFactorDiscrete()
{
}
//Copy assignment operator
StateFactorDiscrete& StateFactorDiscrete::operator= (const StateFactorDiscrete& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}

string StateFactorDiscrete::SoftPrint() const
{
    stringstream ss;

    ss << "SF '"<< GetName() << "' ("<<GetDescription()<<"), values: {";
    vector<string>::const_iterator it = _m_domainValues.begin();
    vector<string>::const_iterator last = _m_domainValues.end();
    while(it != last)
    {
        if(it != _m_domainValues.begin() )
            ss << ", ";
        ss << *it;
        it++;
    }
    ss << "}";
    return(ss.str() );
}


Index StateFactorDiscrete::
AddStateFactorValue(const string &v)
{
    _m_domainValues.push_back(v);
    Index i = _m_domainSize++;
    return(i);
}
