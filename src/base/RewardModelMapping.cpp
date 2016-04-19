/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "RewardModelMapping.h"

using namespace std;
#define DEBUG 0
RewardModelMapping::RewardModelMapping(size_t nrS, size_t nrJA, 
                                       const string &s_str,
                                       const string &ja_str) : 
    RewardModel(nrS, nrJA),
    _m_R(nrS,nrJA)
{
#if DEBUG    
    cout << "Initialized a reward model with " 
        << nrS << "'"<< s_str <<"' and "
        << nrJA << "'"<< ja_str <<"'"<< endl;
#endif
    _m_s_str = s_str;
    _m_ja_str = ja_str;
    for(Index i=0; i < nrS; i++)
        for(Index j=0; j < nrJA; j++)
            _m_R(i, j) = 0.0;
}

RewardModelMapping::~RewardModelMapping()
{
}

string RewardModelMapping::SoftPrint() const
{
    stringstream ss;
    double r;
    ss << _m_s_str <<"\t"<< _m_ja_str <<"\t"
       << "R(" << _m_s_str <<","<< _m_ja_str
       <<  ") (rewards of 0 are not printed)"<<endl;
    for(Index s_i = 0; s_i < GetNrStates(); s_i++)
        for(Index ja_i = 0; ja_i < GetNrJointActions(); ja_i++)
        {
            r=Get(s_i, ja_i);
            if(std::abs(r)>0)
                ss << s_i << "\t" << ja_i << "\t" << r << endl;
        }
    return(ss.str());
}

