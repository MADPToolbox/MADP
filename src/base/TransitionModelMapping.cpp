/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "TransitionModelMapping.h"

using namespace std;

#define DEBUG_TM_MAPPING 0

TransitionModelMapping::TransitionModelMapping(int nrS, int nrJA) :
    TransitionModelDiscrete(nrS, nrJA)
{    
    Matrix *T;
    for(int a=0;a!=nrJA;++a)
    {
        T=new Matrix(nrS,nrS);
        T->clear();
        _m_T.push_back(T);
    }
}

TransitionModelMapping::
TransitionModelMapping(const TransitionModelMapping& TM) :
    TransitionModelDiscrete(TM)
{
    Matrix *T;
    for(unsigned int a=0;a!=TM._m_T.size();++a)
    {
        T=new Matrix(*TM._m_T[a]);
        _m_T.push_back(T);
    }
}

TransitionModelMapping::~TransitionModelMapping()
{    
    for(vector<Matrix*>::iterator it=_m_T.begin();
        it!=_m_T.end(); ++it)
        delete(*it);
    _m_T.clear();
}
