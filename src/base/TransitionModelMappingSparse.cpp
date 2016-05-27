/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "TransitionModelMappingSparse.h"

using namespace std;

TransitionModelMappingSparse::TransitionModelMappingSparse(int nrS, int nrJA) :
    TransitionModelDiscrete(nrS, nrJA)
{    
    SparseMatrix *T;
    for(int a=0;a!=nrJA;++a)
    {
        T=new SparseMatrix(nrS,nrS);
        _m_T.push_back(T);
    }
}

TransitionModelMappingSparse::
TransitionModelMappingSparse(const TransitionModelMappingSparse& TM) :
    TransitionModelDiscrete(TM)
{
    SparseMatrix *T;
    for(unsigned int a=0;a!=TM._m_T.size();++a)
    {
        T=new SparseMatrix(*TM._m_T[a]);
        _m_T.push_back(T);
    }
}

TransitionModelMappingSparse::~TransitionModelMappingSparse()
{    
    for(vector<SparseMatrix*>::iterator it=_m_T.begin();
        it!=_m_T.end(); ++it)
        delete(*it);
    _m_T.clear();
}
