/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ObservationModelMappingSparse.h"

using namespace std;

ObservationModelMappingSparse::ObservationModelMappingSparse(int nrS,
                                                             int nrJA,
                                                             int nrJO) :
    ObservationModelDiscrete(nrS, nrJA, nrJO)
{
    SparseMatrix *O;
    for(int a=0;a!=nrJA;++a)
    {
        O=new SparseMatrix(nrS,nrJO);
        _m_O.push_back(O);
    }
}

ObservationModelMappingSparse::
ObservationModelMappingSparse(const ObservationModelMappingSparse& OM) :
    ObservationModelDiscrete(OM)
{
    SparseMatrix *O;
    for(unsigned int a=0;a!=OM._m_O.size();++a)
    {
        O=new SparseMatrix(*OM._m_O[a]);
        _m_O.push_back(O);
    }
}

ObservationModelMappingSparse::~ObservationModelMappingSparse()
{    
    for(vector<SparseMatrix*>::iterator it=_m_O.begin();
        it!=_m_O.end(); ++it)
        delete(*it);
    _m_O.clear();
}
