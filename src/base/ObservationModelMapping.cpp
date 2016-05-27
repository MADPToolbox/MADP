/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ObservationModelMapping.h"

using namespace std;

ObservationModelMapping::ObservationModelMapping(int nrS, int nrJA,
                                                 int nrJO) : 
    ObservationModelDiscrete(nrS, nrJA, nrJO)
{
    Matrix *O;
    for(int a=0;a!=nrJA;++a)
    {
        O=new Matrix(nrS,nrJO);
        O->clear();
        _m_O.push_back(O);
    }
}

ObservationModelMapping::
ObservationModelMapping(const ObservationModelMapping& OM) :
    ObservationModelDiscrete(OM)
{
    Matrix *O;
    for(unsigned int a=0;a!=OM._m_O.size();++a)
    {
        O=new Matrix(*OM._m_O[a]);
        _m_O.push_back(O);
    }
}

ObservationModelMapping::~ObservationModelMapping()
{    
    for(vector<Matrix*>::iterator it=_m_O.begin();
        it!=_m_O.end(); ++it)
        delete(*it);
    _m_O.clear();
}
