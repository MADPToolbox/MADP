/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * João Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "EventObservationModelMappingSparse.h"

using namespace std;

EventObservationModelMappingSparse::EventObservationModelMappingSparse(int nrS,
                                                             int nrJA,
                                                             int nrJO) :
    ObservationModelDiscrete(nrS, nrJA, nrJO)
{
    SparseMatrix *O;
    for(int a=0;a!=nrJA;++a)
    {
        std::vector<SparseMatrix*> S;
        for(int joI=0;joI!=nrJO;++joI)
        {
            O=new SparseMatrix(nrS,nrS);
            O->clear();
            S.push_back(O);
        }
        _m_O.push_back(S);
    }
}

EventObservationModelMappingSparse::
EventObservationModelMappingSparse(const EventObservationModelMappingSparse& OM) :
    ObservationModelDiscrete(OM)
{
    SparseMatrix *O;
    for(unsigned int a=0;a!=OM._m_O.size();++a)
    {
        std::vector<SparseMatrix*> S;
        for(unsigned int joI=0;joI!=OM._m_O.at(0).size();++joI)
        {
            O=new SparseMatrix(*OM._m_O[a][joI]);
            S.push_back(O);
        }
        _m_O.push_back(S);
    }    
}

EventObservationModelMappingSparse::~EventObservationModelMappingSparse()
{
    for(size_t i = 0; i < _m_O.size(); i++)
    {
      for(vector<SparseMatrix*>::iterator it=_m_O.at(i).begin(); it!=_m_O.at(i).end(); ++it)
        delete(*it);
      _m_O.at(i).clear();
    }
}
