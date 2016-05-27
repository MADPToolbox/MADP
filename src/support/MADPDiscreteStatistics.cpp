/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "MADPDiscreteStatistics.h"
#include "VectorTools.h"
#include <climits>

using namespace std;

//Default constructor
MADPDiscreteStatistics::MADPDiscreteStatistics(MultiAgentDecisionProcessDiscreteInterface* madp, size_t h)
    :
        _m_madp(madp),
        _m_h(h)
{
}
//Copy constructor.    
MADPDiscreteStatistics::MADPDiscreteStatistics(const MADPDiscreteStatistics& o) 
    :
        _m_madp(o._m_madp)
        ,_m_h(o._m_h)
{
}
//Destructor
MADPDiscreteStatistics::~MADPDiscreteStatistics()
{
    ;
}
//Copy assignment operator
MADPDiscreteStatistics& MADPDiscreteStatistics::operator= (const MADPDiscreteStatistics& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    _m_madp = o._m_madp;
    _m_h = o._m_h;


    return *this;
}
        
size_t MADPDiscreteStatistics::ComputeNrJointActionObservationHistories()
{
    size_t nrAgents = _m_madp->GetNrAgents();
    const vector<size_t>& nrAcs = _m_madp->GetNrActions();
    const vector<size_t>& nrObs = _m_madp->GetNrObservations();
    vector<size_t> nrAOs(nrAgents, 0);
    for(Index agI=0; agI < nrAgents; agI++)
        nrAOs.at(agI) = nrAcs.at(agI) * nrObs.at(agI);

    
    vector<size_t> nrAOH(nrAgents,0);
    vector<size_t> nrAOH_prevStage(nrAgents,0);
    size_t nrJAOH=1;    
    for(Index t=0; t < _m_h; t++)
    {
        for(Index agI=0; agI < nrAgents; agI++)
        {
            if(t==0)
                nrAOH.at(agI) = 1;
            else
                nrAOH.at(agI) = nrAOH_prevStage.at(agI) * nrAOs.at(agI);
        }
        size_t nrJAOH_t=VectorTools::VectorProduct(nrAOH);
        nrJAOH += nrJAOH_t;

        nrAOH_prevStage = nrAOH;
    }
    return nrJAOH;

}

size_t MADPDiscreteStatistics::ComputeEstimatedSizeForCachingJointBeliefs()
{

    size_t doubleSizeInChars = sizeof(double);
    size_t doubleSizeInBits = doubleSizeInChars * CHAR_BIT;

    //the number of joint beliefs that would have to be cached
    size_t nrJBs = ComputeNrJointActionObservationHistories();
    //the size of those joint beliefs:
    size_t JBSize = _m_madp->GetNrStates();

    size_t JBCacheSizeInBits = nrJBs* JBSize * doubleSizeInBits;
    size_t JBCacheSizeInMB = (double) JBCacheSizeInBits / (8 * 1024 * 1024);
    return JBCacheSizeInMB;

}
