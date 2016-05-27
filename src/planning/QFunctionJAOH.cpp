/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "QFunctionJAOH.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include <fstream>

#define DEBUG_QHEUR_COMP 0

using namespace std;

QFunctionJAOH::QFunctionJAOH(
    const PlanningUnitDecPOMDPDiscrete *pu) :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOHInterface()
{
}
QFunctionJAOH::QFunctionJAOH(
    const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    QFunctionForDecPOMDP(pu), //virtual base first
    QFunctionJAOHInterface()
{
}

void QFunctionJAOH::ComputeWithCachedQValues(const string &filenameCache,
                                             bool computeIfNotCached)
{
    bool cached;

    {
        ifstream fp(filenameCache.c_str());
        if(!fp)
            cached=false;
        else
            cached=true;
    }

    if(!cached && !computeIfNotCached)
    {
        stringstream ss;
        ss << "QFunctionJAOH::ComputeWithCachedQValues: Q function "
           << filenameCache << " not cached, bailing out";
        throw(E(ss.str()));
        return;
    }

    // Couldn't open cache file, so compute
    if(!cached)
    {
        Compute();
        QTable::Save(_m_QValues,filenameCache);

#if DEBUG_QHEUR_COMP
        cout << "QFunctionJAOH::ComputeWithCachedQValues saved Q values to "
             << filenameCache << endl;
#endif
    }
    else // load Q values from file
    {
#if 0 // don't use this, it makes a copy (so you need twice the memory)
        _m_QValues=
            MDPSolver::LoadQTable(filenameCache,
                                  GetPU()->
                                  GetNrJointActionObservationHistories(),
                                  GetPU()->GetNrJointActions());
#else
        QTable::Load(filenameCache,
                     GetPU()->GetNrJointActionObservationHistories(),
                     GetPU()->GetNrJointActions(),
                     _m_QValues);
#endif

#if DEBUG_QHEUR_COMP
        cout << "QFunctionJAOH::ComputeWithCachedQValues loaded Q values from "
             << filenameCache << endl;
#endif
    }
}
