/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "MonahanPlanner.h"
#include "BeliefValue.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointActionObservationHistoryTree.h"
#include "JointBelief.h"
#include <fstream>

using namespace std;

//Default constructor
MonahanPlanner::MonahanPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                               bool doIncPrune) :
    AlphaVectorPlanning(pu),
    _m_doIncPrune(doIncPrune),
    _m_initialized(false),
    _m_alreadyComputed(false),
    _m_maxNrAlphas(GetPU()->GetHorizon(),0),
    _m_resultsFilename("")
{
}

MonahanPlanner::MonahanPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                               bool doIncPrune) :
    AlphaVectorPlanning(pu),
    _m_doIncPrune(doIncPrune),
    _m_initialized(false),
    _m_alreadyComputed(false),
    _m_maxNrAlphas(GetPU()->GetHorizon(),0),
    _m_resultsFilename("")
{
}

//Destructor
MonahanPlanner::~MonahanPlanner()
{
}

void MonahanPlanner::Save(const std::string &filename) const
{
    AlphaVectorPlanning::ExportValueFunction(filename,_m_qFunction);
}


void MonahanPlanner::Load(const std::string &filename)
{
    if(!MonahanPlanner::_m_initialized)
        Initialize();
    _m_qFunction= AlphaVectorPlanning::ImportValueFunction(filename,
                                                           GetPU()->GetHorizon(),
                                                           GetPU()->GetNrJointActions(),
                                                           GetPU()->GetNrStates());
    _m_alreadyComputed=true;
}

double MonahanPlanner::GetQ(Index jaohI, Index jaI) const
{
    throw(E("MonahanPlanner::GetQ() not sure this is correct"));
    const JointActionObservationHistoryTree *jaoht=
        GetPU()->GetJointActionObservationHistoryTree(jaohI);
    size_t t=jaoht->GetLength();
    
    JointBeliefInterface *b=GetPU()->GetJointBeliefInterface(jaohI);
    ValueFunctionPOMDPDiscrete V=_m_qFunction[t-1][jaI];
    return(BeliefValue::GetValue(*b,V));
}

double MonahanPlanner::GetQ(const JointBeliefInterface &b,
                            Index jaI) const
{
    throw(E("MonahanPlanner::GetQ not implemented for stationary case"));
    return(42);
}

double MonahanPlanner::GetQ(const JointBeliefInterface &b,
                            Index t,
                            Index jaI) const
{
    return(BeliefValue::GetValue(b,_m_qFunction.at(t).at(jaI)));
}

void MonahanPlanner::PlanWithCache(const string &filenameCache,
                                   bool computeIfNotCached)
{
    if(!_m_initialized)
        Initialize();

    bool cached;

    {
        ifstream fp(string(filenameCache + "_t0").c_str());
        if(!fp)
            cached=false;
        else
            cached=true;
    }

    if(!cached && !computeIfNotCached)
    {
        stringstream ss;
        ss << "MonahanPlanner::PlanWithCache: "
           << filenameCache << " not cached, bailing out";
        throw(E(ss.str()));
        return;
    }

    // Couldn't open cache file, so compute
    if(!cached)
    {
        Plan();
        Save(filenameCache);
    }
    else 
    {
        cout << "MonahanPlanner: loading cached " << filenameCache
             << endl;
        Load(filenameCache);
    }
}

void MonahanPlanner::CheckMaxNrVectors(size_t maxNrAlphas, size_t nrAlphas) const
{
    if(maxNrAlphas && nrAlphas>maxNrAlphas)
    {
        stringstream ss;
        ss << "MonahanPlanner::CheckMaxNrVectors too many alpha vectors "
           << nrAlphas << ">" << maxNrAlphas;
        throw(E(ss.str()));
    }
}

size_t MonahanPlanner::GetNrVectors() const
{
    size_t k=0;
    for(Index t=0;t!=_m_qFunction.size();++t)
        for(Index a=0;a!=_m_qFunction.at(t).size();++a)
            k+=_m_qFunction.at(t).at(a).size();

    return(k);
}

void MonahanPlanner::Plan()
{
    if(!MonahanPlanner::_m_initialized)
        Initialize();
    if(_m_alreadyComputed)
        return;
    _m_alreadyComputed=true;

    int h=GetPU()->GetHorizon(),nrA=GetPU()->GetNrJointActions(),
        nrS=GetPU()->GetNrStates();
    QFunctionsDiscrete Q;

    _m_qFunction=vector<QFunctionsDiscrete>(h);

    AlphaVector alpha(nrS);
    ValueFunctionPOMDPDiscrete Q0;

    ValueFunctionPOMDPDiscrete immR=GetImmediateRewardValueFunction();
    CheckMaxNrVectors(_m_maxNrAlphas.at(h-1),immR.size());

    _m_timeStep=h-1;
    while(_m_timeStep>=0)
    {
        if(_m_timeStep==h-1) // last time step, copy in immediate reward function
        {
            QFunctionsDiscrete Qs=ValueFunctionToQ(immR);
            for(int a=0;a!=nrA;a++)
                _m_qFunction[h-1].push_back(Qs[a]);
        }
        else
        {
            Q=BackupStage(_m_qFunction[_m_timeStep+1],_m_maxNrAlphas.at(_m_timeStep));

            CheckMaxNrVectors(_m_maxNrAlphas.at(_m_timeStep),GetNrVectors());
            for(int a=0;a!=nrA;a++)
                _m_qFunction[_m_timeStep].push_back(Q[a]);
        }

        cout << SoftPrintBrief() << "Planner: t " << _m_timeStep << " contains <";
        for(int a=0;a!=nrA;a++)
            cout << " " << _m_qFunction[_m_timeStep][a].size();
        cout << " > vectors (total "
             << GetNrVectors() << ")" << endl;
        _m_timeStep--;
    }

    if(_m_resultsFilename!="")
        ExportValueFunction(_m_resultsFilename,_m_qFunction);

    cout << SoftPrintBrief() << "Planner[h=" 
         << GetPU()->GetHorizon() << "]: Vjb0="
         << BeliefValue::GetValue(Belief(*GetPU()->GetDPOMDPD()->GetISD()),
                                  _m_qFunction[0])
         << endl;
}

void MonahanPlanner::SetResultsFilename(const std::string &filename)
{
    _m_resultsFilename=filename;
}
