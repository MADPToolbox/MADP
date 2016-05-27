/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "MDPValueIteration.h"
#include <fstream>
#include "directories.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "TransitionModelMapping.h"
#include "TransitionModelMappingSparse.h"

using namespace std;

#define DEBUG_MDPValueIteration 0

//Default constructor
MDPValueIteration::MDPValueIteration(const PlanningUnitDecPOMDPDiscrete& pu) :
    MDPSolver(pu)
{
    _m_initialized = false;
}

//Destructor
MDPValueIteration::~MDPValueIteration()
{
}

void MDPValueIteration::Initialize()
{
    StartTimer("Initialize");

    size_t horizon = GetPU()->GetHorizon();
    size_t nrS = GetPU()->GetNrStates();
    size_t nrJA =  GetPU()->GetNrJointActions();

    size_t nrQfunctions;
    if(horizon==MAXHORIZON)
    {
        _m_finiteHorizon=false;
        nrQfunctions=1;
    }
    else
    {
        _m_finiteHorizon=true;
        nrQfunctions=horizon;
    }

    QTable tempTable(nrS,nrJA);
    for(unsigned int s=0;s!=nrS;++s)
        for(unsigned int ja=0;ja!=nrJA;++ja)
            tempTable(s,ja)=0;

    for(Index t=0; t < nrQfunctions; t++)
        _m_QValues.push_back(tempTable);

    _m_initialized = true;

    StopTimer("Initialize");
}

void MDPValueIteration::PlanWithCache(bool computeIfNotCached)
{
    stringstream ss;
    ss << directories::MADPGetResultsDir("GMAA",GetPU()) << "/MDPQtable";
    if(_m_finiteHorizon)
        ss << "_h" << GetPU()->GetHorizon();
    string filenameCache=ss.str();

    PlanWithCache(filenameCache,computeIfNotCached);
}

void MDPValueIteration::PlanWithCache(const string &filenameCache,
                                      bool computeIfNotCached)
{
    if(!_m_initialized)
        Initialize();

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
        ss << "MDPValueIteration::PlanWithCache: "
           << filenameCache << " not cached, bailing out";
        throw(E(ss.str()));
        return;
    }

    // Couldn't open cache file, so compute
    if(!cached)
    {
        Plan();
        if(_m_finiteHorizon)
            QTable::Save(_m_QValues,filenameCache);
        else
            QTable::Save(_m_QValues[0],filenameCache);

#if DEBUG_MDPValueIteration
        cout << "MDPValueIteration::PlanWithCache saved Q values to "
             << filenameCache << endl;
#endif
    }
    else // load Q values from file
    {
        if(_m_finiteHorizon)
            LoadQTables(filenameCache,GetPU()->GetHorizon(),_m_QValues);
        else
            LoadQTable(filenameCache,_m_QValues[0]);

#if DEBUG_MDPValueIteration
        cout << "MDPValueIteration::PlanWithCache loaded Q values from "
             << filenameCache << endl;
#endif

    }
}

QTables MDPValueIteration::GetQTables() const
{
    return(_m_QValues);
}

QTable MDPValueIteration::GetQTable(Index time_step) const
{
    return(_m_QValues.at(time_step));
}

void MDPValueIteration::SetQTables(const QTables &Qs)
{
    _m_QValues=Qs;
}

void MDPValueIteration::SetQTable(const QTable &Q, Index time_step)
{
    _m_QValues[time_step]=Q;
}

/** Duplication of code from the templatized version, but well... */
void MDPValueIteration::PlanSlow()
{
    if(!_m_initialized)
        Initialize();

    StartTimer("Plan");

    size_t horizon = GetPU()->GetHorizon();
    size_t nrS = GetPU()->GetNrStates();
    size_t nrJA =  GetPU()->GetNrJointActions();

    double R_i,R_f,maxQsuc;

    // cache immediate reward for speed
    QTable immReward(nrS,nrJA);
    for(Index sI = 0; sI < nrS; sI++)
        for(Index jaI = 0; jaI < nrJA; jaI++)
            immReward(sI,jaI)=GetPU()->GetReward(sI, jaI);

   
    if(_m_finiteHorizon)
    {
        for(size_t t = horizon - 1; true; t--)
        {
            for(Index sI = 0; sI < nrS; sI++)
            {
                for(Index jaI = 0; jaI < nrJA; jaI++)
                {
                    //calc. expected immediate reward
                    R_i = immReward(sI,jaI);
                    R_f = 0.0;
                    if(t < horizon - 1)
                    {
                        //calc. expected future reward
                        for(Index ssucI = 0; ssucI < nrS; ssucI++)
                        {
                            //find the best action at ssucI
                            maxQsuc = -DBL_MAX;
                            for(Index jasucI = 0; jasucI < nrJA; jasucI++)
                                maxQsuc = max( _m_QValues[t+1](ssucI,jasucI),
                                               maxQsuc);
                            
                            R_f += GetPU()->GetTransitionProbability(sI, jaI,
                                                                     ssucI)
                                * maxQsuc;
                        }//done calc. expected future reward
                    }
                    _m_QValues[t](sI,jaI) = R_i + R_f;
                }//end for jaI
            }//end for sI
            if(t == 0) //escape from (loop t is unsigned!)
                break;
        }
    }
    else // infinite horizon problem
    {
        double maxDelta=DBL_MAX;
        double gamma=GetPU()->GetDiscount();
        QTable oldQtable;

        // in infinite-horizon case, it is typically worth to cache
        // the transition model
        typedef boost::numeric::ublas::compressed_matrix<double> CMatrix;
        vector<CMatrix*> T;
        CMatrix *Ta;
        double p;
        for(unsigned int a=0;a!=nrJA;++a)
        {
#if DEBUG_MDPValueIteration
            PrintTimersSummary();
#endif
            StartTimer("CacheTransitionModel");
            Ta=new CMatrix(nrS,nrS);

            for(unsigned int s=0;s!=nrS;++s)
                for(unsigned int s1=0;s1!=nrS;++s1)
                {
                    p=GetPU()->GetTransitionProbability(s,a,s1);
                    if(p>0)
                        (*Ta)(s,s1)=p;
                }

            T.push_back(Ta);
            StopTimer("CacheTransitionModel");
        }

        Index sI,ssucI;
        while(maxDelta>1e-4)
        {
            StartTimer("Iteration");
            maxDelta=0;
            oldQtable=_m_QValues[0];
            for(Index jaI = 0; jaI < nrJA; jaI++)
            {
                for(CMatrix::const_iterator1 ri=T[jaI]->begin1();
                    ri!=T[jaI]->end1(); ++ri)
                {
                    sI=ri.index1();

                    //calc. expected immediate reward
                    R_i = immReward(sI,jaI);
                    R_f = 0.0;
                    //calc. expected future reward
                    
                    for (CMatrix::const_iterator2 ci = ri.begin();
                         ci != ri.end(); ++ci)
                    {
                        ssucI=ci.index2();

                        //find the best action at ssucI
                        maxQsuc = -DBL_MAX;
                        for(Index jasucI = 0; jasucI < nrJA; jasucI++)
                            maxQsuc = max( oldQtable(ssucI,jasucI),
                                           maxQsuc);
                            
                        R_f += *ci * maxQsuc;
                    }//done calc. expected future reward

                    _m_QValues[0](sI,jaI) = R_i + gamma*R_f;
                    maxDelta=max(maxDelta,abs(oldQtable(sI,jaI)-
                                              _m_QValues[0](sI,jaI)));
                }//end for jaI
            }//end for sI

            StopTimer("Iteration");

#if DEBUG_MDPValueIteration
            cout << "delta " << maxDelta << endl;
            PrintTimersSummary();
#endif
        }

        for(unsigned int a=0;a!=nrJA;++a)
            delete T[a];
    }

    StopTimer("Plan");

#if DEBUG_MDPValueIteration
    PrintTimersSummary();
#endif
}

void MDPValueIteration::Plan()
{
    if(!_m_initialized)
        Initialize();

    StartTimer("Plan");
    
    size_t nrJA =  GetPU()->GetNrJointActions();
    const TransitionModelMappingSparse *tms=0;
    const TransitionModelMapping *tm=0;
    const TransitionModelDiscrete *tmd=
        GetPU()->GetTransitionModelDiscretePtr();

    if(tmd==0)
        PlanSlow(); // just use GetTransitionProbability()
    else if((tms=dynamic_cast<const TransitionModelMappingSparse *>(tmd)))
    {
        std::vector<const TransitionModelMappingSparse::SparseMatrix *> T;
        for(unsigned int a=0;a!=nrJA;++a)
            T.push_back(tms->GetMatrixPtr(a));
        Plan(T);
    }
    else if((tm=dynamic_cast<const TransitionModelMapping *>(tmd)))
    {
        std::vector<const TransitionModelMapping::Matrix *> T;
        for(unsigned int a=0;a!=nrJA;++a)
            T.push_back(tm->GetMatrixPtr(a));
            Plan(T);
    }
    else 
        throw(E("MDPValueIteration::Plan() TransitionModelDiscretePtr not handled"));
    
    StopTimer("Plan");
}

template <class M>
void MDPValueIteration::Plan(std::vector<const M*> T)
{
    size_t horizon = GetPU()->GetHorizon();
    size_t nrS = GetPU()->GetNrStates();
    size_t nrJA =  GetPU()->GetNrJointActions();
    
    Index sI,ssucI;
    double gamma=GetPU()->GetDiscount();
    double R_i,R_f,maxQsuc;
    
    // cache immediate reward for speed
    QTable immReward(nrS,nrJA);
    for(Index sI = 0; sI < nrS; sI++)
        for(Index jaI = 0; jaI < nrJA; jaI++)
            immReward(sI,jaI)=GetPU()->GetReward(sI, jaI);
    
    if(_m_finiteHorizon)
    {
        for(size_t t = horizon - 1; true; t--)
        {
            StartTimer("Iteration");
            for(Index jaI = 0; jaI < nrJA; jaI++)
            {
                for(typename M::const_iterator1 ri=T[jaI]->begin1();
                    ri!=T[jaI]->end1(); ++ri)
                {
                    sI=ri.index1();
                    
                    //calc. expected immediate reward
                    R_i = immReward(sI,jaI);
                    R_f = 0.0;
                    if(t < horizon - 1)
                    {
                        //calc. expected future reward
                        for (typename M::const_iterator2 ci = ri.begin();
                             ci != ri.end(); ++ci)
                        {
                            ssucI=ci.index2();
                            //find the best action at ssucI
                            maxQsuc = -DBL_MAX;
                            for(Index jasucI = 0; jasucI < nrJA; jasucI++)
                                maxQsuc = std::max( _m_QValues[t+1](ssucI,jasucI),
                                                    maxQsuc);
                            
                            R_f += *ci * maxQsuc;
                        }//done calc. expected future reward
                    }
                    _m_QValues[t](sI,jaI) = R_i + gamma*R_f;
                }//end for jaI
            }//end for sI
            StopTimer("Iteration");
            if(t == 0) //escape from (loop t is unsigned!)
                break;
        }
    }
    else // infinite horizon problem
    {
        double maxDelta=DBL_MAX;
        QTable oldQtable;
        
        while(maxDelta>1e-4)
        {
            StartTimer("Iteration");
            maxDelta=0;
            oldQtable=_m_QValues[0];
            for(Index jaI = 0; jaI < nrJA; jaI++)
            {
                for(typename M::const_iterator1 ri=T[jaI]->begin1();
                    ri!=T[jaI]->end1(); ++ri)
                {
                    sI=ri.index1();
                    
                    //calc. expected immediate reward
                    R_i = immReward(sI,jaI);
                    R_f = 0.0;
                    //calc. expected future reward
                    
                    for (typename M::const_iterator2 ci = ri.begin();
                         ci != ri.end(); ++ci)
                    {
                        ssucI=ci.index2();
                        
                        //find the best action at ssucI
                        maxQsuc = -DBL_MAX;
                        for(Index jasucI = 0; jasucI < nrJA; jasucI++)
                            maxQsuc = std::max( oldQtable(ssucI,jasucI),
                                                maxQsuc);
                            
                        R_f += *ci * maxQsuc;
                    }//done calc. expected future reward
                    
                    _m_QValues[0](sI,jaI) = R_i + gamma*R_f;
                    maxDelta=std::max(maxDelta,
                                      std::abs(oldQtable(sI,jaI)-
                                               _m_QValues[0](sI,jaI)));
                }//end for jaI
            }//end for sI
            
            StopTimer("Iteration");

#if DEBUG_MDPValueIteration
            std::cout << "delta " << maxDelta << std::endl;
            PrintTimersSummary();
#endif
        }
    }
}
template
void MDPValueIteration::Plan(std::vector<const TransitionModelMappingSparse::SparseMatrix*> T);
template
void MDPValueIteration::Plan(std::vector<const TransitionModelMapping::Matrix*> T);
