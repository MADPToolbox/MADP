/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

#include "MonahanPOMDPPlanner.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BeliefValue.h"
#include "Belief.h"

using namespace std;

//Default constructor
MonahanPOMDPPlanner::MonahanPOMDPPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                                         bool doIncPrune) :
    MonahanPlanner(pu,doIncPrune)
{
}

MonahanPOMDPPlanner::MonahanPOMDPPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                                         bool doIncPrune) :
    MonahanPlanner(pu,doIncPrune)
{
}

//Destructor
MonahanPOMDPPlanner::~MonahanPOMDPPlanner()
{
}

void MonahanPOMDPPlanner::Initialize()
{
    AlphaVectorPlanning::Initialize();
    MonahanPlanner::_m_initialized=true;
}

QFunctionsDiscrete 
MonahanPOMDPPlanner::BackupStage(const QFunctionsDiscrete &Q,
                                 size_t maxNrAlphas)
{
    int nrA=GetPU()->GetNrJointActions();
    QFunctionsDiscrete Q1(nrA);

    GaoVectorSet G=BackProject(QFunctionsToValueFunction(Q));

    size_t nrVectorsComputed=0;

    // Do the cross-sums, results are stored in V1
    cout << "MonahanPOMDPPlanner::BackupStage <"; cout.flush();
    for(GaoVectorSetIndex a=0;a!=nrA;a++)
    {
        MonahanCrossSum(G,Q1,a,_m_doIncPrune,maxNrAlphas);
        nrVectorsComputed+=Q1[a].size();
        cout << " " << Q1[a].size(); cout.flush();
        CheckMaxNrVectors(maxNrAlphas,GetNrVectors()+nrVectorsComputed);
    }
    cout << ">" << endl;
    return(Q1);
}

void MonahanPOMDPPlanner::MonahanCrossSum(const GaoVectorSet &G,
                                          QFunctionsDiscrete &Q,
                                          Index a,
                                          bool doIncPrune,
                                          size_t maxNrAlphas) const
{
    Index nrS=GetPU()->GetNrStates(),
        nrO=GetPU()->GetNrJointObservations();
    AlphaVector alpha(nrS);

#if DEBUG_AlphaVectorPlanning_CrossSum    
    cout << "AlphaVectorPlanning::MonahanCrossSum for action " << a << endl;
#endif

    // initialize with the number of vectors already computed for other time steps
    size_t nrVectorsComputed=GetNrVectors();

    // Do the cross-sums, creates G_a of (3.25)
    VectorSet Ga=*G[a][0];
    for(Index o=1;o!=nrO;o++)
    {
        if(doIncPrune)
        {
            VectorSet Ga2=CrossSum(Ga,*G[a][o]);
#if 0
            VectorSet Ga1=Prune(Ga2,maxNrAlphas);
            Ga=Ga1;
#else
            Ga=Prune(Ga2);
#endif
        }
        else
        {
            VectorSet Ga1=CrossSum(Ga,*G[a][o]);
            Ga=Ga1;
        }
#if DEBUG_AlphaVectorPlanning_CrossSum    
        if(maxNrAlphas)
            cout << "AlphaVectorPlanning::MonahanCrossSum nrAlphas " << Ga.size1()
                 << " (max " << maxNrAlphas << ")" << endl;
#endif

        CheckMaxNrVectors(maxNrAlphas,nrVectorsComputed+Ga.size1());
    }
     
    // Add the resulting vectors to V (HV_n of (3.25))
    for(Index k=0;k!=Ga.size1();++k)
    {
        alpha.SetAction(a);
        for(Index s=0;s!=nrS;s++)
            alpha.SetValue(Ga(k,s),s);
        Q[a].push_back(alpha);
        nrVectorsComputed++;
    }

    if(!doIncPrune)
    {
        Q[a]=Prune(Q[a]);
        CheckMaxNrVectors(maxNrAlphas,nrVectorsComputed+Q[a].size());
        // THIS IS WRONG
        throw(E("MonahanPOMDPPlanner fix error"));
        if(maxNrAlphas && Q.size()>maxNrAlphas)
        {
            stringstream ss;
            ss << "AlphaVectorPlanning::MonahanCrossSum() too many alpha vectors "
               << Q.size() << ">" << maxNrAlphas;
            throw(E(ss.str()));
        }
    }
}
