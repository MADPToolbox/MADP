/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek
 * Matthijs Spaan
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

using namespace std;

#include <iostream>
#include <fstream>
#include "argumentUtils.h"
#include "argumentHandlers.h"
#include "MADPParser.h"
#include "directories.h"
#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "POMDPDiscrete.h"

#include "ProblemDecTiger.h"
#include "ProblemFireFighting.h"
#include "ProblemFireFightingFactored.h"
#include "ProblemFireFightingGraph.h"
#include "ProblemAloha.h"

#include "QAVParameters.h"
#include "QMDP.h"
#include "QPOMDP.h"
#include "QBG.h"
#include "QHybrid.h"
#include "QAV.h"
#include "MonahanPOMDPPlanner.h"
#include "MonahanBGPlanner.h"
#include "TreeIncPruneBGPlanner.h"
#include "QBGPlanner_TreeIncPruneBnB.h"
#include "NullPlanner.h"

using namespace ArgumentHandlers;
namespace ArgumentUtils {

DecPOMDPDiscreteInterface* GetDecPOMDPDiscreteInterfaceFromArgs(const Arguments& args)
{
    DecPOMDPDiscreteInterface * dp = 0;
    switch(args.problem_type) 
    {

        case(ProblemType::DT):
        {
            ProblemDecTiger* p = new ProblemDecTiger();            
            dp = p;
            break;
        }
        case(ProblemType::FF):
        {
            dp = new ProblemFireFighting(args.nrAgents, args.nrHouses, 
                    args.nrFLs);
            break;
        }
        case(ProblemType::FFF):
        {
            ProblemFireFightingFactored* p = 
                new ProblemFireFightingFactored(args.nrAgents,
                                                args.nrHouses,
                                                args.nrFLs);
            if(args.cache_flat_models)
                p->CacheFlatModels(args.sparse);
            dp = p;
            break;
        }
        case(ProblemType::FFG):
        {
            ProblemFireFightingGraph* p = 
                new ProblemFireFightingGraph(args.nrAgents,
                                             args.nrFLs);
            if(args.cache_flat_models)
                p->CacheFlatModels(args.sparse);
            dp = p;
            break;
        }
        case(ProblemType::Aloha):
        {
            ProblemAloha* p =
                new ProblemAloha(   args.islandConf,
                                    args.alohaVariation,
                                    args.maxBacklog,
                                    args.nrAgents );

            if(args.cache_flat_models)
                p->CacheFlatModels(args.sparse);
            dp = p;
            break;
        }

        case(ProblemType::PARSE):
        default: //try to open the .dpomdp file and parse it
        {
            string dpomdpFile=directories::MADPGetProblemFilename(args);
            // load the DecPOMDP problem 
            if(args.isTOI)
            {
                TOIFactoredRewardDecPOMDPDiscrete *toi=
                    new TOIFactoredRewardDecPOMDPDiscrete("","",dpomdpFile,
                        args.cache_flat_models);
                if(args.sparse)
                    toi->SetSparse(true);
                MADPParser parser(toi);
#if 0 // this is done internally now
                if(args.cache_flat_models)
                {
                    if(args.sparse)
                        toi->CreateCentralizedSparseModels();
                    else
                        toi->CreateCentralizedFullModels();
                }                
#endif
                dp=toi;
            }
            else{
                if(strstr(dpomdpFile.c_str(),".pgmx"))
                    dp = GetFactoredDecPOMDPDiscreteInterfaceFromArgs(args);
                else if(
                        (strstr(dpomdpFile.c_str(),".pomdp"))
                    ||  (strstr(dpomdpFile.c_str(),".POMDP"))
                    )
                {
                    POMDPDiscrete* pomdp = 
                        new POMDPDiscrete("","",dpomdpFile);
                    if(args.sparse)
                        pomdp->SetSparse(true);
                    MADPParser parser(pomdp);
                    dp = pomdp;
                }
                else
                {
                    DecPOMDPDiscrete* decpomdp = 
                    new DecPOMDPDiscrete("","",dpomdpFile);
                    if(args.sparse)
                        decpomdp->SetSparse(true);
                    MADPParser parser(decpomdp);
                    dp = decpomdp;
                }
            }
            if(args.verbose > 0)
                cout << "ArgumentUtils: Problem file " << dpomdpFile << " parsed" << endl;
        }
    }
    if(args.discount > 0)
        dp->SetDiscount(args.discount);
            
    if(args.verbose > 0)
        cout << "ArgumentUtils: Problem " << dp->GetUnixName() 
             << " instantiated." << endl;

    return dp;
}

FactoredDecPOMDPDiscreteInterface* GetFactoredDecPOMDPDiscreteInterfaceFromArgs(const Arguments& args)
{
    FactoredDecPOMDPDiscreteInterface * dp = 0;
    switch(args.problem_type) 
    {

        case(ProblemType::DT):
        case(ProblemType::FF):
        {
            stringstream ss;
            ss << SoftPrint(args.problem_type) << " is not factored."<< endl;
            throw(E(ss));
            break;
        }
        case(ProblemType::FFF):
        {
            dp = new ProblemFireFightingFactored(args.nrAgents, args.nrHouses, args.nrFLs);
            if(args.cache_flat_models)
                dp->CacheFlatModels(args.sparse);
            if(args.verbose > 0)
                cout << "ArgumentUtils: Problem instantiated." << endl;
            break;
        }
        case(ProblemType::FFG):
        {
            dp = new ProblemFireFightingGraph(args.nrAgents, args.nrFLs);
            if(args.cache_flat_models)
                dp->CacheFlatModels(args.sparse);
            if(args.verbose > 0)
                cout << "ArgumentUtils: Problem instantiated." << endl;
            break;
        }
        case(ProblemType::Aloha):
        {
            dp = new ProblemAloha(
                static_cast<ProblemAloha::IslandConfiguration>(args.islandConf),
                static_cast<ProblemAloha::AlohaVariation>(args.alohaVariation),
                args.maxBacklog,
                args.nrAgents );
            if(args.cache_flat_models)
                dp->CacheFlatModels(args.sparse);
            break;
        }
        case(ProblemType::PARSE):
        default: 
        {   
            string dpomdpFile=directories::MADPGetProblemFilename(args);
            FactoredDecPOMDPDiscrete* decpomdp = 
                new FactoredDecPOMDPDiscrete("", "", dpomdpFile);
            
            MADPParser parser(decpomdp);

            if(args.marginalize)
                decpomdp->MarginalizeStateFactor(args.marginalizationIndex,args.sparse);
            else if(args.cache_flat_models)
                decpomdp->CacheFlatModels(args.sparse);
                
            dp = decpomdp;
            break;
        }
        
    }
    if(args.discount > 0)
        dp->SetDiscount(args.discount);

    return dp;
}


QFunctionJAOHInterface* GetQheuristicFromArgs(
    const PlanningUnitDecPOMDPDiscrete* pu,
    const ArgumentHandlers::Arguments &args)
{
    QFunctionJAOHInterface* q = 0;
#if 0
    QAVParameters qavParams;
    bool useAV=false;
#endif
    switch(args.qheur)
    {
    case eQMDP:
        q = new QMDP(pu,false);
        break;
    case eQPOMDP:
        q = new QPOMDP(pu);
        break;
    case eQBG:
        q = new QBG(pu);
        break;
    case eQMDPc:
        q = new QMDP(pu,true);
        break;
    case eQPOMDPav:
    {
        //true means 'use incremental pruning'
        MonahanPOMDPPlanner *M=new MonahanPOMDPPlanner(pu,true);
        q = new QAV<MonahanPOMDPPlanner>(pu,M);

        break;
    }
    case eQBGav:
    {
        //true means 'use incremental pruning'
        MonahanBGPlanner *M=new MonahanBGPlanner(pu,true);
        q = new QAV<MonahanBGPlanner>(pu,M);

        break;
    }
    case eQHybrid:
        q = GetHybridQheuristicFromArgs(pu,args);
        break;
    case eQPOMDPhybrid:
    {
        ArgumentHandlers::Arguments args1=args;
        args1.QHybridFirstTS=eQPOMDP;
        args1.QHybridLastTS=eQPOMDP;
#if 0 // don't override this any more
        args1.QHybridHorizonLastTimeSteps=0;
#endif
        q = GetHybridQheuristicFromArgs(pu,args1);
        break;
    }
    case eQBGhybrid:
    {
        ArgumentHandlers::Arguments args1=args;
        args1.QHybridFirstTS=eQBG;
        args1.QHybridLastTS=eQBG;
#if 0 // don't override this any more
        args1.QHybridHorizonLastTimeSteps=0;
#endif
        q=GetHybridQheuristicFromArgs(pu,args1);
        break;
    }
    case eQBGTreeIncPrune:
    {
        TreeIncPruneBGPlanner *M=new TreeIncPruneBGPlanner(pu);
        if(args.TreeIPpruneAfterUnion)
            M->SetPruneAfterUnion(true);
        else
            M->SetPruneAfterUnion(false);
        if(args.TreeIPpruneAfterCrossSum)
            M->SetPruneAfterCrossSum(true);
        else
            M->SetPruneAfterCrossSum(false);
        if(args.TreeIPuseVectorCache)
            M->SetUseVectorCache(true);
        else
            M->SetUseVectorCache(false);
            
        q = new QAV<TreeIncPruneBGPlanner>(pu,M);

        break;
    }
    case eQBGTreeIncPruneBnB:
    {
        QBGPlanner_TreeIncPruneBnB *M=new QBGPlanner_TreeIncPruneBnB(pu);
        q = new QAV<QBGPlanner_TreeIncPruneBnB>(pu,M);
        break;
    }
    }
    if(q==0)
        throw(E("argumentUtils::GetQheuristicFromArgs() no Q heuristic instantiated"));

    if(args.verbose>0)
        cout << "Instantiated " << q->SoftPrintBrief() << " heuristic." << endl;
    
    return(q);
}

QFunctionJAOHInterface* GetHybridQheuristicFromArgs(
    const PlanningUnitDecPOMDPDiscrete* pu,
    const ArgumentHandlers::Arguments &args)
{
    QFunctionJointBeliefInterface *qLastTimeSteps=0;
    size_t QHybridHorizonLastTimeSteps=0;
    if(args.QHybridHorizonLastTimeSteps==0)
    {
        // the user did not specify what horizon the vector-based
        // heuristic should have, so we compute a minimally-sized
        // representation

        if(args.QHybridLastTS!=eQPOMDP && args.QHybridLastTS!=eQBG)
            throw(E("argumentUtils::GetHybridQheuristicFromArgs QHybridLastTS should be QPOMDP or QBG when determining horLastT automatically"));
        
        if(!args.useQcache && !args.requireQcache)
        {
            // first we figure out what the equivalent amount of
            // alpha-vectors is wrt to the tree-based representation
            vector<size_t> maxNrAlphas(pu->GetHorizon(),0);
            size_t nr=0;
            for(Index t=0; t<pu->GetHorizon() ;++t)
            {
                size_t nrT=1; //the number of histories at 
                for (Index aI=0 ; aI<pu->GetNrAgents(); ++aI)
                {
                    size_t historiesThisAgent =  
                        pu->GetNrObservationHistories(aI,t) *  
                        pu->GetNrActionHistories(aI,t);
                    nrT *= historiesThisAgent;
                }
                nr += nrT;
                
                double nrHist=nr;
                double nrParametersTree=nrHist*pu->GetNrJointActions();
                double sizeOfAlphaVector=pu->GetNrStates() + 1;
                double nrAlpha=nrParametersTree/sizeOfAlphaVector;
                maxNrAlphas.at(t)=static_cast<size_t>(nrAlpha);
                // make sure the max is not set to 0, because that is
                // interpreted as "no maximum"
                if(maxNrAlphas.at(t)==0)
                    maxNrAlphas.at(t)=1;
            }
            if(args.verbose>=1)
                cout << "MaxNrAlphas per ts: " << SoftPrintVector(maxNrAlphas) << endl;
            
            // Next, given the maximum number of vectors per stage, we try to compute 
            vector<MonahanPlanner*> planners(pu->GetHorizon(),0);
            MonahanPlanner* M=0;
            size_t QHybridHorizonLastTimeStepsOk=0;
            for(QHybridHorizonLastTimeSteps=1;
                QHybridHorizonLastTimeSteps!=pu->GetHorizon();
                ++QHybridHorizonLastTimeSteps)
            {
                vector<size_t> maxNrAlphasForT(QHybridHorizonLastTimeSteps,0);
                for(Index ts=0;ts!=QHybridHorizonLastTimeSteps;++ts)
                    maxNrAlphasForT.at(ts)=maxNrAlphas.at(ts+(pu->GetHorizon()-
                                                              QHybridHorizonLastTimeSteps));
                if(args.verbose>=1)
                    cout << "Computing horLastT " << QHybridHorizonLastTimeSteps
                         << " maxNrAlphas " << SoftPrintVector(maxNrAlphasForT)
                         << endl;

                NullPlanner *np=new NullPlanner(QHybridHorizonLastTimeSteps,
                                                pu->GetDPOMDPD());

                if(args.QHybridLastTS==eQPOMDP)
                    M=new MonahanPOMDPPlanner(np,true);
                else
                    M=new MonahanBGPlanner(np,true);

                planners.at(QHybridHorizonLastTimeSteps)=M;

                M->SetMaxNrAlphas(maxNrAlphasForT);

                try {
                    // make a new planner because the destructor of QAV will
                    // delete the planner as well
#if 0 // don't try to load cached value functions anymore, can mess up
      // timing results
                    string cacheFilename;
                    if(args.QHybridLastTS==eQPOMDP)
                    {
                        QAV<MonahanPOMDPPlanner> qav(np);
                        cacheFilename=qav.GetCacheFilename();
                    }
                    else
                    {
                        QAV<MonahanBGPlanner> qav(np);
                        cacheFilename=qav.GetCacheFilename();
                    }   
                    ifstream fp(string(cacheFilename + "_t0").c_str());
                    bool cached;
                    if(!fp)
                        cached=false;
                    else
                        cached=true;

                    if(cached)
                    {
                        if(args.verbose>=0)
                            cout << "Loading " << cacheFilename << endl;
                        M->Load(cacheFilename);
                    }
                    else
                    {
#endif
                        if(args.verbose>=0)
                            cout << "Computing " << M->SoftPrintBrief() << " with horizon "
                                 << QHybridHorizonLastTimeSteps << endl;
                        M->Plan();
#if 0
                    }
#endif
                }
                catch(E& e){
                    e.Print();
                    // if we get an exception we break the loop
                    break;
                }
                // we didn't get an exception, so this horizon is still ok
                QHybridHorizonLastTimeStepsOk=QHybridHorizonLastTimeSteps;
            }
            QHybridHorizonLastTimeSteps=QHybridHorizonLastTimeStepsOk;
            if(args.verbose>=1)
                cout << "Minimally-sized hybrid representation uses "
                     << pu->GetHorizon()-QHybridHorizonLastTimeSteps
                     << " ts as a tree, and "
                     << QHybridHorizonLastTimeSteps
                     << " as "
                     << planners.at(QHybridHorizonLastTimeSteps)->GetNrVectors()
                     << " vectors."
                     << endl;
            
            // it might happen that we just use a tree representation
            if(QHybridHorizonLastTimeSteps>0)
            {
                NullPlanner *np=new NullPlanner(QHybridHorizonLastTimeSteps,pu->GetDPOMDPD());
                if(args.QHybridLastTS==eQPOMDP)
                    qLastTimeSteps = new QAV<MonahanPOMDPPlanner>(
                        np,dynamic_cast<MonahanPOMDPPlanner*>(planners.at(QHybridHorizonLastTimeSteps)));
                else
                    qLastTimeSteps = new QAV<MonahanBGPlanner>(
                        np,dynamic_cast<MonahanBGPlanner*>(planners.at(QHybridHorizonLastTimeSteps)));
                // cleanup planners that we don't use
                for(Index k=0;k!=planners.size();++k)
                    if(k!=QHybridHorizonLastTimeSteps)
                        delete planners.at(k);
            }
        }
        else // we try to load the heuristic from disk
        {
            NullPlanner *np=new NullPlanner(0,pu->GetDPOMDPD());
            if(args.QHybridLastTS==eQPOMDP)
                qLastTimeSteps = new QAV<MonahanPOMDPPlanner>(np,
                                                              new MonahanPOMDPPlanner(np,true));
            else
                qLastTimeSteps = new QAV<MonahanBGPlanner>(np,
                                                           new MonahanBGPlanner(np,true));

            QHybrid *qh=new QHybrid(pu,args.QHybridFirstTS,
                                    qLastTimeSteps,
                                    0);
            stringstream ss;
            ss << qh->GetCacheFilename() << "_horLast";
            ifstream fp(ss.str().c_str());
            if(!fp)
            {
                stringstream ss1;
                ss1 << "QHybrid::Load: failed to "
                    << "open file " << ss.str();
                throw(E(ss1.str()));
            }
            string buffer;
            getline(fp,buffer);
            istringstream is(buffer);
            is >> QHybridHorizonLastTimeSteps;
            
            // this also deletes M and qLastTimeSteps
            delete qh;
            delete np;

            np=new NullPlanner(QHybridHorizonLastTimeSteps,pu->GetDPOMDPD());
            if(args.QHybridLastTS==eQPOMDP)
                qLastTimeSteps = new QAV<MonahanPOMDPPlanner>(np,new MonahanPOMDPPlanner(np,true));
            else
                qLastTimeSteps = new QAV<MonahanBGPlanner>(np,new MonahanBGPlanner(np,true));
        }
    } 
    else // we use a fixed (non-optimized) horizon for the last time steps
    {
        QHybridHorizonLastTimeSteps=args.QHybridHorizonLastTimeSteps;
        NullPlanner *np=new NullPlanner(args.QHybridHorizonLastTimeSteps,
                                        pu->GetDPOMDPD());
        switch(args.QHybridLastTS)
        {
        case eQMDP:
            qLastTimeSteps=new QMDP(np,false);
            break;
        case eQPOMDP:
        {
            MonahanPOMDPPlanner *M=new MonahanPOMDPPlanner(np,true);
            QAV<MonahanPOMDPPlanner> *qav=new QAV<MonahanPOMDPPlanner>(np,M);
            if(args.useQcache)
                M->PlanWithCache(qav->GetCacheFilename());
            else
                M->Plan();
//            qav->GetPlanner()->Load(qav->GetCacheFilename());
            qLastTimeSteps = qav;
        }
        break;
        case eQBG:
        {
            MonahanBGPlanner *M=new MonahanBGPlanner(np,true);
            QAV<MonahanBGPlanner> *qav=new QAV<MonahanBGPlanner>(np,M);
            if(args.useQcache)
                M->PlanWithCache(qav->GetCacheFilename());
            else
                M->Plan();
//            qav->GetPlanner()->Load(qav->GetCacheFilename());
            qLastTimeSteps = qav;
        }
        break;
        default:
            throw(E("argumentUtils::GetQheuristicFromArgs QHybrid last time step QHeur not valid"));
            break;
        }
    }
    QHybrid *qh=new QHybrid(pu,args.QHybridFirstTS,
                            qLastTimeSteps,
                            QHybridHorizonLastTimeSteps);
    if(args.QHybridHorizonLastTimeSteps==0)
        qh->SetOptimizedHorLast(true);
    else
        qh->SetOptimizedHorLast(false);
    
    return(qh);
}

} //namespace
