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

#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "MADPParser.h"
#include "DecPOMDPDiscrete.h"
#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "GeneralizedMAAStarPlannerForDecPOMDPDiscrete.h"

#include "GMAA_MAAstar.h"
#include "GMAA_MAAstarClassic.h"
#include "GMAA_MAAstarCluster.h"
#include "GMAA_kGMAA.h"
#include "GMAA_kGMAACluster.h"

#include "QBG.h"
#include "QPOMDP.h"
#include "QMDP.h"
#include "QHybrid.h"

#include "Timing.h"
#include "directories.h"
#include "qheur.h"
#include "gmaatype.h"

#include "BGIP_SolverType.h"
#include "BGIP_SolverCreator_AM.h"
#include "BGIP_SolverCreator_BFS.h" 
#include "BGIP_SolverCreator_CE.h"
#include "BGIP_SolverCreator_MP.h"
#include "BGIP_SolverCreator_BnB.h" 
#include "BGIP_SolverCreator_Random.h" 
#include "BGIP_SolverCreator_BFSNonInc.h" 

#include "argumentHandlers.h"
#include "argumentUtils.h"

#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "AgentRandom.h"

#include "JointPolicyPureVectorForClusteredBG.h"
#include "NullPlanner.h"
#include "MonahanBGPlanner.h"
#include "MonahanPOMDPPlanner.h"
#include "QAV.h"
#include "QAlphaVector.h"
#include "BeliefSetNonStationary.h"
#include "OptimalValueDatabase.h"

using namespace qheur;
using namespace GMAAtype;
using namespace BGIP_SolverType;
using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "GMAA";

// Program documentation
static char doc[] =
"GMAA - runs Generalized MAAStar planners \
\vFor more information please consult the MADP documentation.";

const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::CE_child,
    ArgumentHandlers::MaxPlus_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::bgsolver_child,
    ArgumentHandlers::gmaa_child,
    ArgumentHandlers::gmaa_cluster_child,
    ArgumentHandlers::qheur_child,
    ArgumentHandlers::BnB_child,
    { 0 }
};


#include "argumentHandlersPostChild.h"

void SampleRandomPolicy(DecPOMDPDiscreteInterface* decpomdp,
                        const ArgumentHandlers::Arguments &args);

void InitializeOutput(ArgumentHandlers::Arguments &args,
                      DecPOMDPDiscreteInterface* decpomdp,
                      QFunctionJAOHInterface *q,
                      bool GMAAusesBGIPSolver,
                      string &filename,
                      string &timingsFilename,
                      string &jpolFilename,
                      ofstream &of,
                      ofstream &of_jpol);

GeneralizedMAAStarPlannerForDecPOMDPDiscrete* GetGMAAInstance(
    DecPOMDPDiscreteInterface* decpomdp,
    ArgumentHandlers::Arguments &args,
    const PlanningUnitMADPDiscreteParameters &params,
    const string &filename,
    BGIP_SolverCreatorInterface * bgipsc_p
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * bgsc_p,
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * bgscCluster_p
    );

void GetBGIPSolverCreatorInstances(
    ArgumentHandlers::Arguments &args,
    BGIP_SolverCreatorInterface * & bgipsc_p
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * &bgsc_p,
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * &bgscCluster_p
    );

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    
    bool errorOccurred=false;
try 
{

    if(args.verbose >= 0)
    {
        stringstream description;
        switch(args.gmaa)
        {
        case MAAstar:
            if(args.useBGclustering && args.bgsolver==BnB)
                description << "GMAA-ICE (JAIR 2013)";
            else if(args.useBGclustering)
                description << "GMAA-IC (AAMAS 2009)";
            else
                description << "GMAA-MAAstar";
            break;
        case MAAstarClassic:
            description << "GMAA-MAAstar (JAIR 2008)";
            break;
        case FSPC:
            description << "FSPC (JAIR 2008)";
            break;
        case kGMAA: 
            description << "kGMAA (JAIR 2008)";
            break;
        }
        if(args.gmaa!=MAAstarClassic)
            description << " using a "  << SoftPrint(args.bgsolver)
                        << " solver and a "
                        << SoftPrint(args.qheur) << " heuristic";

        cout << description.str() << endl;
    }

    Timing Time;    
    tms ts_before, ts_after;
    clock_t ticks_before, ticks_after;
    Time.Start("Overall");
    Time.Start("PlanningUnit");
//timer starting - don't do file I/O (if possible)        
    ticks_before = times(&ts_before);

    srand(time(0));

    if(args.verbose >= 1)
        cout << "Instantiating the problem..."<<endl;
    DecPOMDPDiscreteInterface* decpomdp =
        GetDecPOMDPDiscreteInterfaceFromArgs(args);
    if(args.verbose >= 1)
        cout << "...done."<<endl;

    bool GMAAusesBGIPSolver=true;
    // MAAstarClassic uses a built-in BFS solver, so we don't use
    // bgsc_p/bgscCluster_p
    if(args.gmaa==MAAstarClassic)
        GMAAusesBGIPSolver=false;

    bool optimalSolutionMethod=false;
    if(args.gmaa==MAAstar || args.gmaa==MAAstarClassic)
        optimalSolutionMethod=true;

    GeneralizedMAAStarPlannerForDecPOMDPDiscrete * gmaa = 0;
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete * gmaaFirstInstance = 0;
    QFunctionJAOHInterface *q=0;
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * bgsc_p = 0;
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * bgscCluster_p = 0;
    double V=-DBL_MAX;

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointActionObservationHistories(false);
    params.SetComputeJointObservationHistories(false);
    params.SetComputeJointActionHistories(false);
    params.SetComputeJointBeliefs(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    params.SetComputeIndividualActionObservationHistories(false);
    params.SetComputeIndividualActionHistories(false);
    params.SetComputeIndividualObservationHistories(false);


    if(args.verbose >= 1)
        cout << "Instantiating the planning unit..."<<endl;

    Time.Start("PlanningUnit");

    // Instantiates bgsc_p or bgscCluster_p
    BGIP_SolverCreatorInterface * bgipsc_p = 0;
    GetBGIPSolverCreatorInstances(args,bgipsc_p);
    if(bgipsc_p && args.verbose >= 1)    
        cout << "BGIP_SolverCreatorInterface instance: " << bgipsc_p->SoftPrint() << endl;

    // We need to make sure that the first GMAA instance exists until
    // the end of the program, as the Q-heuristic will use
    // functionality from it. It's a circular dependence...
    gmaaFirstInstance=GetGMAAInstance(decpomdp,args,params,"", bgipsc_p);
    gmaa=gmaaFirstInstance;

    ticks_after = times(&ts_after);
    Time.Stop("PlanningUnit");
//timer is stopped - ok to do file I/O
    if(args.verbose >= 0)
        cout << "GMAA Planner initialized" << endl;

    q=GetQheuristicFromArgs(gmaaFirstInstance,args);
    string filename="",timingsFilename="", jpolFilename="";
    ofstream of;
    ofstream of_jpol;
    // args can be updated (dryrun)
    InitializeOutput(args,decpomdp,q,GMAAusesBGIPSolver,
                     filename,timingsFilename,jpolFilename,of,of_jpol);
    if(!args.dryrun && args.saveAllBGs)
        gmaa->SetSaveAllBGs(filename + "_BG");

    clock_t wc_time_init = ticks_after - ticks_before;
    clock_t utime_init = ts_after.tms_utime - ts_before.tms_utime;
    clock_t stime_init = ts_after.tms_stime - ts_before.tms_stime;

    //write to tempCout rather than cout, to avoid the possibility
    //of delays by writing to cout (I'm not sure if this is also written
    //to the NFS drive directly?)
    //stringstream tempCout;

    //here the computation of the Q function starts
    try 
    { 
        if(args.verbose >= 1)
            cout << "Computing the Q heuristic ("<<SoftPrint(args.qheur)<<")..."<<endl;
        Time.Start("ComputeQ");
        //timer starting - don't do file I/O (if possible)        
        ticks_before = times(&ts_before);
        if(args.useQcache || args.requireQcache)
        {
            if(args.requireQcache)
                q->ComputeWithCachedQValues(false);
            else
                q->ComputeWithCachedQValues(true);
        }
        else
            q->Compute();
    }
    catch(std::bad_alloc &e)
    {
        cout << "GMAA ran out of memory while computing the QHeuristic" << endl;
        if(!args.dryrun)
            of << "Computing Qheuristic ran out of memory: " << e.what() << endl;
        exit(1);
    }

    ticks_after = times(&ts_after);
    Time.Stop("ComputeQ");
//timer is stopped - ok to do file I/O
    if(args.verbose >= 0)
        cout << SoftPrint(args.qheur) << " heuristic computed" << endl;
    clock_t wc_time_Qcomp = ticks_after - ticks_before;
    clock_t utime_Qcomp = ts_after.tms_utime - ts_before.tms_utime;
    clock_t stime_Qcomp = ts_after.tms_stime - ts_before.tms_stime;

    bool errorOccurred=false;
    clock_t total_utime_maa=0;
    double total_value=0;
    for(int restartI = 0; restartI < args.nrRestarts; restartI++)
    {
        // no point in running all restarts if an error occurred
        if(errorOccurred) 
            break;

        cout << endl
             << "===================== GMAA run " << restartI+1
             << "/" << args.nrRestarts << " starting" << endl;

        if(restartI>0)
        {
            // we don't delete the *first* GMAA instance, as that one
            // is still used by the QHeuristic
            if(restartI>1)
                delete gmaa;
            gmaa=GetGMAAInstance(decpomdp,args,params,filename, bgipsc_p);
        }

        gmaa->SetQHeuristic(q);

//timer starting - don't do file I/O (if possible)        
        Time.Start("Plan");
        ticks_before = times(&ts_before);

        try {
            gmaa->Plan();
        }
        catch(std::bad_alloc &e)
        {
            cout << "GMAA ran out of memory" << endl;
            if(!args.dryrun)
                of << "GMAA ran out of memory: " << e.what() << endl;
            errorOccurred=true;
            continue; // go to next restart
        }
        catch(EDeadline &e)
        {
            cout << "GMAA exceeded the deadline" << endl;
            if(!args.dryrun)
                of << e.SoftPrint() << endl;
            errorOccurred=true;
            continue; // go to next restart
        }
        catch(E &e)
        {
            cout << "Other exception was thrown: " << e.SoftPrint() << endl;
            if(!args.dryrun)
                of << e.SoftPrint() << endl;
            errorOccurred=true;
            continue; // go to next restart
        }

        V = gmaa->GetExpectedReward();
        ticks_after = times(&ts_after);
        Time.Stop("Plan");
//timer is stopped - ok to do file I/O
        cout << "\nvalue="<< V << endl;
        total_value+=V;

        
    //output the found policy    
        boost::shared_ptr<JointPolicyDiscretePure> found_jpol;
        if(!args.useBGclustering && args.horizon<15)
        {
            // with clustering or with a high horizon we don't want to
            // expand the policy, takes too long
            found_jpol = gmaa->GetJointPolicyDiscretePure()->ToJointPolicyPureVector();
            if(args.verbose >= 1)
                cout << found_jpol->SoftPrint();
            of_jpol <<  found_jpol->SoftPrint();
            for(Index k=0;k!=gmaa->GetNrAgents();++k)
            {
                const PolicyDiscretePure* policyDiscretePure=
                    static_cast<PolicyDiscretePure*>(found_jpol->GetIndividualPolicyDiscrete(k));
                of_jpol << endl << gmaa->PolicyToDotGraph(*policyDiscretePure,k);
            }
        }
        else
            of_jpol << "Not outputting joint policy" << endl;

        // check if the value corresponds to the optimal value in case
        // this is an optimal method and we already computed it before
        OptimalValueDatabase db(gmaa);
        cout<<"OptimalValueDatabase: entry '"<<db.GetEntryName()<<"'"<<endl;
        //cout<<db.SoftPrint()<<endl;
        if(optimalSolutionMethod)
        {
            if(db.IsInDatabase())
            {
                if(!db.IsOptimal(V))
                {
                    stringstream ss;
                    ss << "OptimalValueDatabase: GMAA error, computed value " << V 
                       << " does not match"
                       << " previously computed optimal value "
                       << db.GetOptimalValue();
                    of << ss.str() << endl;
                    cout << ss.str() << endl;
                    errorOccurred=true;
                    continue;
                }
                else
                    cout<<"OptimalValueDatabase: Computed value matches with OptimalValueDatabase"<<endl;
            }
            else
            {
                cout << "OptimalValueDatabase: Optimal value unknown." << endl;
                if (args.testMode) // raise error when testing
                {
                    errorOccurred=true;
                    continue;
                }
                else // otherwise save new value
                    try { db.SetOptimalValue(V);}  
                    catch(E& e){ e.Print(); }
            }
        }
        else // approximate methods
        {
            if(db.IsInDatabase())
            {
                cout << "OptimalValueDatabase: Computed value is " << V / db.GetOptimalValue()
                     << " of optimal (value " << db.GetOptimalValue() << ")"
                     << endl;
                if (V>db.GetOptimalValue())
                {
                    errorOccurred=true;
                    continue;
                }
            }
            else
            {
                cout << "OptimalValueDatabase: Optimal value unknown." << endl;
                if (args.testMode) // raise error when testing
                {
                    errorOccurred=true;
                    continue;
                }
            }
        }

    //simulations...
        if(args.verbose >= 0)
            cout << "Running Simulation to determine control quality..." << endl;
        Time.Start("Simulation");
        double Vsim=42;//<-- 42 makes for easier debugging than -1...
        const int nrSimRuns = 10000;
        SimulationDecPOMDPDiscrete sim(*gmaa,nrSimRuns);
        if (args.verbose >= 9)
            sim.SetVerbose(true);
        SimulationResult result;
        if(args.useBGclustering)
        {
            boost::shared_ptr<JointPolicyDiscretePure> jppv = 
                gmaa->GetJointPolicyDiscretePure();
            boost::shared_ptr<JointPolicyPureVectorForClusteredBG> jp4CBG = 
                boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>
                ( jppv );
            if(jp4CBG == 0)
                throw E("conversion to JPPVectorForClusteredBG failed!");
            result=sim.RunSimulations(jp4CBG);
        }
        else// if(!args.useBGclustering)
            result=sim.RunSimulations( found_jpol );
        Time.Stop("Simulation");
        Vsim = result.GetAvgReward();
        if(args.verbose >= 0)
            cout << "Sampled value = " << Vsim 
                 <<" (computed was " << V << ")" << endl;

        clock_t wc_time_maa =  ticks_after - ticks_before;
        clock_t utime_maa =   ts_after.tms_utime - ts_before.tms_utime;
        clock_t stime_maa =   ts_after.tms_stime - ts_before.tms_stime;

        total_utime_maa+=utime_maa;

        clock_t wc_time_tot = wc_time_init + wc_time_Qcomp + wc_time_maa;
        clock_t utime_tot = utime_init + utime_Qcomp + utime_maa;
        clock_t stime_tot = stime_init + stime_Qcomp + stime_maa;

        if (args.verbose >= 0 && args.useBGclustering && args.gmaa==MAAstar)
        {
            cout << endl << "Cluster statistics:"<<endl;
            static_cast<GMAA_MAAstarCluster*>(gmaa)->PrintClusteringStats();
        }
        
        of_jpol << "\nSampled value =  " << Vsim << " (nrSimRuns="<<
            nrSimRuns<< ")\nComputed value = " << V <<  endl;
        of_jpol.flush();

        if(!args.dryrun)
        {
            if(restartI==0)
            {
                of<< "#hor\tvalue     \tValue simul.\
\twc-tot.\tut-tot.\tst-tot.\
\twc-GMAA\tut-GMAA\tst-GMAA\
\twc-Qcom\tut-Qcom\tst-Qcom\
\twc-init\tut-init\tst-init\
\tnrEvalBGjpols\tmaxPoolSize\tJPindex\tk - times are in ticks (1/"<<
sysconf(_SC_CLK_TCK)<<"s)"<< 
"and are GMAAF times (do not include heuristic computation)"<<endl;
            }

            of << args.horizon<<"\t"
               << fixed << setprecision(6)
               << V <<"\t"
               << Vsim << "\t";
            of << wc_time_tot <<"\t";
            of << utime_tot <<"\t";
            of << stime_tot <<"\t";
            of << wc_time_maa <<"\t";
            of << utime_maa <<"\t";
            of << stime_maa <<"\t";
            of << wc_time_Qcomp <<"\t";
            of << utime_Qcomp <<"\t";
            of << stime_Qcomp <<"\t";
            of << wc_time_init <<"\t";
            of << utime_init <<"\t";
            of << stime_init <<"\t";
            of << gmaa->GetNrEvaluatedJPolBGs() <<"\t";
            of << gmaa->GetMaxJPolPoolSize() <<"\t";
            of << gmaa->GetJointPolicyDiscretePure()->SoftPrintBrief() <<"\t";
            of << args.k <<"\n";
            of.flush();

            // output average statistics after completing the last restart
            if(restartI==(args.nrRestarts-1))
                of << "# h " << args.horizon<<"\t"
                   << " avg GMAA time (s): "
                   << (static_cast<double>(total_utime_maa)/
                       sysconf(_SC_CLK_TCK))/args.nrRestarts
                   << " avg value: " << total_value/args.nrRestarts
                   << endl;
        }
        
        if(!args.dryrun)
        {
            if(args.saveTimings)
                gmaa->SaveTimers(timingsFilename);

            if(dynamic_cast<GMAA_MAAstarCluster*>(gmaa))
                static_cast<GMAA_MAAstarCluster*>(gmaa)->
                    SaveClusterStats(filename + "_clusterStats");
        }

        cout << "===================== GMAA run " << restartI+1
             << "/" << args.nrRestarts << " ended, Dec-POMDP value="
             << V << endl;

    }
    of.close();
    of_jpol.flush();
    of_jpol.close();

    Time.Stop("Overall");

    if(!errorOccurred)
    {
        if(args.verbose >= 0)
        {
            cout << endl << "Summary of timing results:" << endl;
            Time.PrintSummary();
            gmaa->PrintTimersSummary();
        }
#if 1 // not very informative        
        SampleRandomPolicy(decpomdp,args);
#endif
    }

    delete q;
    delete gmaa;
    if(args.nrRestarts>1)
        delete gmaaFirstInstance;
    delete bgipsc_p;
    delete decpomdp;
}
catch(E& e){ 
    e.Print(); 
    exit(-1); 
}
 
 return errorOccurred;
}

void SampleRandomPolicy(DecPOMDPDiscreteInterface* dpomdp,
                        const ArgumentHandlers::Arguments &args)
{
    NullPlanner* np=new NullPlanner(args.horizon,dpomdp);

    SimulationDecPOMDPDiscrete sim(*np,args);
    sim.SetVerbose(false);
    SimulationResult result;

    vector<AgentFullyObservable*> agents;
    for(unsigned int i=0;i!=np->GetNrAgents();++i)
    {
        AgentFullyObservable *agent1;
        AgentRandom agent(np,0);
        agent1=new AgentRandom(agent);
        agent1->SetIndex(i);
        agents.push_back(agent1);
    }
    result=sim.RunSimulations(agents);

    if(args.verbose >= 0)
        cout << "evaluateRandomPolicy sampled value: "
             << result.GetAvgReward() << endl;

    for(unsigned int i=0;i!=np->GetNrAgents();++i)
        delete agents.at(i);
    delete np;
}

void InitializeOutput(ArgumentHandlers::Arguments &args,
                      DecPOMDPDiscreteInterface* decpomdp,
                      QFunctionJAOHInterface *q,
                      bool GMAAusesBGIPSolver,
                      string &filename,
                      string &timingsFilename,
                      string &jpolFilename,
                      ofstream &of,
                      ofstream &of_jpol)
{
    if(!args.dryrun)
    {
        // Create results dir if it doesn't exist already
        try {
        directories::MADPCreateResultsDir("GMAA",*decpomdp);
        } catch(E& e)
        {
            e.Print();
            cout << "Results will not be stored to disk." << endl;
            args.dryrun=true;
        }

        stringstream ss;
        ss  << directories::MADPGetResultsFilename("GMAA",*decpomdp,args)
            << SoftPrint(args.gmaa)
            << "_" << q->SoftPrintBrief() << "_h" << args.horizon
            << "_restarts"<< args.nrRestarts;

        //check the method specific arguments 
        //and add them to file name
        if(args.useBGclustering)
        {
            ss << "_Cluster";
            ss  << BayesianGameWithClusterInfo::SoftPrint(
                        static_cast<BayesianGameWithClusterInfo::BGClusterAlgorithm>(args.BGClusterAlgorithm)
                    );
            if(args.BGClusterAlgorithm != BayesianGameWithClusterInfo::Lossless)
                ss << "_tJB" << args.thresholdJB << "_tPjaoh" << args.thresholdPjaoh;
        }
        switch(args.gmaa)
        {
        case MAAstar:
        case MAAstarClassic:
        case FSPC:
            break;
        case kGMAA: 
            ss << "_k" << args.k;
            break;
        }
        if(args.cache_flat_models)
            ss << "_CacheFM";
        else
            ss << "_NoCache";
        if(GMAAusesBGIPSolver)
        {
            ss << "_" << SoftPrint(args.bgsolver);
            // if not, we don't want to add these parameters to the
            // filename as they will not be used
            switch(args.bgsolver)
            {
            case BFS:
            case BFSNonInc:
                // BFS has no parameters
                break;
            case AM:
                ss << "_AM_restarts"<< args.nrAMRestarts;
                break;
            case CE:
                ss  << "_CEr" << args.nrCERestarts 
                    << "_i" << args.nrCEIterations
                    << "_s" << args.nrCESamples 
                    << "_sfu" << args.nrCESamplesForUpdate
                    << "_a" << args.CE_alpha 
                    << "_ht" << args.CE_use_hard_threshold;
                break;
            case MaxPlus:
                break;
            case BnB:
                ss << "_ka" << args.BnB_keepAll
                   << "_JTO" << SoftPrint(args.BnBJointTypeOrdering)
                   << "_CCI" << args.BnB_consistentCompleteInformationHeur;
                break;
            case CGBG_MaxPlus:
                break;
            case NDP:
                break;    
            case Random:
                ss << "_Random";
                break;
            }
        }

        filename=ss.str();

        if(args.noReCompute)
        {
            struct stat sb;
            bool fileExists=true;

            if (stat(filename.c_str(), &sb) == -1) {
                if(errno==ENOENT)
                    fileExists=false;
                else
                {
                    perror("stat");
                    exit(EXIT_SUCCESS);
                }
            }
            
            if(S_ISREG(sb.st_mode) &&
               sb.st_size==0)
                fileExists=false;
            
            if(fileExists)
            {
                cout << "Results file " << filename
                     << " already exists, not recomputing it." << endl;
                exit(0);
            }
        }

        if(!args.dryrun)
        {
            of.open(filename.c_str());
            if(!of)
            {
                cout << "GMAA: could not open " << filename << endl;
                cout << "Results will not be stored to disk." << endl;
                args.dryrun=true;
            }
            timingsFilename=filename + "_Timings";
            jpolFilename=filename + "_JPol";
            if(!args.dryrun)
            {
                of_jpol.open(jpolFilename.c_str());
                if(!of_jpol)
                    throw("Could not open jpol file");
            }
            if(args.verbose >= 0)
                cout << "Computing " << ss.str() << endl;
        }
    }
}


GeneralizedMAAStarPlannerForDecPOMDPDiscrete* GetGMAAInstance(
    DecPOMDPDiscreteInterface* decpomdp,
    ArgumentHandlers::Arguments &args,
    const PlanningUnitMADPDiscreteParameters &params,
    const string &filename,
    BGIP_SolverCreatorInterface * bgipsc_p
    )
{
    GeneralizedMAAStarPlannerForDecPOMDPDiscrete *gmaa=0;

    int verboseness = args.verbose;
    if(args.useBGclustering)
    {
        switch(args.gmaa)
        {
        case MAAstar:
            if(!bgipsc_p->IsExactSolver())
            {
                cout << "Error: MAAstar requires an exact BG solver, and " <<
                    bgipsc_p->SoftPrintBrief() << " is not" << endl;
                exit(1);
            }
            gmaa=new GMAA_MAAstarCluster(bgipsc_p, args.horizon, decpomdp, &params, verboseness );
            break;
        case FSPC:
            args.k=1; // fall through on purpose
        case kGMAA:
        {
            GMAA_kGMAACluster *gmaaCluster=
                new GMAA_kGMAACluster(bgipsc_p, args.horizon, decpomdp, &params, args.k,
                                      static_cast<BayesianGameWithClusterInfo::BGClusterAlgorithm>(args.BGClusterAlgorithm));
            gmaaCluster->SetTresholdJB(args.thresholdJB);
            gmaaCluster->SetTresholdPjaoh(args.thresholdPjaoh);
            gmaa=gmaaCluster;
            break;
        }
        case MAAstarClassic:
            throw E("MAAstarClassic not implemented for clustered version");
            break;
        default:
            throw E("unrecognized GMAA type?!");
        }
    }
    else // regular, un-clustered BGs
    {
        switch(args.gmaa)
        {
        case MAAstar:
            if(!bgipsc_p->IsExactSolver())
            {
                cout << "Error: MAAstar requires an exact BG solver, and " <<
                    bgipsc_p->SoftPrintBrief() << " is not" << endl;
                exit(1);
            }
            gmaa=new GMAA_MAAstar(bgipsc_p, args.horizon, decpomdp, &params, verboseness );
            break;
        case FSPC:
            args.k=1; // fall through on purpose
        case kGMAA:
            gmaa=new GMAA_kGMAA(bgipsc_p, args.horizon, decpomdp, &params, args.k);
            break;
        case MAAstarClassic:
            gmaa=new GMAA_MAAstarClassic(args.horizon, decpomdp, &params, verboseness );
            break;
        default:
            throw E("unrecognized GMAA type?!");
        }
    }

    if(!args.dryrun)
    {
#if 0 // don't store this stuff every iteration, the files can get
      // very large and will slow down the execution!
        // instead we do it when an exception is thrown (but might not
        // work when running out of memory)
        if(args.saveTimings)
            gmaa->SetIntermediateTimingFilename(timingsFilename);
        if(dynamic_cast<GMAA_MAAstarCluster*>(gmaa))
            static_cast<GMAA_MAAstarCluster*>(gmaa)->
                SetClusterStatsFilename(filename + "_clusterStats");

#endif
        if(args.saveAllBGs)
        {
            string bgFilename=filename + "_BG";
            gmaa->SetSaveAllBGs(bgFilename);
        }
    }
    
    gmaa->SetVerbose(args.verbose);
    if(args.GMAAdeadline)
        gmaa->SetDeadline(args.GMAAdeadline);

    return(gmaa);
}

void GetBGIPSolverCreatorInstances(
    ArgumentHandlers::Arguments &args,
    BGIP_SolverCreatorInterface * & bgipsc_p
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVector> * &bgsc_p,
    //BGIP_SolverCreatorInterface_T<JointPolicyPureVectorForClusteredBG> * &bgscCluster_p
    )
{
    if(args.gmaa!=MAAstarClassic) // Classic uses a built-in solver
    {
        // for MAAstar we need to make sure we keep all the solutions,
        // so set k to INT_MAX
        if(args.gmaa==MAAstar)
            args.k=INT_MAX;
        switch(args.bgsolver)
        {
        case BFS:
            if(args.useBGclustering)
                bgipsc_p = 
                    new BGIP_SolverCreator_BFS<JointPolicyPureVectorForClusteredBG>(
                        args.verbose, args.k);
            else
                bgipsc_p = new BGIP_SolverCreator_BFS<JointPolicyPureVector>(args.verbose, args.k);
            break;
        case BFSNonInc:
            if(args.useBGclustering)
                bgipsc_p = 
                    new BGIP_SolverCreator_BFSNonInc<JointPolicyPureVectorForClusteredBG>(
                        args.verbose, args.k);
            else
                bgipsc_p = new BGIP_SolverCreator_BFSNonInc<JointPolicyPureVector>(
                    args.verbose, args.k);
            break;
        case AM:
            if(args.useBGclustering)
                bgipsc_p = 
                    new BGIP_SolverCreator_AM<JointPolicyPureVectorForClusteredBG>(
                        args.nrAMRestarts, args.verbose, args.k);
            else
                bgipsc_p = new BGIP_SolverCreator_AM<JointPolicyPureVector>(
                    args.nrAMRestarts, args.verbose, args.k);
            break;

        case CE:       
            if(args.useBGclustering)
                throw(E("BGIP_SolverCE does not work yet with clustered policies"));
            else
                bgipsc_p = new BGIP_SolverCreator_CE(
                    args.nrCERestarts,
                    args.nrCEIterations,
                    args.nrCESamples,
                    args.nrCESamplesForUpdate,
                    args.CE_use_hard_threshold,
                    args.CE_alpha);
            break;

        case MaxPlus:
            if(args.useBGclustering)
                bgipsc_p =
                    new BGIP_SolverCreator_MP<JointPolicyPureVectorForClusteredBG> (
                    args.maxplus_maxiter, args.maxplus_updateT, 
                    args.maxplus_verbose, args.maxplus_damping, 
                    args.k,//<- nr solutions to return by BG solver
                    args.maxplus_nrRestarts
                    );
            else
                bgipsc_p = new BGIP_SolverCreator_MP<JointPolicyPureVector> (
                    args.maxplus_maxiter, args.maxplus_updateT, 
                    args.maxplus_verbose, args.maxplus_damping, 
                    args.k,//<- nr solutions to return by BG solver
                    args.maxplus_nrRestarts
                    );
            break;
        case BnB:
            if(args.useBGclustering)
                bgipsc_p =
                    new BGIP_SolverCreator_BnB<JointPolicyPureVectorForClusteredBG>
                    (args.verbose -2 , args.k, args.BnB_keepAll,
                     args.BnBJointTypeOrdering,
                     args.BnB_consistentCompleteInformationHeur);
            else
                bgipsc_p = new BGIP_SolverCreator_BnB<JointPolicyPureVector>
                    (args.verbose -2 , args.k, args.BnB_keepAll,
                     args.BnBJointTypeOrdering,
                     args.BnB_consistentCompleteInformationHeur);
            break;
            
        case CGBG_MaxPlus:
            throw E("CGBG max plus only works with factored Dec-POMDPs (use GMAAF instead)");
            break;

        case Random:
            
            if(args.useBGclustering)
                throw E("Random not implemented for clustered GMAA");
            else
                bgipsc_p = new BGIP_SolverCreator_Random
                    (args.verbose, args.k);
            break;

        default:
            throw E("BGIP_Solver is not handled");
            break;
        }

        if(bgipsc_p == 0 && bgipsc_p == 0)
            throw(E("No BGIP Solver Creator instantiated"));
    }
}

