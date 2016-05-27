/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek
 * Matthijs Spaan
 * Julian Kooij 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <time.h>
#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include "MADPParser.h"
#include "DICEPSPlanner.h"
#include "directories.h"
#include "DecPOMDPDiscrete.h"
#include "ProblemFireFighting.h"
#include "Timing.h"

#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;



const char *argp_program_version = "DICEPSPlanner";

// Program documentation
static char doc[] =
"DICEPSPlanner - runs the DICEPSPlanner \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 6;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::CE_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    cout << "DICE: direct CE Policy Search"<<endl;
    cout << "-----------------------------"<<endl;
    // parse the command line arguments
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    int restarts = args.nrCERestarts;

    srand(time(0));


    int horizon = args.horizon;
    cout << "Horizon = " << horizon << endl;


    try {
    //start timers
    Timing Time;    
    Time.Start("Overall");

    DecPOMDPDiscreteInterface & decpomdp = * GetDecPOMDPDiscreteInterfaceFromArgs(args);

    // setup the output file stream 
    string filename="/dev/null",timingsFilename="/dev/null";
    ofstream of;
    stringstream ss;
    ss << directories::MADPGetResultsFilename("DICEPS",decpomdp,args)
        << "h" << horizon;
    // add the CE parameters into the output file name
    ss  << "_CEr" << args.nrCERestarts 
        << "_i" << args.nrCEIterations
        << "_s" << args.nrCESamples 
        << "_sfu" << args.nrCESamplesForUpdate
        << "_a" << args.CE_alpha 
        << "_ht" << args.CE_use_hard_threshold
        << "_evals" << args.nrCEEvalutionRuns;
    if(!args.dryrun)
    {
        directories::MADPCreateResultsDir("DICEPS",decpomdp);
        filename=ss.str();
        timingsFilename=filename + "_Timings";
    }
    of.open(filename.c_str());
    if(!of)
    {
        cerr << "could not open " << filename << endl;
        return(1);
    }
    cout << "Computing " << ss.str() << endl;
    //write headers
    of << "#horiz."<<"\t";
    of << "value     " <<"\t";
    of << "wctime"<< "\t";
    of << "utime " <<"\t";
    of << "stime " <<"\t";
    of << "found jpol index\t(1tick=1/"<<sysconf(_SC_CLK_TCK)<<"s)\n";
    of.flush();
    
    //Initialization of the planner with typical options for JESP:
    Time.Start("PlanningUnit");
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointActionObservationHistories(false);
    params.SetComputeJointActionHistories(false);
    params.SetComputeIndividualActionObservationHistories(false);
    params.SetComputeIndividualActionHistories(false);
    //params.SetComputeIndividualObservationHistories(false);
    // joint observations histories are needed for
    // efficient computation of joint actions
    params.SetComputeJointObservationHistories(true);
    params.SetComputeJointBeliefs(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);
    DICEPSPlanner* planner;
    planner = new DICEPSPlanner (params, &decpomdp,
        horizon,
        //CE params
        args.nrCERestarts,
        args.nrCEIterations,
        args.nrCESamples,
        args.nrCESamplesForUpdate, 
        args.CE_use_hard_threshold, //(gamma in CE papers)
        args.CE_alpha, //the learning rate
        args.nrCEEvalutionRuns //the number of evaluation runs
        , args.verbose
    );
    Time.Stop("PlanningUnit");
    cout << "DICEPSPlanner initialized" << endl;

    clock_t total_utime_diceps=0;
    double total_value=0;
    for(int restartI = 0; restartI < restarts; restartI++)
    {
        //start all timers:
        tms ts_before, ts_after;
        clock_t ticks_before, ticks_after;
        Time.Start("Plan");
        ticks_before = times(&ts_before);
        planner->Plan();
        //stop all timers
        ticks_after = times(&ts_after);
        Time.Stop("Plan");
        clock_t ticks =  ticks_after - ticks_before;
        clock_t utime =   ts_after.tms_utime - ts_before.tms_utime;
        clock_t stime =   ts_after.tms_stime - ts_before.tms_stime;

        total_utime_diceps+=utime;

        double V = planner->GetExpectedReward();
        if(args.verbose >= 0)
        {
            cout << "value="<< V << endl;
            if(args.verbose)        {
            planner->GetJointPolicyPureVector()->Print();
            cout <<  endl;
            }
        }
        total_value+=V;
        
        of << horizon<<"\t";
        char formvalue[10];
        sprintf(formvalue, "%.6f", V);
        of << formvalue <<"\t";
        of << ticks <<"\t";
        of << utime <<"\t";
        of << stime <<"\t";
        of << "-1\n";//Cannot get index of joint pol., since  "planner->GetJointPolicyPureVector()->GetIndex()" does not work
        of.flush();

        // output average statistics after completing the last restart
        if(restartI==(restarts-1))
            of << "# h " << args.horizon<<"\t"
               << " avg DICEPS time (s): "
               << (static_cast<double>(total_utime_diceps)/
                   sysconf(_SC_CLK_TCK))/restarts
               << " avg value: " << total_value/restarts
               << endl;
    }
    /* clean up */

    Time.Stop("Overall");

    if(args.verbose >= 0)
    {
        Time.PrintSummary();
        planner->PrintTimersSummary();
    }
#if 0
    if(//args.saveTimings && 
            !args.dryrun)
    {
        Time.Save(timingsFilename);
        planner->SaveTimers(timingsFilename);
    }
#endif
    delete planner;
    }
    catch(E& e){ e.Print(); }
}
