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

#define CHECK_RESULT 0

#include <time.h>
#include <sys/times.h>
#include <iostream>
#include <fstream>
#include <float.h>
#include "MADPParser.h"
#include "JESPExhaustivePlanner.h"
#include "JESPDynamicProgrammingPlanner.h"
#include "directories.h"
#include "DecPOMDPDiscrete.h"
#include "Timing.h"
#include "ValueFunctionDecPOMDPDiscrete.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"



#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;


const char *argp_program_version = "JESP";

// Program documentation
static char doc[] =
"JESP - runs the JESP planner \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 6;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::JESP_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    int restarts = args.nrRestarts;

    srand(time(0));

    int horizon=args.horizon;

    try {
    //start timers
    Timing Time;    
    Time.Start("Overall");

    DecPOMDPDiscreteInterface & decpomdp = * GetDecPOMDPDiscreteInterfaceFromArgs(args);

    //set the filename etc.
    string filename="/dev/null",timingsFilename="/dev/null";
    ofstream of;
    stringstream ss;
    ss  << directories::MADPGetResultsFilename("JESP",decpomdp,args)
        << SoftPrint(args.jesp) //the jesp type
        << "_h" << horizon
        << "_JESPrestarts"<< restarts;
    //check the method specific arguments and add them to file name
    switch(args.jesp)
    {
    case JESPtype::JESPExhaustive:
        break;
    case JESPtype::JESPDP:
        break;
    }
    if(!args.dryrun)
    {
        filename=ss.str();
        timingsFilename=filename + "_Timings";

        of.open(filename.c_str());
        if(!of)
        {
            cout << "JESP: could not open " << filename << endl;
            cout << "Results will not be stored to disk." << endl;
            args.dryrun=true;
        }

        if(!args.dryrun)
        {
            cout << "Computing " << ss.str() << endl;
            //write headers
            of << "#horiz."<<"\t";
            of << "value     " <<"\t";
            of << "ticks"<< "\t";
            of << "utime" <<"\t";
            of << "found jpol index\t(1tick=1/"<<sysconf(_SC_CLK_TCK)<<"s)\n";
            of.flush();
        }
    }
    
    //Initialization of the planner with typical options for JESP:
    Time.Start("PlanningUnit");
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointActionObservationHistories(false);
    params.SetComputeJointObservationHistories(false);
    params.SetComputeJointBeliefs(false);
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);
    PlanningUnitDecPOMDPDiscrete* jesp = 0;
    if(args.jesp == JESPtype::JESPExhaustive)
    {
        jesp = new JESPExhaustivePlanner (params,horizon,&decpomdp);
        cout << "JESPExhaustivePlanner initialized" << endl;
    }
    else if(args.jesp == JESPtype::JESPDP)
    {
        jesp = new JESPDynamicProgrammingPlanner (params,horizon,&decpomdp);
        cout << "JESPDynamicProgrammingPlanner initialized" << endl;
    }
    Time.Stop("PlanningUnit");
    cout << "JESP Planner initialized" << endl;

    for(int restartI = 0; restartI < restarts; restartI++)
    {
        //start all timers:
        Time.Start("Plan");
        tms ts_before, ts_after;
        clock_t ticks_before, ticks_after;
        ticks_before = times(&ts_before);

        jesp->Plan();
        double V = jesp->GetExpectedReward();
        if(args.verbose >= 0)
        {
            cout << "value="<< V << endl;
            if(args.verbose)        {
            jesp->GetJointPolicyPureVector()->Print();
            cout <<  endl;
            }
        }

        //stop all timers
        ticks_after = times(&ts_after);
        clock_t ticks =  ticks_after - ticks_before;
        clock_t utime =   ts_after.tms_utime - ts_before.tms_utime;
        Time.Stop("Plan");

#if CHECK_RESULT
        ValueFunctionDecPOMDPDiscrete vf(jesp, jesp->GetJointPolicyPureVector());
        double v = vf.CalculateV(true);
        cout << "Validated value (exact/approx):="<<v;
        SimulationDecPOMDPDiscrete sim(*jesp, 1000);
        SimulationResult simres = 
            sim.RunSimulations( jesp->GetJointPolicyPureVector() );
        v = simres.GetAvgReward();
        cout << " / "<<v <<endl;
#endif
        if(!args.dryrun)
        {
            of << horizon<<"\t";
            char formvalue[10];
            sprintf(formvalue, "%.6f", V);
            of << formvalue <<"\t";
            of << ticks <<"\t";
            of << utime <<"\t";
            of << jesp->GetJointPolicyPureVector()->GetIndex() <<"\n";
            of.flush();
        }
    }

    delete jesp;
    
    Time.Stop("Overall");

    if(args.verbose >= 0)
    {
        Time.PrintSummary();
    }
    if(args.saveTimings && !args.dryrun)
        Time.Save(timingsFilename);
    }
    catch(E& e){ e.Print(); }
}
