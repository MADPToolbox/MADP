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

#include "parser/MADPParser.h"
#include "GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete.h"
#include "GMAA_MAA_ELSI.h"
#include "FactoredQLastTimeStepOrQBG.h"
#include "FactoredQLastTimeStepOrQPOMDP.h"
#include "FactoredQLastTimeStepOrQMDP.h"
#include "Timing.h"
#include "directories.h"
#include "qheur.h"
#include "gmaatype.h"
#include "FactoredDecPOMDPDiscrete.h"
#include "argumentUtils.h"
#include "JointPolicyPureVector.h"

using namespace qheur;
using namespace GMAAtype;
using namespace std;
using namespace ArgumentUtils;

FactoredQFunctionStateJAOHInterface* getQheuristic(
        GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete *gmaa,
        Qheur_t Qheur)
{
    FactoredQFunctionStateJAOHInterface* q = 0;
    switch(Qheur)
    {
    case(eQMDP):
        q = new FactoredQLastTimeStepOrQMDP(gmaa);
        break;
    case(eQPOMDP):
        q = new FactoredQLastTimeStepOrQPOMDP(gmaa);
        break;
    case(eQBG):
        q = new FactoredQLastTimeStepOrQBG(gmaa);
        break;
    case(eQMDPc):
        q = new FactoredQLastTimeStepOrQMDP(gmaa);
        break;
    default:
        throw(E("non-supported heuristic"));
    }

    return(q);
}


#include "argumentHandlers.h"

const char *argp_program_version = "GMAA_ELSI";

// Program documentation
static char doc[] =
"GMAA_ELSI - runs Generalized MAAStar planners for factored Dec-POMDPs\
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 7;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::gmaa_child,
    ArgumentHandlers::qheur_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    Qheur_t Qheur=args.qheur;
    int horizon=args.horizon;
    int k=args.k;
    int restarts=args.nrRestarts;
   
    try {

    Timing Time;    
    Time.Start("Overall");

    srand(time(0));

    FactoredDecPOMDPDiscreteInterface* decpomdp = 
        GetFactoredDecPOMDPDiscreteInterfaceFromArgs(args);
    decpomdp->CacheFlatModels(args.sparse);
    
//    we have no sparse trans., observ. models for FactoredDecPOMDPDiscrete
//    (they are specified by a 2DNB which should be sparse enough)
//     if(args.sparse) 
//         decpomdp->SetSparse(true);


    GeneralizedMAAStarPlannerForFactoredDecPOMDPDiscrete *gmaa=0;
    FactoredQFunctionStateJAOHInterface *q;

    string filename="",timingsFilename="";
    ofstream of;
    if(!args.dryrun)
    {
        stringstream ss;
        ss << directories::MADPGetResultsFilename("GMAA_ELSI",
                *decpomdp,args)
           << "_" << SoftPrint(Qheur) << "_h" << horizon << "_restarts"
           << restarts << "_k" << k;
        filename=ss.str();
        of.open(filename.c_str());
        if(!of)
        {
            cerr << "GMAA: could not open " << filename << endl;
            return(1);
        }
        timingsFilename=filename + "Timings";

        cout << "Computing " << ss.str() << endl;
    }
    //tms timeStruct, 
    tms ts_before, ts_Qcomp, ts_after;
    clock_t ticks_before, ticks_Qcomp, ticks_after;
    
    ticks_before = times(&ts_before);

    PlanningUnitMADPDiscreteParameters params;
//    params.SetComputeAll(true);
    if(horizon >= 4)
    {
        params.SetComputeJointBeliefs(false);
        params.SetComputeJointActionObservationHistories(false);
//    params.SetComputeJointBeliefs(false);
//    params.SetUseSparseJointBeliefs(true);
    }
    if(args.sparse)
        params.SetUseSparseJointBeliefs(true);
    else
        params.SetUseSparseJointBeliefs(false);

    double V=-DBL_MAX;
    Time.Start("PlanningUnit");
    
    gmaa=new GMAA_MAA_ELSI(params, horizon, decpomdp);

    if(!args.dryrun)
    {
        if(args.saveTimings)
            gmaa->SetIntermediateTimingFilename(timingsFilename);

        if(args.saveAllBGs)
        {
            string bgFilename=filename + "_BG";
            gmaa->SetSaveAllBGs(bgFilename);
        }
    }
    if(args.verbose)
        gmaa->SetVerbose(true);

    Time.Stop("PlanningUnit");
    cout << "GMAA Planner initialized" << endl;
    
    q=getQheuristic(gmaa,Qheur);
    Time.Start("ComputeQ");
    q->Compute();
    Time.Stop("ComputeQ");
    ticks_Qcomp = times(&ts_Qcomp);
    cout << "Q heuristic computed" << endl;

    //gmaa->SetQHeuristic(q);
    gmaa->SetFactoredQHeuristic(q);
    Time.Start("Plan");
    gmaa->Plan();
    V = gmaa->GetExpectedReward();
    Time.Stop("Plan");

    cout << "\nvalue="<< V << endl;
    if(args.verbose >= 0)
        gmaa->GetJointPolicyDiscretePure()->Print();

    ticks_after = times(&ts_after);
    clock_t ticks =  ticks_after - ticks_before;
    clock_t utime =   ts_after.tms_utime - ts_before.tms_utime;

    clock_t time_maa = ticks_after - ticks_Qcomp;
    clock_t utime_maa = ts_after.tms_utime - ts_Qcomp.tms_utime;
    clock_t time_Qcomp = ticks_Qcomp - ticks_before;
    clock_t utime_Qcomp = ts_Qcomp.tms_utime - ts_before.tms_utime;

    if(!args.dryrun)
    {
        of << horizon<<"\t";
        char s[10];
        sprintf(s, "%.6f", V);
        of << s <<"\t";
        of << ticks <<"\t";
        of << utime <<"\t";
        of << time_maa <<"\t";
        of << utime_maa <<"\t";
        of << time_Qcomp <<"\t";
        of << utime_Qcomp <<"\t";
        of << gmaa->GetNrEvaluatedJPolBGs() <<"\t";
        of << gmaa->GetMaxJPolPoolSize() <<"\t";
        //of << gmaa->GetJointPolicyPureVector()->GetIndex() <<"\t";
        of << gmaa->GetJointPolicyDiscretePure()->SoftPrintBrief() <<"\t";
        of << k <<"\n";
        of.flush();
    }

    Time.Stop("Overall");

    if(args.verbose >= 0)
    {
        cout << endl << "Timing summary"<<endl;
        Time.PrintSummary();
        gmaa->PrintTimersSummary();
    }
    if(args.saveTimings && !args.dryrun)
        gmaa->SaveTimers(timingsFilename);

    }
    catch(E& e){ e.Print(); }
}
