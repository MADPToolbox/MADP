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

#include <iostream>
#include <float.h>
#include "MADPParser.h"
#include "TransitionObservationIndependentMADPDiscrete.h"
#include "QFunctionJAOHInterface.h"
#include "NullPlanner.h"
#include "Timing.h"
#include "directories.h"
#include "qheur.h"
#include "argumentHandlers.h"
#include "argumentUtils.h"
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

using namespace qheur;
using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "calculateQheuristic";

// Program documentation
static char doc[] =
"calculateQheuristic - calculates and saves Q heuristics  \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 6;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::qheur_child,
    ArgumentHandlers::modelOptions_child,
    ArgumentHandlers::outputFileOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    DecPOMDPDiscreteInterface* decpomdp;
    try {
        ArgumentHandlers::Arguments args;
        argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

        Timing times;
        times.Start("Parsing");
        //DecPOMDPDiscreteInterface* 
        decpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
        TransitionObservationIndependentMADPDiscrete *toi=0;
        if((toi=dynamic_cast<TransitionObservationIndependentMADPDiscrete*>(decpomdp)) &&
           args.qheur==eQMDP &&
           !args.cache_flat_models /* otherwise
                                    * GetDecPOMDPDiscreteInterfaceFromArgs
                                    * already caches the flat
                                    * models */)
        {
            // we don't need a centralized obs model
            toi->CreateCentralizedSparseTransitionModel();
        }

        times.Stop("Parsing");

        if(!args.dryrun)
            directories::MADPCreateResultsDir("GMAA",*decpomdp);
        
        size_t horizon;
        if(args.infiniteHorizon)
            horizon=MAXHORIZON;
        else
            horizon=args.horizon;

        times.Start("Overall");

        PlanningUnitMADPDiscreteParameters params;
#if 0 // Caching doesn't seem worth the trouble if we're computing
      // just one thing (not to mention the memory savings)
        if(Qheur==eQMDP) // don't need any of this for solving the MDP
            params.SetComputeAll(false);
        else
        {
            params.SetComputeAll(true);
            params.SetUseSparseJointBeliefs(true);
        }
#else
        params.SetComputeAll(false);
        if(args.sparse)
            params.SetUseSparseJointBeliefs(true);
#endif

        times.Start("PlanningUnit");
        NullPlanner np(params,horizon,decpomdp);
        times.Stop("PlanningUnit");

        struct timeval tvStart, tvEnd;
        gettimeofday (&tvStart, NULL);

        QFunctionJAOHInterface* q=0;
        for(int restartI = 0; restartI < args.nrRestarts; restartI++)
        {
            // with hybrid heuristics already some computation is done
            // before Compute(), so start timing now
            times.Start("ComputeQ");
            q = GetQheuristicFromArgs(&np, args);
            q->Compute();
            times.Stop("ComputeQ");

            // we want to keep the last q computed
            if(restartI<(args.nrRestarts-1))
                delete q;
        }

        gettimeofday (&tvEnd, NULL);

        clock_t wallclockTime = 
            static_cast<clock_t>(((tvEnd.tv_sec - tvStart.tv_sec) +
                                  static_cast<double>(tvEnd.tv_usec-tvStart.tv_usec)/1e6) * sysconf(_SC_CLK_TCK));

        cout << "Wallclock: from "
             << tvStart.tv_sec << "." << tvStart.tv_usec
             << " until "
             << tvEnd.tv_sec << "." << tvEnd.tv_usec
             << " which took " << wallclockTime << " clock ticks"
             << endl;
        
        times.AddEvent("WallclockTime", wallclockTime);

        if(!args.dryrun)
        {
            times.Start("Save");
            q->Save();
            times.Stop("Save");
            if(args.verbose >= 0)
                cout << "Q saved to " << q->GetCacheFilename() << endl;
        }
        times.Stop("Overall");

        if(args.verbose >= 0)
            times.PrintSummary();

        if(!args.dryrun)
        {
            stringstream ss;
            ss << directories::MADPGetResultsDir("GMAA",*decpomdp)
               << "/calculateQheuristic" << q->SoftPrintBrief() << "_h"
               << horizon;
            if(decpomdp->GetDiscount()!=1)
                ss << "_g" << decpomdp->GetDiscount();
            ss << "_Timings";
            times.Save(ss.str());
            if(args.verbose >= 0)
                cout << "Timings saved to " << ss.str() << endl;
        }

        if(horizon!=MAXHORIZON)
        {
            double Vjb0=-DBL_MAX;
            for(Index a=0;a!=np.GetNrJointActions();++a)
                Vjb0=max(q->GetQ(Globals::INITIAL_JAOHI,a),Vjb0);
            cout << "Value of jaohI 0 = " << Vjb0 << endl;
        }

        delete q;
    }
    catch(E& e){ e.Print(); }

    cout << "cleanup..." << endl;
    delete decpomdp;
}
