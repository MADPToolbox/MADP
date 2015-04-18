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
#include <fstream>
#include "DecPOMDPDiscrete.h"
#include "MADPParser.h"
#include "TOIFactoredRewardDecPOMDPDiscrete.h"
#include "TOICompactRewardDecPOMDPDiscrete.h"
#include "directories.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "AgentRandom.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "evaluateRandomPolicy";

// Program documentation
static char doc[] =
"evaluateRandomPolicy - Evaluates a random policy  \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 6;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::outputFileOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::simulation_child,
    ArgumentHandlers::modelOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    ArgumentHandlers::Arguments args;
    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

    string dpomdpFile=directories::MADPGetProblemFilename(args);

    try {
    
    cout << "Initializing problem..."<<endl;
        
    TOICompactRewardDecPOMDPDiscrete *toi=0;
    DecPOMDPDiscreteInterface *dpomdp=0;
    PlanningUnitDecPOMDPDiscrete *np=0;
    if(args.isTOI)
    {
        toi=new TOICompactRewardDecPOMDPDiscrete("","",dpomdpFile);
        MADPParser parser(toi);
        dpomdp=toi;
        np=new NullPlanner(args.horizon,toi);
    }
    else
    {
        dpomdp = GetDecPOMDPDiscreteInterfaceFromArgs(args);
        cout << "DecPOMDP initialized" << endl;

        np=new NullPlanner(args.horizon,dpomdp);
        cout << "NullPlanner initialized" << endl;
    }

    ofstream of;
    if(!args.dryrun)
    {
        // Create results dir if it doesn't exist already
        directories::MADPCreateResultsDir("evaluateRandomPolicy",*dpomdp);
        stringstream ss;
        ss  << directories::MADPGetResultsFilename("evaluateRandomPolicy",
                                                   *dpomdp,args)
            << "h" << args.horizon << "_nrRuns" << args.nrRuns;
        string filename=ss.str();
        of.open(filename.c_str());
        if(!of)
        {
            cerr << "evaluateRandomPolicy: could not open "
                 << filename << endl;
            return(1);
        }
        
        cout << "Computing " << filename << endl;
    }

    SimulationResult result;
    if(np)
    {
        SimulationDecPOMDPDiscrete sim(*np,args);
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
    }
    cout << "evaluateRandomPolicy sampled value: "
         << result.GetAvgReward() << endl;


    if(!args.dryrun)
        of << result.GetAvgReward() << endl;


    }
    catch(E& e){ e.Print(); }

    return(0);
}
