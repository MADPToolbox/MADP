/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "argumentHandlers.h"
#include <string.h>
#include "BayesianGameWithClusterInfo.h"

using namespace ArgumentHandlers;

namespace ArgumentHandlers {

const char *argp_program_bug_address = 
"https://github.com/MADPToolbox/MADP";

// a group for input arguments
static const int GID_INPUTARG=1;


//Dec-POMDP File options (problemFile)
static const int GID_PROBLEMFILE=GID_INPUTARG;
const char *problemFile_argp_version = ".dpomdp file argument parser 0.1";
static const char *problemFile_args_doc = "PROBLEM";
static const char *problemFile_doc = 
"This is the documentation for the .dpomdp file argument parser\
This parser should be included as a child argp parser in the \
main argp parser of your application.(and this message will\
not be shown)\
\vPROBLEM is either \n\
a) one of the following standard problems: DT, FF, FFF, FFG, Aloha \n\
or \n\
b) or a problem filename (.dpomdp, .toi-dpomdp, .pgmx or .pomdp). \
If it includes an extension, PROBLEM will be attempted to be loaded as is, \
otherwise ~/.madp/problems/PROBLEM.{dpomdp,pgmx,POMDP} will be searched (and .toi-dpomdp if TOI models are requested). \
Some of the standard problems take options.";

static const int OPT_NRAGENTS=1; //#agents
static const int OPT_NRHOUSES=2; //#houses
static const int OPT_NRFLS=3;    //#FLs
static const int OPT_ISLANDCONF=4;    // island configuration
static const int OPT_MAXBACKLOG=5;    // maximum backlog
static const int OPT_VARIATION=6;    // aloha variation
static struct argp_option problemFile_options[] = {
//FFG:
{"agents",      OPT_NRAGENTS,   "NRAGENTS",  0, "FireFighting: the number of agents (2)" },
{"houses",      OPT_NRHOUSES,   "NRHOUSES",  0, "FireFighting: the number of houses (3)" },
{"firelevels",  OPT_NRFLS,      "NRFLS",  0, "FireFighting: the number of firelevels (3)"},
//Aloha:
{"islands",  OPT_ISLANDCONF,      "ISLANDS",  0, "Aloha: the island configuration (TwoIslands), can be TwoIslands, OneIsland, TwoIndependentIslands, ThreeIslandsInLine, ThreeIslandsClustered, SmallBigSmallInLine, FiveIslandsInLine, FourIslandsInLine, FourIslandsInSquare, SixIslandsInLine, SevenIslandsInLine"},
{"variation",  OPT_VARIATION,      "VARIATION",  0, "Aloha: which variation to use (NoNewPacket), can be NoNewPacket, NewPacket, NewPacketSendAll, NewPacketProgressivePenalty"},
{"backlog",  OPT_MAXBACKLOG,      "MAXBL",  0, "Aloha: the maximum backlog per island (2)"},
{ 0 }
};
error_t
problemFile_parse_argument (int key, char *arg, struct argp_state *state)
{
    static bool got_problemFile = 0;
    //if we already got the dec-pomdp file return
    if(got_problemFile)
        return ARGP_ERR_UNKNOWN;
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
        case OPT_NRAGENTS:
            theArgumentsStruc->nrAgents = atoi(arg);
            break;
        case OPT_NRHOUSES:
            theArgumentsStruc->nrHouses = atoi(arg);
            break;
        case OPT_NRFLS:
            theArgumentsStruc->nrFLs = atoi(arg);
            break;
        case OPT_ISLANDCONF:
            if(strlen(arg)==1)
                theArgumentsStruc->islandConf=
                    static_cast<ProblemAloha::IslandConfiguration>(atoi(arg));
            else if(strcmp(arg,"TwoIslands")==0)
                theArgumentsStruc->islandConf=ProblemAloha::TwoIslands;
            else if(strcmp(arg,"OneIsland")==0)
                theArgumentsStruc->islandConf=ProblemAloha::OneIsland;
            else if(strcmp(arg,"TwoIndependentIslands")==0)
                theArgumentsStruc->islandConf=ProblemAloha::TwoIndependentIslands;
            else if(strcmp(arg,"ThreeIslandsInLine")==0)
                theArgumentsStruc->islandConf=ProblemAloha::ThreeIslandsInLine;
            else if(strcmp(arg,"ThreeIslandsClustered")==0)
                theArgumentsStruc->islandConf=ProblemAloha::ThreeIslandsClustered;
            else if(strcmp(arg,"SmallBigSmallInLine")==0)
                theArgumentsStruc->islandConf=ProblemAloha::SmallBigSmallInLine;
            else if(strcmp(arg,"FiveIslandsInLine")==0)
                theArgumentsStruc->islandConf=ProblemAloha::FiveIslandsInLine;
            else if(strcmp(arg,"FourIslandsInLine")==0)
                theArgumentsStruc->islandConf=ProblemAloha::FourIslandsInLine;
            else if(strcmp(arg,"FourIslandsInSquare")==0)
                theArgumentsStruc->islandConf=ProblemAloha::FourIslandsInSquare;
            else if(strcmp(arg,"SixIslandsInLine")==0)
                theArgumentsStruc->islandConf=ProblemAloha::SixIslandsInLine;
            else if(strcmp(arg,"SevenIslandsInLine")==0)
                theArgumentsStruc->islandConf=ProblemAloha::SevenIslandsInLine;
            else
                return ARGP_ERR_UNKNOWN;
            break;
        case OPT_VARIATION:
            if(strlen(arg)==1)
                theArgumentsStruc->alohaVariation=
                    static_cast<ProblemAloha::AlohaVariation>(atoi(arg));
            else if(strcmp(arg,"NoNewPacket")==0)
                theArgumentsStruc->alohaVariation=ProblemAloha::NoNewPacket;
            else if(strcmp(arg,"NewPacket")==0)
                theArgumentsStruc->alohaVariation=ProblemAloha::NewPacket;
            else if(strcmp(arg,"NewPacketSendAll")==0)
                theArgumentsStruc->alohaVariation=ProblemAloha::NewPacketSendAll;
            else if(strcmp(arg,"NewPacketProgressivePenalty")==0)
                theArgumentsStruc->alohaVariation=ProblemAloha::NewPacketProgressivePenalty;
            else
                return ARGP_ERR_UNKNOWN;
            break;
        case OPT_MAXBACKLOG:
            theArgumentsStruc->maxBacklog = atoi(arg);
            break;
        case ARGP_KEY_NO_ARGS:
            argp_usage (state);
            break;
        case ARGP_KEY_ARG:
        {
            theArgumentsStruc->dpf = std::string("").c_str(); 
            if(strcmp(arg, "DT") == 0)
                theArgumentsStruc->problem_type = ProblemType::DT;
            else if(strcmp(arg, "FF") == 0)
                theArgumentsStruc->problem_type = ProblemType::FF;
            else if(strcmp(arg, "FFF") == 0)
                theArgumentsStruc->problem_type = ProblemType::FFF;
            else if(strcmp(arg, "FFG") == 0)
                theArgumentsStruc->problem_type = ProblemType::FFG;
            else if(strcmp(arg, "Aloha") == 0)
                theArgumentsStruc->problem_type = ProblemType::Aloha;
            else
            {
                theArgumentsStruc->dpf = arg; 
                got_problemFile = 1; 
                theArgumentsStruc->problem_type = ProblemType::PARSE;
            }
            break;
        }
        default:
            return ARGP_ERR_UNKNOWN;
     }
    return 0;
}
static struct argp problemFile_argp = { problemFile_options, problemFile_parse_argument, problemFile_args_doc, problemFile_doc };
extern const struct argp_child problemFile_child = {&problemFile_argp, 0, "Problem specification options", GID_PROBLEMFILE};



//global options (globalOptions)
static const int GID_GLOBALOPTIONS=8;
const char *globalOptions_argp_version = "global options parser 0.1";
static const char *globalOptions_args_doc = 0;
static const char *globalOptions_doc = 
"This is the documentation for the global options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)"; 
//\v";
static struct argp_option globalOptions_options[] = {
{"verbose",  'v', 0,       0, "Produce verbose output. Specifying this option multiple times increases verbosity." },
{"quiet",    'q', 0,       0, "Don't produce any output" },
{"silent",   's', 0,       OPTION_ALIAS },
{ 0 }
};
error_t
globalOptions_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
        case 'q': case 's':
            theArgumentsStruc->verbose--;
            break;
        case 'v':
            theArgumentsStruc->verbose++;
            break;
        default:
            return ARGP_ERR_UNKNOWN;
     }
    return 0;
}
static struct argp globalOptions_argp = { globalOptions_options, globalOptions_parse_argument, globalOptions_args_doc, globalOptions_doc };
const struct argp_child globalOptions_child = {&globalOptions_argp, 0, "General options", GID_GLOBALOPTIONS};



static const int GID_OUTPUTFILEOPTIONS=6;
//output file options (outputFileOptions)
const char *outputFileOptions_argp_version = "Output file options parser 0.1";
static const char *outputFileOptions_args_doc = 0;
static const char *outputFileOptions_doc = 
"This is the documentation for the output file options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)"; 
//\v";

/* Keys for options without short-options. */
static const int OPT_PREFIX=1;
static const int OPT_NORECOMPUTE=2;
static struct argp_option outputFileOptions_options[] = {
{"dry-run",  'd', 0,       0, "Do not actually create any output files." },
{"description",  'D', "DESCR",       0, 
"Use DESCR to describe the problem. This will be used \
in the output file's names." },
{"prefix",    OPT_PREFIX, "PREFIX",      0, 
"Use PREFIX as a prefix for the output result files." },
{"dontReCompute",  OPT_NORECOMPUTE, 0,       0, "Check if the output file already exists, if so don't re-compute the result, but exit immediately." },
{ 0 }
};
error_t
outputFileOptions_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
        case 'd': 
            theArgumentsStruc->dryrun = 1;
            break;
        case 'D': 
            theArgumentsStruc->description = arg;
            break;
        case OPT_PREFIX:
            theArgumentsStruc->prefix = arg;
            break;
        case OPT_NORECOMPUTE:
            theArgumentsStruc->noReCompute = 1;
            break;
        default:
            return ARGP_ERR_UNKNOWN;
     }
    return 0;
}
static struct argp outputFileOptions_argp = { outputFileOptions_options, outputFileOptions_parse_argument, outputFileOptions_args_doc, outputFileOptions_doc };
const struct argp_child outputFileOptions_child = {&outputFileOptions_argp, 0, "Output file options", GID_OUTPUTFILEOPTIONS};



//model options modelOptions
static const int GID_MODELOPTIONS=3;
const char *modelOptions_argp_version = "Model options parser 0.1";
static const char *modelOptions_args_doc = 0;
static const char *modelOptions_doc = 
"This is the documentation for the model options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v";

static const int OPT_TOI=1;
static struct argp_option modelOptions_options[] = {
{"cache-flat-models",   'f',0,  0, "Cache flat models. Indicates that flat transition, observation and reward models should be cached for factored models. (recommended when using exact inference techniques on factored models)"},
{"sparse",              's',0,  0, "Use sparse transition and observation models" },
{"toi",         OPT_TOI,    0,  0, "Indicate that PROBLEM is a transition observation independent Dec-POMDP" },
{"discount",  'g', "GAMMA",     0, "Set the problem's discount parameter (overriding its default)" },
{ 0 }
};
error_t
modelOptions_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
        case 'f': 
            theArgumentsStruc->cache_flat_models = true;
            break;
        case 's': 
            theArgumentsStruc->sparse = 1;
            break;
        case OPT_TOI:
            theArgumentsStruc->isTOI=1;
            break;
        case 'g':
            theArgumentsStruc->discount = strtof(arg,0);
            break;
        default:
            return ARGP_ERR_UNKNOWN;
     }
    return 0;
}
static struct argp modelOptions_argp = { modelOptions_options, modelOptions_parse_argument, modelOptions_args_doc, modelOptions_doc };
const struct argp_child modelOptions_child = {&modelOptions_argp, 0, "Model options", GID_MODELOPTIONS};


//Solution method options (solutionMethodOptions)
const char *solutionMethodOptions_argp_version = "Solution method options parser 0.1";
static const char *solutionMethodOptions_args_doc = 0;
static const char *solutionMethodOptions_doc = 
"This is the documentation for the solution method options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)"; 
//\v";

//we define a special solution method group ID, so it is easier to group other
//parsers with this one
static const int GID_SM=2;
static const int OPT_INF=1;
static struct argp_option solutionMethodOptions_options[] = {
{"horizon",'h',"HOR", 0, "Specifies the horizon to be considered" },
{"inf", OPT_INF , 0,       0, "Indicate that horizon is infinite" },
{"restarts", 'r', "RESTARTS", 0, "Set the number of times an algorithm is repeated"},
{ 0 }
};
error_t
solutionMethodOptions_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
        case 'h': 
            theArgumentsStruc->horizon = atoi(arg);
            break;
        case OPT_INF:
            theArgumentsStruc->infiniteHorizon=true;
            theArgumentsStruc->horizon = MAXHORIZON;
            break;
        case 'r':
            theArgumentsStruc->nrRestarts = atoi(arg);
            break;
        default:
            return ARGP_ERR_UNKNOWN;
     }
    return 0;
}
static struct argp solutionMethodOptions_argp = { solutionMethodOptions_options, solutionMethodOptions_parse_argument, solutionMethodOptions_args_doc, solutionMethodOptions_doc };
const struct argp_child solutionMethodOptions_child = {&solutionMethodOptions_argp, 0, "Solution method options", GID_SM };


//joint policy argument parser (jpolIndex)
static const int GID_JPOLINDEX=GID_INPUTARG;
const char *jpolIndex_argp_version = "joint policy index argument parser 0.1";
static const char *jpolIndex_args_doc = "JPOL-INDEX";
static const char *jpolIndex_doc = 
"This is the documentation for the joint policy index argument parser\
This parser should be included as a child argp parser in the \
main argp parser of your application.(and this message will\
not be shown)\
\vJPOL-INDEX is a (long long) integer that is the index of the joint \
policy to be considered.";
static struct argp_option jpolIndex_options[] = {
//{"none",  'v', 0,       0, "no options" },
{ 0 }
};
error_t
jpolIndex_parse_argument (int key, char *arg, struct argp_state *state)
{
    static bool got_jpolIndex = 0;
    //if we already got the dec-pomdp file return
    if(got_jpolIndex)
        return ARGP_ERR_UNKNOWN;
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
        case ARGP_KEY_NO_ARGS:
            argp_usage (state);
            break;
        case ARGP_KEY_ARG:
        {
            char *left;
#if USE_ARBITRARY_PRECISION_INDEX
            theArgumentsStruc->jpolIndex = arg;
#else
            theArgumentsStruc->jpolIndex = strtoull(arg,&left,10);
            if(*left != '\0')
            {
                std::stringstream ss;
                ss << "Could not entirely parse the JPOL-INDEX: arg='"
                   << arg <<"', left='" << left
                   << "', found jpol index='"<< theArgumentsStruc->jpolIndex
                   << "'";
                throw(E(ss));
            }
#endif
            got_jpolIndex = 1;
            break;
        }
        default:
            return ARGP_ERR_UNKNOWN;
     }
    return 0;
}
static struct argp jpolIndex_argp = { jpolIndex_options, jpolIndex_parse_argument, jpolIndex_args_doc, jpolIndex_doc };
const struct argp_child jpolIndex_child = {&jpolIndex_argp, 0, 0, GID_PROBLEMFILE};


//BGSOLVER options (bgsolver)
static const int GID_BGSOLVER=GID_SM;
static const int OPT_DEADLINE=1;
const char *bgsolver_argp_version = "BGSOLVER options parser 0.1";
static const char *bgsolver_args_doc = 0;
static const char *bgsolver_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
BGIP_SOLVERTYPE parameter:\n\
BFS\t-\tBrute force search, incremental (for GMAA-ICE))\n\
BFSNonInc\t-\tBrute force search, non-incremental (for plain MAA*)\n\
AM \t-\tAlternating Maximization (see AM options)\n\
CE \t-\tCross-Entropy (see CE options) \n\
MP \t-\tMax-Plus (see MaxPlus options) \n\
BnB \t-\tBranch-and-Bound (see BnB options)\n\
CGBG_MP\t-\tMax-Plus for CGBGs (only for factored Dec-POMDPs) \n\
NDP\t-\tNon-serial Dynamic Programming (only for factored Dec-POMDPs) \n\
Random\t-\tGives random solutions, for testing purposes";

static struct argp_option bgsolver_options[] = {
{"BGIP_Solver", 'B', "BGIP_SOLVERTYPE", 0, "Select which Bayesian game solver to use"},
{"deadline", OPT_DEADLINE, "TIME", 0, "Deadline for solving BGs in s, only implemented for BruteForceSearch"},
{ 0 }
};
error_t
bgsolver_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'B':
        if(strcmp(arg,"BFS")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::BFS;
        if(strcmp(arg,"BFSNonInc")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::BFSNonInc;
        else if(strcmp(arg,"AM")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::AM;
        else if(strcmp(arg,"CE")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::CE;
        else if(strcmp(arg,"MP")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::MaxPlus;
        else if(strcmp(arg,"BnB")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::BnB;
        else if(strcmp(arg,"CGBG_MP")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::CGBG_MaxPlus;
        else if(strcmp(arg,"NDP")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::NDP;
        else if(strcmp(arg,"Random")==0)
            theArgumentsStruc->bgsolver = BGIP_SolverType::Random;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case OPT_DEADLINE:
        theArgumentsStruc->deadline = strtod(arg, 0);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp bgsolver_argp = { bgsolver_options, bgsolver_parse_argument, bgsolver_args_doc, bgsolver_doc };
const struct argp_child bgsolver_child = {&bgsolver_argp, 0, "BGIP Solver options", GID_PROBLEMFILE};


//GMAA options (gmaa)
static const int GID_GMAA=GID_SM;
const char *gmaa_argp_version = "GMAA options parser 0.1";
static const char *gmaa_args_doc = 0;
static const char *gmaa_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
GMAA parameter:\n\
MAAstar        -> use GMAA_MAAstar\n\
kGMAA          -> use GMAA_kGMAA (uses K)\n\
FSPC           -> use GMAA_FSPC\n\
MAAstarClassic -> use GMAA_MAAstarClassic (uses built-in BFS solver)";

static const int AM_RESTARTS=1;
static const int SLACK=2;
static const int OPT_GMAADEADLINE=3;
static const int OPT_REQUIREQCACHE=4;
static struct argp_option gmaa_options[] = {
{"GMAA",    'G', "GMAA", 0, "Select which GMAA variation to use" },
{"k",   'k', "K", 0, "Set k in k-GMAA" },
{"AM-restarts", AM_RESTARTS, "AM_RESTARTS", 0, "nr restarts for solving BGs withFSPC_AM"},
{"saveAllBGs", 'b', 0, 0, "Save all Bayesian Games to disk"},
{"saveTimings", 't', 0, 0, "Save timing results to disk"},
{"useQcache", 'c', 0, 0, "Use cached Q heuristics (compute Q if not cached)"},
{"requireQcache", OPT_REQUIREQCACHE, 0, 0, "Use cached Q heuristics (fail if not cached)"},
{"slack",   SLACK, "FLOAT", 0, "Sets slack to avoid pruning in case of inadmissible heuristics or approximate past rewards. (default=0.0)"},
{"GMAAdeadline", OPT_GMAADEADLINE, "TIME", 0, "Deadline for completing GMAA, in s"},
{ 0 }
};
error_t
gmaa_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {

    case 'G':
        if(strcmp(arg,"MAAstar")==0)
            theArgumentsStruc->gmaa=MAAstar;
        else if(strcmp(arg,"FSPC")==0)
            theArgumentsStruc->gmaa=FSPC;
        else if(strcmp(arg,"kGMAA")==0)
            theArgumentsStruc->gmaa=kGMAA;
        else if(strcmp(arg,"MAAstarClassic")==0)
            theArgumentsStruc->gmaa=MAAstarClassic;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case 'k':
        theArgumentsStruc->k = atoi(arg);
        break;
    case AM_RESTARTS:
        theArgumentsStruc->nrAMRestarts = atoi(arg);
        break;
    case 'b':
        theArgumentsStruc->saveAllBGs = 1;
        break;
    case 't':
        theArgumentsStruc->saveTimings = 1;
        break;
    case 'c':
        theArgumentsStruc->useQcache = 1;
        break;
    case OPT_REQUIREQCACHE:
        theArgumentsStruc->requireQcache = true;
        break;
    case SLACK:
        theArgumentsStruc->slack = atof(arg);
        break;
    case OPT_GMAADEADLINE:
        theArgumentsStruc->GMAAdeadline = atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp gmaa_argp = { gmaa_options, gmaa_parse_argument,
                                 gmaa_args_doc, gmaa_doc
};
const struct argp_child gmaa_child = {&gmaa_argp, 0, "GMAA options", GID_GMAA};


//GMAA Cluster options (gmaa_cluster)
static const int GID_GMAA_CLUSTER=GID_SM;
const char *gmaa_cluster_argp_version = "GMAA Cluster options parser 0.1";
static const char *gmaa_cluster_args_doc = 0;
static const char *gmaa_cluster_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
CLUSTERALG parameter:\n\
Lossless      -> Lossless clustering (default)\n\
ApproxJB      -> Approximate clustering based on joint belief threshold\n\
ApproxPjaoh   -> Approximate clustering based on Pjaoh threshold\n\
ApproxPjaohJB -> Approximate clustering based on joint belief and Pjaoh threshold\n\
\n";

static struct argp_option gmaa_cluster_options[] = {
{"useBGclustering", 'C', 0, 0, "Use Bayesian Game clustering"},
{"BGClusterAlgorithm", 'A', "CLUSTERALG", 0, "Which clustering algorithm to use (Lossless)"},
{"thresholdJB", 'j', "THRESHOLDJB", 0, "Threshold for considering two joint beliefs equal"},
{"thresholdPjaoh", 'p', "THRESHOLDPJAOH", 0, "Threshold for considering two Pjaoh equal"},
{ 0 }
};
error_t
gmaa_cluster_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'C':
        theArgumentsStruc->useBGclustering = 1;
        break;
    case 'A':
        if(strcmp(arg,"Lossless")==0)
            theArgumentsStruc->BGClusterAlgorithm=BayesianGameWithClusterInfo::Lossless;
        else if(strcmp(arg,"ApproxJB")==0)
            theArgumentsStruc->BGClusterAlgorithm=BayesianGameWithClusterInfo::ApproxJB;
        else if(strcmp(arg,"ApproxPjaoh")==0)
            theArgumentsStruc->BGClusterAlgorithm=BayesianGameWithClusterInfo::ApproxPjaoh;
        else if(strcmp(arg,"ApproxPjaohJB")==0)
            theArgumentsStruc->BGClusterAlgorithm=BayesianGameWithClusterInfo::ApproxPjaohJB;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case 'j':
        theArgumentsStruc->thresholdJB = strtof(arg,0);
        break;
    case 'p':
        theArgumentsStruc->thresholdPjaoh = strtof(arg,0);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}

// this does not work (as easy as this)
////const struct argp_child gmaa_childrenVector[] = {
    //ArgumentHandlers::CE_child,
    //{ 0 }
//};
static struct argp gmaa_cluster_argp = { gmaa_cluster_options, gmaa_cluster_parse_argument,
                                 gmaa_cluster_args_doc, gmaa_cluster_doc

};
const struct argp_child gmaa_cluster_child = {&gmaa_cluster_argp, 0, "GMAA Cluster options", GID_GMAA_CLUSTER};


//Perseus options (perseus)
static const int GID_PERSEUS=GID_SM;
const char *perseus_argp_version = "Perseus options parser 0.1";
static const char *perseus_args_doc = 0;
static const char *perseus_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)";

static struct argp_option perseus_options[] = {
{"savePOMDP",  'P', 0, 0, "Save the POMDP to disk" },
{"saveIntermediateV",  'V', 0, 0, "Save intermediate value functions to disk" },
{"saveTimings", 't', 0, 0, "Save timing results to disk"},
{"minNrIterations",   'i', "ITERS", 0, "Make Perseus run at least ITERS iterations" },
{"initReward",  'I', 0, 0, "Initialize the value function with the immediate reward."},
{"initZero",  'z', 0, 0, "Initialize the value function with 0."},
{ 0 }
};

error_t
perseus_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'n':
        theArgumentsStruc->nrBeliefs = atoi(arg);
        break;
    case 'P':
        theArgumentsStruc->savePOMDP=1;
        break;
    case 'V':
        theArgumentsStruc->saveIntermediateV=1;
        break;
    case 't':
        theArgumentsStruc->saveTimings = 1;
        break;
    case 'i':
        theArgumentsStruc->minimumNrIterations = atoi(arg);
        break;
    case 'I':
        theArgumentsStruc->initializeWithImmediateReward=1;
        break;
    case 'z':
        theArgumentsStruc->initializeWithZero=1;
        break;
    case 'm':
        theArgumentsStruc->marginalize = true;
        theArgumentsStruc->marginalizationIndex = atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp perseus_argp = { perseus_options, perseus_parse_argument,
                                    perseus_args_doc, perseus_doc };
const struct argp_child perseus_child = {&perseus_argp, 0,
                                         "Perseus general options", GID_PERSEUS};

//Perseus belief set sampling options (perseusbelief)
static const int GID_PERSEUSBELIEF=GID_SM;
const char *perseusbelief_argp_version = "Perseus Belief options parser 0.1";
static const char *perseusbelief_args_doc = 0;
static const char *perseusbelief_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)";

static struct argp_option perseusbelief_options[] = {
{"beliefs",'n', "BELIEFS", 0, "Set the belief set size" },
{"saveBeliefs",  'B', 0, 0, "Save beliefs to disk" },
{"beliefSamplingHorizon",   'H', "HORIZON", 0, "Introduce an artificial horizon when sampling the beliefs (useful in infinite-horizon case)" },
{"uniqueBeliefs",  'u', 0, 0, "Sample unique beliefs (no duplicates)" },
{"useQMDP",  'Q', 0, 0, "Follow the QMDP policy while sampling beliefs instead of acting uniformly at random."},
{"QMDPexploreProb",  'x', "PROB", 0, "Probability with which to take a random action when using QMDP for belief sampling."},
{ 0 }
};
error_t
perseusbelief_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'n':
        theArgumentsStruc->nrBeliefs = atoi(arg);
        break;
    case 'B':
        theArgumentsStruc->saveBeliefs=1;
        break;
    case 'H':
        theArgumentsStruc->resetAfter = atoi(arg);
        break;
    case 'u':
        theArgumentsStruc->uniqueBeliefs=1;
        break;
    case 'Q':
        theArgumentsStruc->useQMDPforSamplingBeliefs=1;
        break;
    case 'x':
        theArgumentsStruc->QMDPexploreProb = strtof(arg,0);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp perseusbelief_argp = { perseusbelief_options, perseusbelief_parse_argument,
                                    perseusbelief_args_doc, perseusbelief_doc };
const struct argp_child perseusbelief_child = {&perseusbelief_argp, 0,
                                         "Perseus belief set sampling options", GID_PERSEUSBELIEF};

//Perseus Backup options (perseusbackup)
static const int GID_PERSEUSBACKUP=GID_SM;
const char *perseusbackup_argp_version = "Perseus Backup options parser 0.1";
static const char *perseusbackup_args_doc = 0;
static const char *perseusbackup_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\vBACKUP parameter:\n\
0  or POMDP   -> use PerseusPOMDP (default)\n\
1  or BG      -> use PerseusBG (BG solver can be specified by BGBACKUP)\n\
2  or EVENT_POMDP -> use PerseusConstrainedPOMDP (only applicable to event-driven models)\n\
\n\
BGBACKUP parameter (details in AlphaVectorBG.{h,cpp}):\n\
0 or EXH_MAX       -> use EXHAUSTIVE_ONLYKEEPMAX backup in PerseusBG\n\
1 or EXH_ALL       -> use EXHAUSTIVE_STOREALL backup in PerseusBG\n\
2 or BGS_EXH       -> use BGIP_SOLVER_EXHAUSTIVE backup in PerseusBG (default)\n\
3 or BGS_ALTMAX    -> use BGIP_SOLVER_ALTERNATINGMAXIMIZATION backup in PerseusBG\n\
4 or BGS_ALTMAX100 -> use BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS backup\n\
5 or BGS_BNB       -> use BGIP_SOLVER_BRANCH_AND_BOUND backup\n\
";

static struct argp_option perseusbackup_options[] = {
{"backup",  'b', "BACKUP", 0, "Select which backup to use, see below" },
{"vectorEachBelief",   'e', 0, 0, "If specified, don't sample from belief set, but compute vector for each belief" },
{"BGbackup",  'y', "BGBACKUP", 0, "Select which BG backup to use for PerseusBG" },
{"waitPenalty",   'w', "PENALTY", 0, "Set the wait penalty for PerseusImplicitWaiting" },
{"weight",   'W', "WEIGHT", 0, "Set the weight for PerseusWeighted{NS}" },
{"commModel",   'c', "COMM", 0, "Select the communication model for PerseusWeighted" },
{ 0 }
};
error_t
perseusbackup_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'y':
        if(strlen(arg)==1)
            theArgumentsStruc->bgBackup=static_cast<BGBackupType>(atoi(arg));
        else if(strcmp(arg,"EXH_MAX")==0)
            theArgumentsStruc->bgBackup=EXHAUSTIVE_ONLYKEEPMAX;
        else if(strcmp(arg,"EXH_ALL")==0)
            theArgumentsStruc->bgBackup=EXHAUSTIVE_STOREALL;
        else if(strcmp(arg,"BGS_EXH")==0)
            theArgumentsStruc->bgBackup=BGIP_SOLVER_EXHAUSTIVE;
        else if(strcmp(arg,"BGS_ALTMAX")==0)
            theArgumentsStruc->bgBackup=BGIP_SOLVER_ALTERNATINGMAXIMIZATION;
        else if(strcmp(arg,"BGS_ALTMAX100")==0)
            theArgumentsStruc->bgBackup=BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS;
        else if(strcmp(arg,"BGS_BNB")==0)
            theArgumentsStruc->bgBackup=BGIP_SOLVER_BRANCH_AND_BOUND;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case 'b':
        if(strlen(arg)==1)
            theArgumentsStruc->backup=static_cast<PerseusBackupType>(atoi(arg));
        else if(strcmp(arg,"POMDP")==0)
            theArgumentsStruc->backup=POMDP;
        else if(strcmp(arg,"BG")==0)
            theArgumentsStruc->backup=BG;
        else if(strcmp(arg,"EVENT_POMDP")==0)
            theArgumentsStruc->backup=EVENT_POMDP;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case 'w':
        theArgumentsStruc->waitPenalty = strtof(arg,0);
        break;
    case 'W':
        theArgumentsStruc->weight = strtof(arg,0);
        break;
    case 'c':
        theArgumentsStruc->commModel = atoi(arg);
        break;
    case 'e':
        theArgumentsStruc->computeVectorForEachBelief = 1;
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp perseusbackup_argp = { perseusbackup_options,
                                          perseusbackup_parse_argument,
                                          perseusbackup_args_doc,
                                          perseusbackup_doc };
const struct argp_child perseusbackup_child = {&perseusbackup_argp, 0,
                                               "Perseus backup options",
                                               GID_PERSEUSBACKUP};

//Qheur options (qheur)
static const int GID_QHEUR=GID_SM;
static const int PRUNEAFTERUNION = 1;
static const int PRUNEAFTERCROSSSUM = 2;
static const int TREEIPVECTORCACHE = 3;
const char *qheur_argp_version = "QHEUR options parser 0.1";
static const char *qheur_args_doc = 0;
static const char *qheur_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\vQHEUR parameter:\n\
QMDP              (defined on joint beliefs)\n\
QPOMDP            (defined on joint history tree)\n\
QBG               (defined on joint history tree)\n\
QMDPc             (cached for each joint AO history)\n\
QPOMDPav          (uses alpha vectors over joint beliefs)\n\
QBGav             (uses alpha vectors over joint beliefs)\n\
QHybrid           (hybrid between vector and trees, customizable)\n\
QPOMDPhybrid      (QPOMDP hybrid between vector and trees, no options)\n\
QBGhybrid         (QBG hybrid between vector and trees, no options)\n\
QBGTreeIncPrune   (vector-based QBG using tree-based incremental pruning with memoization)\n\
QBGTreeIncPruneBnB(vector-based QBG using tree-based incremental pruning with branch-and-bound)\n";

static struct argp_option qheur_options[] = {
{"Qheuristic",  'Q', "QHEUR", 0, "Select which Q-heuristic to use" },
{"horLastT",  'h', "H", 0, "For QHybrid, the horizon for the last time steps" },
{"firstTSheur",  'f', "QHEUR", 0, "For QHybrid, which heuristic to use for the first time steps (QBG or QPOMDP)" },
{"lastTSheur",  'l', "QHEUR", 0, "For QHybrid, which heuristic to use for the first time steps (QMDP/QPOMDP/QBG)" },
{"pruneAfterUnion",  PRUNEAFTERUNION, "0/1", 0, "For QBGTreeIncPrune, whether to prune after every union" },
{"pruneAfterCrossSum",  PRUNEAFTERCROSSSUM, "0/1", 0, "For QBGTreeIncPrune, whether to prune after every cross sum" },
{"useVectorCache",  TREEIPVECTORCACHE, "0/1", 0, "For QBGTreeIncPrune, whether to cache VectorSets" },
{ 0 }
};
error_t
qheur_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments* theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'Q':
        if(strcmp(arg,"QMDP")==0)
            theArgumentsStruc->qheur=eQMDP;
        else if(strcmp(arg,"QPOMDP")==0)
            theArgumentsStruc->qheur=eQPOMDP;
        else if(strcmp(arg,"QBG")==0)
            theArgumentsStruc->qheur=eQBG;
        else if(strcmp(arg,"QMDPc")==0)
            theArgumentsStruc->qheur=eQMDPc;
        else if(strcmp(arg,"QPOMDPav")==0)
            theArgumentsStruc->qheur=eQPOMDPav;
        else if(strcmp(arg,"QBGav")==0)
            theArgumentsStruc->qheur=eQBGav;
        else if(strcmp(arg,"QHybrid")==0)
            theArgumentsStruc->qheur=eQHybrid;
        else if(strcmp(arg,"QPOMDPhybrid")==0)
            theArgumentsStruc->qheur=eQPOMDPhybrid;
        else if(strcmp(arg,"QBGhybrid")==0)
            theArgumentsStruc->qheur=eQBGhybrid;
        else if(strcmp(arg,"QBGTreeIncPrune")==0)
            theArgumentsStruc->qheur=eQBGTreeIncPrune;
        else if(strcmp(arg,"QBGTreeIncPruneBnB")==0)
            theArgumentsStruc->qheur=eQBGTreeIncPruneBnB;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case 'h':
        theArgumentsStruc->QHybridHorizonLastTimeSteps = atoi(arg);
        break;
    case 'f':
        if(strcmp(arg,"QPOMDP")==0)
            theArgumentsStruc->QHybridFirstTS=eQPOMDP;
        else if(strcmp(arg,"QBG")==0)
            theArgumentsStruc->QHybridFirstTS=eQBG;
        else
        {
            std::cerr << "QHybrid: the heuristic for the first time steps can only be QBG or QPOMDP" << std::endl;
            return ARGP_ERR_UNKNOWN;
        }
        break;
    case 'l':
        if(strcmp(arg,"QMDP")==0)
            theArgumentsStruc->QHybridLastTS=eQMDP;
        else if(strcmp(arg,"QPOMDP")==0)
            theArgumentsStruc->QHybridLastTS=eQPOMDP;
        else if(strcmp(arg,"QBG")==0)
            theArgumentsStruc->QHybridLastTS=eQBG;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case PRUNEAFTERUNION:
        theArgumentsStruc->TreeIPpruneAfterUnion=atoi(arg);
        break;
    case PRUNEAFTERCROSSSUM:
        theArgumentsStruc->TreeIPpruneAfterCrossSum=atoi(arg);
        break;
    case TREEIPVECTORCACHE:
        theArgumentsStruc->TreeIPuseVectorCache=atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp qheur_argp = { qheur_options, qheur_parse_argument,
                                  qheur_args_doc, qheur_doc };
const struct argp_child qheur_child = {&qheur_argp, 0, "Q-heuristic options", 
                                       GID_QHEUR};


//Simulation options (simulation)
static const int GID_SIMULATION=7;
const char *simulation_argp_version = "Simulation options parser 0.1";
static const char *simulation_args_doc = 0;
static const char *simulation_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)"; 
//\v";
static struct argp_option simulation_options[] = {
{"runs",  'r', "RUNS", 0, "Set the number of episodes to simulate" },
{"seed",  'S', "SEED", 0, "Set the random seed" },
{ 0 }
};
error_t
simulation_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'r':
        theArgumentsStruc->nrRuns=atoi(arg);
        break;
    case 'S':
        theArgumentsStruc->randomSeed=atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp simulation_argp = { simulation_options,
                                       simulation_parse_argument,
                                       simulation_args_doc, simulation_doc };
const struct argp_child simulation_child = {&simulation_argp, 0,
                                            "Simulation options", 
                                            GID_SIMULATION};

//CE options (CE)
static const int GID_CE=GID_SM;
const char *CE_argp_version = "CE options parser 0.1";
static const char *CE_args_doc = 0;
static const char *CE_doc = 
"This is the documentation for the CE (cross entropy) options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
Options for using CE (the cross-entropy method) for optimization include \
the number of restarts (runs), the number of iteration for each runs, \
how much samples are drawn each iteration, and how much of those are \
used to update the probability distribution.";

static const int CE_RESTARTS = 1;
static const int CE_EVALUATION_RUNS = 2;
static struct argp_option CE_options[] = {
{"CE-restarts", CE_RESTARTS, "CERESTARTS", 0, "Set the number of CE restarts (runs)"},
{"CE-eval-runs", CE_EVALUATION_RUNS, "CEEVALRUNS", 0, "Set the number of policy evaluation runs. More runs will result in more accurate evaluation. (set 0 for exact evaluation)."},
{"iterations", 'i', "ITERATIONS", 0, "Set the number of iterations per run"},
{"samples", 'n', "SAMPLES", 0, "Set the number of samples per iteration"},
{"updateSamples", 'u', "UPDATESAMPPLES", 0, "Set the number of samples used to update the prob. distribution."},
{"not_strictly_improving", 'N', 0, 0, "Do not use a hard threshold: do not require that newly sampled policies are strictly better then before. (this corresponds to the gamma in the CE papers)"},
{"alpha", 'a', "ALPHA", 0, "The learning rate"},
{ 0 }
};
error_t
CE_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case CE_EVALUATION_RUNS:
        theArgumentsStruc->nrCEEvalutionRuns = atoi(arg);
        break;
    case CE_RESTARTS:
        theArgumentsStruc->nrCERestarts = atoi(arg);
        break;
    case 'i':
        theArgumentsStruc->nrCEIterations =  atoi(arg);
        break;
    case 'n':
        theArgumentsStruc->nrCESamples =  atoi(arg);
        break;
    case 'u':
        theArgumentsStruc->nrCESamplesForUpdate =  atoi(arg);
        break;
    case 'N':
        theArgumentsStruc->CE_use_hard_threshold = 0;
        break;
    case 'a':
        theArgumentsStruc->CE_alpha =  atof(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp CE_argp = { CE_options, CE_parse_argument,
                                 CE_args_doc, CE_doc };
const struct argp_child CE_child = {&CE_argp, 0, "CE options", GID_CE};


//MaxPlus options (MaxPlus)
static const int GID_MaxPlus=GID_SM;
const char *MaxPlus_argp_version = "MaxPlus options parser 0.1";
static const char *MaxPlus_args_doc = 0;
static const char *MaxPlus_doc = 
"This is the documentation for the MaxPlus (cross entropy) options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
Options for using MaxPlus for optimization include:\n\
-the number of restarts (only useful for randomized Maxplus versions),\n\
-the number maximum number of of iterations of mesage passing in each run,\n\
-the update scheme,\n\
-the damping factor (may speed up convergence, at extra computational cost per \
iteration)\n\
-the verboseness (a positive number), for debugging.";

static const int MAXPLUS_RESTARTS = 1;
static const int MAXPLUS_UPDATE = 2;
static const int MAXPLUS_ITER = 3;
static const int MAXPLUS_VERB = 4;
static const int MAXPLUS_DAMP = 5;
static struct argp_option MaxPlus_options[] = {
{"MP-restarts", MAXPLUS_RESTARTS, "MaxPlusRESTARTS", 0, "Set the number of MaxPlus restarts (runs)"},
{"MP-update", MAXPLUS_UPDATE, "STRING", 0, "The update scheme: \"PARALL\"(default), \"SEQRND\", or \"SEQMAX\""},
{"MP-iters", MAXPLUS_ITER, "NUMBER", 0, "The maximum number of iterations performed by Max-Plus"},
{"MP-verbose", MAXPLUS_VERB, "0...9", 0, "Set verboseness level of Max-Plus (o by default"},
{"MP-damp", MAXPLUS_DAMP, "REAL", 0, "Set the damping factor (default 0.5)"},
{ 0 }
};
error_t
MaxPlus_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case MAXPLUS_RESTARTS:
        theArgumentsStruc->maxplus_nrRestarts =  atoi(arg);
        break;
    case MAXPLUS_UPDATE:
        theArgumentsStruc->maxplus_updateT =  std::string(arg);
        break;
    case MAXPLUS_ITER:
        theArgumentsStruc->maxplus_maxiter =  atoi(arg);
        break;
    case MAXPLUS_VERB:
        theArgumentsStruc->maxplus_verbose =  atoi(arg);
        break;
    case MAXPLUS_DAMP:
        theArgumentsStruc->maxplus_damping =  atof(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp MaxPlus_argp = { MaxPlus_options, MaxPlus_parse_argument,
                                 MaxPlus_args_doc, MaxPlus_doc };
const struct argp_child MaxPlus_child = {&MaxPlus_argp, 0, "MaxPlus options", GID_MaxPlus};


//JESP options (JESP)
static const int GID_JESP=GID_SM;
const char *JESP_argp_version = "JESP options parser 0.1";
static const char *JESP_args_doc = 0;
static const char *JESP_doc = 
"This is the documentation for the JESP (Joint Equilibrium-based \
Search for Policies) options parser.\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
Options for using  JESP (Joint Equilibrium-based \
Search for Policies) include: \
the type of JESP (exhaustive or Dynamic Programming), which can be:\n\
0 or Exh - exhaustive JESP\n\
1 or DP  - Dynamic programming\n\n\
and the number of restarts (runs).";

static const int JESP_TYPE = 2;
static struct argp_option JESP_options[] = {
{"JESP-type", JESP_TYPE, "JESPTYPE", 0, "Set the type of JESP (Exh or [DP])"},
{"saveTimings", 't', 0, 0, "Save timing results to disk"},
{ 0 }
};
error_t
JESP_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case JESP_TYPE:
        if(strlen(arg)==1)
            theArgumentsStruc->jesp=static_cast<JESP_t>(atoi(arg));
        else if(strcmp(arg,"Exh")==0)
            theArgumentsStruc->jesp = JESPExhaustive;
        else if(strcmp(arg,"DP")==0)
            theArgumentsStruc->jesp = JESPDP;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case 't':
        theArgumentsStruc->saveTimings = 1;
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp JESP_argp = { JESP_options, JESP_parse_argument,
                                 JESP_args_doc, JESP_doc };
const struct argp_child JESP_child = {&JESP_argp, 0, "JESP options", GID_JESP};

//onlinePOMDP options (onlinePOMDP)
static const int GID_onlinePOMDP=GID_SM;
const char *onlinePOMDP_argp_version = "onlinePOMDP options parser 0.1";
static const char *onlinePOMDP_args_doc = 0;
static const char *onlinePOMDP_doc = 
"This is the documentation for the onlinePOMDP options parser.\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v";

static struct argp_option onlinePOMDP_options[] = {
{"nrNodesExpanded",  'n', "NODES", 0, "Set the number of nodes to be expanded at every action choice" },
{ 0 }
};
error_t
onlinePOMDP_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'n':
        theArgumentsStruc->nrNodesExpanded = atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp onlinePOMDP_argp = { onlinePOMDP_options,
                                        onlinePOMDP_parse_argument,
                                        onlinePOMDP_args_doc, onlinePOMDP_doc };
const struct argp_child onlinePOMDP_child = {&onlinePOMDP_argp, 0, "onlinePOMDP options", GID_onlinePOMDP};

//RL options (RL)
static const int GID_RL=GID_SM;
static const int RLSTARTRUN = 1;
const char *RL_argp_version = "RL options parser 0.1";
static const char *RL_args_doc = 0;
static const char *RL_doc = 
"This is the documentation for the RL options parser.\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v";

static struct argp_option RL_options[] = {
{"nrRLruns",  'n', "RUNS", 0, "Set the number of learning runs" },
{"nrEvals",  'e', "EVALS", 0, "Set how often intermediate evaluation are run" },
{"startRun", RLSTARTRUN , "START", 0, "Start the RL from a particular run" },
{ 0 }
};
error_t
RL_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'n':
        theArgumentsStruc->nrRLruns = atoi(arg);
        break;
    case 'e':
        theArgumentsStruc->nrIntermediateEvaluations = atoi(arg);
        break;
    case RLSTARTRUN:
        theArgumentsStruc->startAtRLrun = atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp RL_argp = { RL_options,
                               RL_parse_argument,
                               RL_args_doc, RL_doc };
const struct argp_child RL_child = {&RL_argp, 0, "RL options", GID_RL};
//BNB options (BNB)
const int BNB_KEEPALL = 1;
const int BNB_TYPE = 2;
const int BNB_RECOMPUTE = 3;
static const int GID_BNB=GID_SM;
const char *BNB_argp_version = "BNB options parser 0.1";
static const char *BNB_args_doc = 0;
static const char *BNB_doc = 
"This is the documentation for the BNB options parser.\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)\
\v\
JointTypeOrdering can be\n\
Id       - Identity Mapping\n\
MaxC     - Ordered by maximum contribution\n\
MinC     - Ordered by minimum contribution\n\
MaxDiff  - Ordered by maximum difference\n\
Prob     - Ordered by descending probability\n\
Basis    - Use only the basis types\n\
CMaxC    - Ordered by maximum contribution (updated for consistency)\n\
CMinC    - Ordered by minimum contribution (updated for consistency)\n\
CMaxDiff - Ordered by maximum difference (updated for consistency)\n";

static struct argp_option BNB_options[] = {
{"BnB-keepAll", BNB_KEEPALL, 0, 0, "Generate all joint action extensions"},
{"BnB-ordering", BNB_TYPE, "BNBTYPE", 0, "Choose the ordering of joint types"},
{"BnB-CIheur", BNB_RECOMPUTE, 0, 0, "Use the Complete Information heuristic instead of the Consistent CI one"},
{ 0 }
};
error_t
BNB_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case BNB_KEEPALL:
        theArgumentsStruc->BnB_keepAll = true;
        break;
    case BNB_TYPE:
        if(strcmp(arg,"Id")==0)
            theArgumentsStruc->BnBJointTypeOrdering = IdentityMapping;
        else if(strcmp(arg,"MaxC")==0)
            theArgumentsStruc->BnBJointTypeOrdering = MaxContribution;
        else if(strcmp(arg,"MinC")==0)
            theArgumentsStruc->BnBJointTypeOrdering = MinContribution;
        else if(strcmp(arg,"MaxDiff")==0)
            theArgumentsStruc->BnBJointTypeOrdering = MaxContributionDifference;
        else if(strcmp(arg,"Prob")==0)
            theArgumentsStruc->BnBJointTypeOrdering = DescendingProbability;
        else if(strcmp(arg,"Basis")==0)
            theArgumentsStruc->BnBJointTypeOrdering = BasisTypes;
        else if(strcmp(arg,"CMaxC")==0)
            theArgumentsStruc->BnBJointTypeOrdering = ConsistentMaxContribution;
        else if(strcmp(arg,"CMinC")==0)
            theArgumentsStruc->BnBJointTypeOrdering = ConsistentMinContribution;
        else if(strcmp(arg,"CMaxDiff")==0)
            theArgumentsStruc->BnBJointTypeOrdering = ConsistentMaxContributionDifference;
        else
            return ARGP_ERR_UNKNOWN;
        break;
    case BNB_RECOMPUTE:
        theArgumentsStruc->BnB_consistentCompleteInformationHeur = false;
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp BnB_argp = { BNB_options, BNB_parse_argument,
                                 BNB_args_doc, BNB_doc };
const struct argp_child BnB_child = {&BnB_argp, 0, "BNB options", GID_BNB};

//Event-Driven POMDP options
static const int GID_EVENTPOMDP=GID_SM;
const char *eventPomdp_argp_version = "Event-Driven POMDP options parser 0.1";
static const char *eventPomdp_args_doc = 0;
static const char *eventPomdp_doc = 
"This is the documentation for the options options parser\
This parser should be included as a child argp parser in the \
main argp parser of your application. (and this message will\
not be shown)";

static struct argp_option eventPomdp_options[] = {
{"marginalize",   'm',"MARGININDEX",  0, "Marginalize a state factor out of the flat transition and observation models. This is useful for problems which you want to describe graphically, but have to solve with a non-factored solver like Perseus. Use only for Event-Driven POMDPs."},
{"falseNegative", 'F',"FALSEINDEX", 0, "Index of a joint observation corresponding to a system-wide false negative detection of an event. This is usually necessary for realistic results in Event-Driven POMDPs."},
{ 0 }
};

error_t
eventPomdp_parse_argument (int key, char *arg, struct argp_state *state)
{
    /* Get the input argument from argp_parse, which we
      know is a pointer to our arguments structure. */
    struct Arguments *theArgumentsStruc = (struct Arguments*) state->input;

    switch (key)
    {
    case 'm':
        theArgumentsStruc->marginalize = true;
        theArgumentsStruc->marginalizationIndex = atoi(arg);
        break;
    case 'F':
        theArgumentsStruc->falseNegativeObs = atoi(arg);
        break;
    default:
        return ARGP_ERR_UNKNOWN;
    }
    return 0;
}
static struct argp eventPomdp_argp = { eventPomdp_options, eventPomdp_parse_argument,
                                        eventPomdp_args_doc, eventPomdp_doc };
const struct argp_child eventPomdp_child = {&eventPomdp_argp, 0,
                                            "Event-Driven POMDP options", GID_EVENTPOMDP};

} // end ArgumentHandlers namespace

