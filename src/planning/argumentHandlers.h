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

#ifndef  _ARGUMENTHANDLERS_H_
#define  _ARGUMENTHANDLERS_H_ 1

#include <iostream>
#include <stdlib.h>
#include <argp.h>
#include <vector>
#include "Globals.h"
#include "qheur.h"
#include "gmaatype.h"
#include "JESPtype.h"
#include "PerseusBackupType.h"
#include "BGBackupType.h"
#include "BGIP_SolverType.h"
#include "ProblemType.h" //problem enum.
#include "ProblemAloha.h"
#include "BnB_JointTypeOrdering.h"

/// \brief ArgumentHandlers contains functionality for parsing and
/// handling command-line arguments.
namespace ArgumentHandlers {

using namespace qheur;
using namespace GMAAtype;
using namespace JESPtype;
using namespace BGIP_SolverType;
using namespace ProblemType;
using namespace BGIP_BnB;

/// Arguments contains all defined parameters to be set on the command line. 
struct Arguments
{
    //General options (globopt)
    int verbose;    // < 0 means quiet, > 0 is verbose
    
    //The dec-pomdp file argument (dpf)
    const char * dpf; //the dpomdp file string
    //do we use a standard (i.e. .cpp) problem rather than parsing:
    Problem_t problem_type;
	//For FFG:
    size_t nrAgents;
    size_t nrHouses;
    size_t nrFLs;
    ProblemAloha::IslandConfiguration islandConf;
    ProblemAloha::AlohaVariation alohaVariation;
    size_t maxBacklog;

    //output file options (outputFileOptions)
    int dryrun;
    char * description, * prefix;
    int noReCompute;

    //model options (modelOptions)
    bool cache_flat_models;
    int sparse;
    int isTOI;
    double discount;

    //solution methods options (solutionMethodOptions)
    int horizon;
    bool infiniteHorizon;
    int nrRestarts; //the number of times that an algorithm is run

    //the jpolIndex input argument
    LIndex jpolIndex;

    // BGSolver options
    BGIP_Solver_t bgsolver; //the BG solver type
    double deadline;

    // GMAA options
    GMAA_t gmaa; //the GMAA type
    int k;
    int saveAllBGs;
    int saveTimings;
    int useQcache;
    bool requireQcache;
    double slack;  // slack parameter to stop search before finding optimal solution
    size_t GMAAdeadline;

    // GMAA Cluster options
    int useBGclustering;
    int BGClusterAlgorithm;

    // JESP options
    JESP_t jesp;

    // alternating maximization options
    int nrAMRestarts;

    // Perseus options
    int savePOMDP;
    int saveIntermediateV;
    int minimumNrIterations;
    int initializeWithImmediateReward;
    int initializeWithZero;

    // Perseus belief set sampling options
    int uniqueBeliefs;
    int nrBeliefs;
    int saveBeliefs;
    int resetAfter;
    int useQMDPforSamplingBeliefs;
    double QMDPexploreProb;

    // Perseus backup options
    PerseusBackupType backup;
    BGBackupType bgBackup;
    double waitPenalty;
    double weight;
    int commModel;
    int computeVectorForEachBelief;
    
    // Qheur options
    Qheur_t qheur;
    size_t QHybridHorizonLastTimeSteps;
    Qheur_t QHybridFirstTS;
    Qheur_t QHybridLastTS;
    bool TreeIPpruneAfterUnion;
    bool TreeIPpruneAfterCrossSum;
    bool TreeIPuseVectorCache;

    // Simulation options
    int nrRuns;
    int randomSeed;

    // TOI options
    int TOIpolicy;

    // TOIcentralized options
    int TOIcentralized;
    int useCentralizedModels;

    // CE (cross-entropy) options 
    size_t nrCERestarts;
    size_t nrCEIterations;
    size_t nrCESamples;
    size_t nrCESamplesForUpdate;
    bool CE_use_hard_threshold; //(gamma in CE papers)
    double CE_alpha; //the learning rate
    size_t nrCEEvalutionRuns; // number of policy evaluation runs

    // online POMDP options
    int nrNodesExpanded;

    // RL options;
    int nrRLruns;
    int nrIntermediateEvaluations;
    int startAtRLrun;

    //Maxplus options
    size_t maxplus_maxiter;
    size_t maxplus_verbose;
    double maxplus_damping;
    size_t maxplus_nrRestarts;
    std::string maxplus_updateT;

    // BGIP Branch-and-Bound Options
    BnB_JointTypeOrdering BnBJointTypeOrdering;
    bool BnB_keepAll;
    bool BnB_consistentCompleteInformationHeur;

    // solveBG options
    const char * bgFilename;
    int startIndex;
    int endIndex;

    //Event-Driven POMDP options
    bool marginalize;
    size_t marginalizationIndex;
    int falseNegativeObs;

    //default values by constructor:
    Arguments()
    {
        // general
        verbose = 0;
        // problem file
        dpf = 0;
        problem_type = PARSE;
        nrAgents = 2;
        nrHouses = 3;
        nrFLs = 3;
        islandConf = ProblemAloha::TwoIslands;
        alohaVariation = ProblemAloha::NoNewPacket;
        maxBacklog = 2;

        // output
        dryrun = 0;
        description = NULL;
        prefix = NULL;
        noReCompute = 0;

        // model
        cache_flat_models = false;
        sparse = 0;
        isTOI = 0;
        discount = -1;

        // solution method
        horizon = 2;
        infiniteHorizon = false;
        nrRestarts = 1;

        // jpol index
        jpolIndex = 0;

        // BGSolver options
        bgsolver = BFS;
        deadline = 0;

        // GMAA
        gmaa = MAAstarClassic;
        nrAMRestarts = 10;
        k = 1;
        saveAllBGs = 0;
        saveTimings = 0; // also used for Perseus
        useQcache = 0;
        requireQcache = false;
        slack = 0.0;
        GMAAdeadline = 0;

        // GMAA Cluster
        useBGclustering = 0;
        BGClusterAlgorithm = 0;
 
        //JESP options
        jesp = JESPDP;

        // Perseus
        savePOMDP = 0;
        saveIntermediateV = 0;
        minimumNrIterations = 0;
        initializeWithImmediateReward = 0;
        initializeWithZero = 0;

        // Perseus belief set sampling options
        nrBeliefs = 10;
        saveBeliefs = 0;
        resetAfter = 0;
        uniqueBeliefs = 0;
        useQMDPforSamplingBeliefs = 0;
        QMDPexploreProb = 0.1;

        // PerseusBackup
        backup = POMDP;
        bgBackup = BGIP_SOLVER_EXHAUSTIVE;
        waitPenalty = -1;
        weight = -1;
        commModel = -1;
        computeVectorForEachBelief = 0;

        // Qheur options
        qheur = eQMDP;
        QHybridHorizonLastTimeSteps = 0;
        QHybridFirstTS = eQBG;
        QHybridLastTS = eQMDP;
        TreeIPpruneAfterUnion = true;
        TreeIPpruneAfterCrossSum = true;
        TreeIPuseVectorCache = true;

        // Simulation options
        nrRuns = 1000;
        randomSeed = 42;

        // TOI options
        TOIpolicy = 0;

        // TOIcentralized options
        TOIcentralized = 0;
        useCentralizedModels = 0;

        // CE (cross-entropy) options 
        nrCERestarts = 10;
        nrCEIterations = 50;
        nrCESamples = 50;
        nrCESamplesForUpdate = 10;
        CE_use_hard_threshold = 1; //(gamma in CE papers)
        CE_alpha = 0.3; //the learning rate
        nrCEEvalutionRuns = 0; // number of policy evaluation runs. 0 = exact evaluation

        // online POMDP 
        nrNodesExpanded = 10;

        // RL options;
        nrRLruns=10000;
        nrIntermediateEvaluations=10;
        startAtRLrun=0;
        
        // options for max-plus
        maxplus_maxiter = 25;
        maxplus_verbose = 0;
        maxplus_damping = 0.5;
        maxplus_nrRestarts = 1;
        maxplus_updateT = std::string("PARALL");
        BnBJointTypeOrdering = IdentityMapping;
        BnB_keepAll = false;
        BnB_consistentCompleteInformationHeur = true;

        // solveBG options
        bgFilename = 0;
        startIndex = -1;
        endIndex = -1;

        // Event-Driven POMDP options
        marginalize = false;
        marginalizationIndex = false;
        falseNegativeObs = -1;
    }
        
};


extern const char *argp_program_bug_address;

error_t problemFile_parse_argument (int key, char *arg,
                                    struct argp_state *state);
extern const struct argp_child problemFile_child;

error_t globalOptions_parse_argument (int key, char *arg,
                                      struct argp_state *state);
extern const struct argp_child globalOptions_child;

error_t outputFileOptions_parse_argument (int key, char *arg,
                                          struct argp_state *state);
extern const struct argp_child outputFileOptions_child;

error_t modelOptions_parse_argument (int key, char *arg,
                                     struct argp_state *state);
extern const struct argp_child modelOptions_child;

error_t solutionMethodOptions_parse_argument (int key, char *arg,
                                              struct argp_state *state);
extern const struct argp_child solutionMethodOptions_child;

error_t jpolIndex_parse_argument (int key, char *arg, 
                                  struct argp_state *state);
extern const struct argp_child jpolIndex_child;

error_t bgsolver_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child bgsolver_child;

error_t gmaa_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child gmaa_child;

error_t gmaa_cluster_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child gmaa_cluster_child;

error_t perseus_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child perseus_child;

error_t perseusbackup_parse_argument (int key, char *arg, 
                                      struct argp_state *state);
extern const struct argp_child perseusbackup_child;

error_t perseusbelief_parse_argument (int key, char *arg, 
                                      struct argp_state *state);
extern const struct argp_child perseusbelief_child;

error_t qheur_parse_argument (int key, char *arg, 
                              struct argp_state *state);
extern const struct argp_child qheur_child;


error_t simulation_parse_argument (int key, char *arg, 
                                   struct argp_state *state);
extern const struct argp_child simulation_child;

error_t toi_parse_argument (int key, char *arg, 
                            struct argp_state *state);
extern const struct argp_child toi_child;

error_t toicentralized_parse_argument (int key, char *arg, 
                                       struct argp_state *state);
extern const struct argp_child toicentralized_child;

error_t CE_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child CE_child;

error_t MaxPlus_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child MaxPlus_child;

error_t JESP_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child JESP_child;

error_t onlinePOMDP_parse_argument (int key, char *arg, 
                             struct argp_state *state);
extern const struct argp_child onlinePOMDP_child;

error_t RL_parse_argument (int key, char *arg, 
                           struct argp_state *state);
extern const struct argp_child RL_child;
error_t BnB_parse_argument (int key, char *arg, 
                           struct argp_state *state);
extern const struct argp_child BnB_child;


error_t eventPomdp_parse_argument (int key, char *arg, 
                                    struct argp_state *state);
extern const struct argp_child eventPomdp_child;

} //namespace

#endif /* !_ARGUMENTHANDLERS_H_*/
