
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    params.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    August, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: params.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/params.h,v $
 *    $Revision: 1.8 $
 *    $Date: 2004/11/01 04:27:34 $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    1994-1997, Brown University
 *    1998-2003, Anthony R. Cassandra
 *
 *    All Rights Reserved
 *                          
 *    Permission to use, copy, modify, and distribute this software and its
 *    documentation for any purpose other than its incorporation into a
 *    commercial product is hereby granted without fee, provided that the
 *    above copyright notice appear in all copies and that both that
 *    copyright notice and this permission notice appear in supporting
 *    documentation.
 * 
 *    ANTHONY CASSANDRA DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 *    INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR ANY
 *    PARTICULAR PURPOSE.  IN NO EVENT SHALL ANTHONY CASSANDRA BE LIABLE FOR
 *    ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *    WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *    ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *    OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  </COPYRIGHT>
 *
 *</SOURCE_HEADER>
 */

#ifndef PARAMS_H
#define PARAMS_H

/* Need this for stat field. */
#include "global.h"
#include "alpha.h"
#include "stats.h"
#include "pomdp-solve-options.h"

/*******************************************************************/
/**************      USEFUL MNENOMIC CONSTANTS      ****************/
/*******************************************************************/

/* The suffix to use for alpha vector files. */
#define ALPHA_FILE_SUFFIX                    ".alpha"

/* The suffix to use for policy graph files. */
#define PG_FILE_SUFFIX                       ".pg"

/* The suffix to use for penultimate alpha vector files. */
#define PENULTIMATE_SUFFIX                    ".prev"

/* We will save the solution after each iteration in a temporary file,
   so that if the program terminates abnormally, we can recover the
   latest solution. We will use the PID of the file to make sure the
   filename is unique so multiple copies can run at the same time. */
#define SAVE_FILE_NAME_FORMAT                "##pomdp-solve-%d##"

/*******************************************************************/
/**************             TYPEDEFS                ****************/
/*******************************************************************/

/* When purging a set of alpha vectors we are faced with a number of
   choices.  We can do nothing (i.e., not purge them), we can use only
   a simple domination check, or we can use linear programs to get a
   completely parsimonious set (i.e., prune).  For the pruning, it
   always makes sense to do domination checking as well, so we
   conciously did not allow specification of LPs without domination
   checking, i.e., pruning assumes you want the domination checking,
   which will be done first. */
#define MAX_NUM_PURGE_OPTIONS          4
typedef enum { purge_none, 
               purge_dom, 
               purge_prune,
               purge_epsilon_prune } PurgeOption;
#define PURGE_OPTION_STRINGS     { \
                                      "none", \
                                      "domonly", \
                                      "normal_prune", \
                                      "epsilon_prune" \
                                  }

#define MAX_INC_PRUNE_TYPES                    3
typedef enum { NormalIp, 
               RestrictedRegionIp, 
               GeneralizedIp } GeneralizedIpChoice;
#define INC_PRUNE_TYPE_STRINGS     { \
                                      "normal", \
                                      "restricted_region", \
                                      "generalized" \
                                   }     

#define MAX_VI_VARIATION_TYPES                    4
typedef enum { NormalVi, 
               ZlzSpeedup,
               AdjustableEpsilonVi, 
               FixedSolnSizeVi } ViVariation;
#define VI_VARIATION_TYPE_STRINGS     { \
                                      "normal", \
                                      "zlz", \
                                      "adjustable_epsilon", \
                                      "fixed_soln_size" \
                                   }     

/* We just want to encapsulate all the parameters used in the
   pomdp-solve program into a single structure for convenience. These
   do not include any parameters that are specific to a particular
   algorithm. */ 
typedef struct PomdpSolveParamStruct *PomdpSolveParams;
struct PomdpSolveParamStruct {

  /* This is the structure that holds all the things that can be
	configured through the command line or configuration file. */
  PomdpSolveProgOptions opts;

  /* We'll keep track of the epoch (iteration) as value iteration
     progresses. */
  int cur_epoch;

  /* When using the ZLZ value iteration speedup, the number of value
     function epochs and the number of value function *updates* are
     not the same.  This is because there are intermediate value
     function updates done during each epoch.  There are times when it
     is more conveneient to know how many times the value function was
     updated. One instance of this is when we need to adjust for
     negative rewards. This measn we add a constant value to the
     rewards, which then accumulate with each update.  To know the
     true value function with respect to the original problem requires
     knowing exactly how many times these adjusted rewards were
     incorporated into the value function. This corresponds to the
     number of updates. */
  int update_count;

  /* Since we save the stats for each epoch, we can optionally print
     them out when the program is finished executing. */
  int stat_summary;
  
  /* Name of file to print all output information to.  Defaults to
     stdout. */
  char report_filename[MAX_FILENAME_LENGTH];

  /* All messages will be output to the same file handle, which
     defaults to stdout if no filename is given on the command
     line. */
  FILE *report_file;

  /* The name of the POMDP file we will use. */
  char param_filename[MAX_FILENAME_LENGTH];

  /* We allow the discount factor to be overridden on the command
     line. If we don't over-ride the discount factor then set the
     override variable to be negative.  This will indicate whether we
     should override the file's discount factor after we read the
     file. */
  double override_discount;

  /* How many epoch to run the algorithm for. */
  int horizon;

  /* Names of files to write out solutions */
  char alpha_filename[MAX_FILENAME_LENGTH];   /* alpha vectors */
  char pg_filename[MAX_FILENAME_LENGTH];      /* policy graph  */

  /* Sometimes we want to initialize value iteration with an initial
     value function. */
  char initial_policy_filename[MAX_FILENAME_LENGTH];
  AlphaList initial_policy;

  /* We can set a timer to interrupt the program after too many
     seconds have elapsed.  It is done from the command line and is
     optional. Zero or negative values turns time limits off.  */
  int max_secs;

  /* The nature of these problems is such that they could require all
     the memory in the universe.  Therefore, to prevent them from
     going out and actually searching for all this memory, we put a
     ceiling on how much memory it can consume.  This can also be set
     via the command line if more or less is desired. Zero or negative
     values turns memory limits off. */
  int memory_limit;

  /* Whether or not to save each epoch's solution in a separate
     file. */
  int save_all;


  /* When creating the projection sets at the start of each epoch, we
     have a choice of purging each projectioon set.  This defines what
     purging option to use. */
  PurgeOption proj_purge;

  /* An optimization we can use it to save witness points as we
     uncover them for a vector and use these points to help prime
     later prune operations. */
  int use_witness_points;

  /* The filename to use as a backup file for each iteration's answer. */
  char backup_file[1024];

  /* The filename to use for previous epoch solution. */
  char penultimate_filename[1024];

  /* For the algorithms which construct the value function one action
     at a time, the final step is to merge and purge all the
     single-action (Q) value functions.  This specifies how to purge
     them. This is only valid for those algorithms which build up the
     solution one action at a time. */
  PurgeOption q_purge_option;

  /* There are many places in the code where a simple domination check
     can be sued to save time.  This globally turns this option onor
     off for all of these places. */
  int domination_check;

  /* An optimization of a few of the algorithms is to preceed any linear
     programming for finding the Q set with thowing a bunch of
     random points out and finding the vectors for these points.  This
     saves us from having to do LPs to find these vectors.  Not clear
     how useful it will be, but I know you cannot make any theoretical
     claims about it. */
  int alg_init_rand_points;

  /* An optimization of the prune() routine is to preceed linear
     programming for finding parsimonious sets with thowing a bunch of
     random points out and finding the vectors for these points.  This
     saves us from having to do LPs to find these vectors.  Not clear
     how useful it will be, but I know you cannot make any theoretical
     claims about it. */
  int prune_init_rand_points;

  /* In this section, we gather all the values that deal with floating
     point comparison/precision problems.  */
  double stop_delta;

  /* General epsilon to use for numerical comparisons. */
  double epsilon;

  /* Epsilon to use in the linear programs.  */
  double lp_epsilon;

  /* Epsilon to use when doing epsilon pruning. */
  double prune_epsilon;

  /* There are many places where we compare one alpha vector to
     another or to some specific value.  This is the epsilon that is
     used in those comparisons. */
  double alpha_epsilon;

  /* Precision for comparison of vertices in the linear support
     algorithm. */
  double vertex_epsilon;

  /* Solution value precision for each iteration, used to compare the
     LP objective values. */
  double sparse_epsilon;

  /* Given the model parameters, it can be impossible to get a
     particular observation for a situation.  To detect this we
     compute the probability of the observation and then compare it to
     zero.  As will all our floating point comparisons, we need a
     tolerance factor. */
  double impossible_obs_epsilon;

  double double_equality_precision;

  /* Place to hang statistics off of (optional) */
  SolutionStats stat;

  /****************************************/
  /****  Algorithm specific section  ******/
  /****************************************/

  /* Specific for the incremental pruning algorithm, this says what
     type of incremental pruning to use. */
  GeneralizedIpChoice ip_type;

  PurgeOption enum_purge_option;

  PurgeOption fg_purge_option;

  /****************************************/
  /***  VI variation specific section  */
  /****************************************/

  /* This is used as temporary storage to keep the resulting maximal
     computed difference between the original and epislon pruned set
     when using the epsilon pruning option.  We store it here and move
     it elsewhere if we need it.  Because the prune algorithm can be
     called in a lot of context, it is up to the overall calling
     context to decide exactly what is does with it. */
  double epsilon_diff_of_last_prune;

  /* Defines exactly what value iteration variation to use. */
  ViVariation vi_variation;

  /* These are use for automatic epsilon adjustment. */
  double starting_epsilon;
  double ending_epsilon;
  double epsilon_adjust_factor;

  /* Sets the maximum number of vectors for a given epoch.  A value
     less than '1' mean that there is no upper limit. */
  int max_soln_size;

  /* When testing whether to increment/decrement the epsilon, we use
     some window of the past hostory of the epochs to decide.
     Specifically, we look at the recent history of the solution sizes
     over the last few epochs.  These define the hostory window length
     and the amount by which we assume a change is not important. */
  int epoch_history_window_length; 
  int epoch_history_window_delta;

};

/*******************************************************************/
/**************         DEFAULT VALUES              ****************/
/*******************************************************************/

#define DEFAULT_STOP_DELTA              1e-9
#define DEFAULT_VERBOSE                 0
#define DEFAULT_HORIZON                 -1
#define DEFAULT_PREFIX                  "solution"
#define DEFAULT_SAFE                    true
#define DEFAULT_CONTEXT_DETAILS         0
#define DEFAULT_PROJECTION_PURGE        purge_prune
#define DEFAULT_USE_WITNESS_POINTS      0
#define DEFAULT_Q_PURGE_OPTION          purge_prune
#define DEFAULT_DOMINATION_CHECK        1

/* Main epsilon parameter which many other epsilons need to be derived
   from. */
#define DEFAULT_EPSILON                 1e-9

/* Main epsilon parameter which many other epsilons need to be derived
   from. */
#define DEFAULT_LP_EPSILON              1e-9

/* Epsilon parameter for considering model parameters zero or non-zero
   when using sparse representations. */
#define DEFAULT_SPARSE_EPSILON          1e-9

/* Main epsilon parameter which many other epsilons need to be derived
   from. */
#define DEFAULT_PRUNE_EPSILON           1e-3

/* For comparisons that involve alpha vector and components. */
#define DEFAULT_ALPHA_EPSILON           1e-9

/* Epsilon for the vertex comparison in the vertex enumeration. */
#define DEFAULT_VERTEX_EPSILON          1e-9

/* When comparing numbers, how accurate should we be? */
#define DEFAULT_DOUBLE_EQUALITY_PRECISION          1e-9

/* When summing all probablity for a particular observation, if it is
   close enough to zero, then this observation is deemed not
   possible.  This epsilon factor says how close it has to be. */
#define DEFAULT_IMPOSSIBLE_OBS_EPSILON             1e-9

/* An optimization is to preceed linear programming for finding
   parsimonious sets with thowing a bunch of random points out and
   finding the vectors for these points.  This saves us from having to
   do LPs to find these vectors.  Not clear how useful it will be, but
   I know you cannot make any theoretical claims about it. One is for
   the algorithm set initialization and the other for the prune()
   routine. */
#define DEFAULT_ALG_INIT_RAND_POINTS               0
#define DEFAULT_PRUNE_INIT_RAND_POINTS             0

/****************************************/
/****  Algorithm specific section  ******/
/****************************************/

#define DEFAULT_INC_PRUNE_TYPE                   NormalIp

/* How to purge the set of vectors that are enumerated. */
#define DEFAULT_ENUM_PURGE_OPTION                purge_prune

/* How to purge the set of vectors that are created from the finite
   grid. */  
#define DEFAULT_FG_PURGE_OPTION                  purge_prune

/****************************************/
/***  VI variation specific section  */
/****************************************/

#define DEFAULT_VI_VARIATION                    NormalVi
#define DEFAULT_STARTING_EPSILON                1e-1
#define DEFAULT_ENDING_EPSILON                  1e-3
#define DEFAULT_EPSILON_ADJUST_FACTOR           10.0
#define DEFAULT_MAX_SOLN_SIZE                   10
#define DEFAULT_HISTORY_WINDOW_LENGTH           5
#define DEFAULT_HISTORY_WINDO_DELTA             3

/*******************************************************************/
/**************       COMMAND LINE OPTIONS          ****************/
/*******************************************************************/

#define CMD_ARG_POMDP_FILE           "-p"
#define CMD_ARG_DISCOUNT             "-discount"

#define CMD_ARG_TERMINAL_VALUES      "-terminal_values"
#define CMD_ARG_HORIZON              "-horizon"
#define CMD_ARG_DELTA                "-stop_delta"

#define CMD_ARG_OUTPUT               "-o"
#define CMD_ARG_SAVE_ALL             "-save_all"
#define CMD_ARG_REPORT_FILE          "-stdout"
#define CMD_ARG_STAT_SUMMARY         "-stat_summary"

#define CMD_ARG_MEMORY_LIMIT         "-memory_limit"
#define CMD_ARG_MAX_SECS             "-time_limit"

#define CMD_ARG_Q_PURGE              "-q_purge"
#define CMD_ARG_DOM_CHECK            "-dom_check"

#define CMD_ARG_PROJ_PURGE           "-proj_purge"
#define CMD_ARG_WITNESS_POINTS       "-witness_points"

#define CMD_ARG_RAND_SEED            "-rand_seed"

#define CMD_ARG_ALG_INIT_RAND        "-alg_rand"
#define CMD_ARG_PRUNE_INIT_RAND      "-prune_rand"

#define CMD_ARG_EPSILON              "-epsilon"
#define CMD_ARG_LP_EPSILON           "-lp_epsilon"
#define CMD_ARG_PRUNE_EPSILON        "-prune_epsilon"

/****************************************/
/****  Algorithm specific section  ******/
/****************************************/

#define CMD_ARG_INC_PRUNE_TYPE             "-inc_prune"
#define CMD_ARG_ENUM_PURGE_OPTION          "-enum_purge"
#define CMD_ARG_FG_PURGE_OPTION            "-fg_purge"

/****************************************/
/****  VI variation specific section  ***/
/****************************************/

#define CMD_ARG_VI_VARIATION                    "-vi_variation"
#define CMD_ARG_STARTING_EPSILON                "-start_epsilon"
#define CMD_ARG_ENDING_EPSILON                  "-end_epsilon"
#define CMD_ARG_EPSILON_ADJUST_FACTOR           "-epsilon_adjust"
#define CMD_ARG_SOLN_SIZE                       "-max_soln_size"
#define CMD_ARG_HISTORY_WINDOW_LENGTH           "-history_length"
#define CMD_ARG_HISTORY_WINDO_DELTA             "-history_delta"

/*******************************************************************/
/**************       EXTERNAL VARIABLES            ****************/
/*******************************************************************/

extern char *purge_option_str[];
extern char *inc_prune_type_str[];

/*******************************************************************/
/**************       EXTERNAL FUNCTIONS            ****************/
/*******************************************************************/

/* Creates the memory for the structure to hold the parameters used in
   solving a POMDP.  Also sets the fields to the default values.  */
extern PomdpSolveParams newPomdpSolveParams(  );

/* Frees the memory for pointers in the params and the param structure
   itself.
*/
 extern void destroyPomdpSolveParams( PomdpSolveParams param );

/*
  Main routine for parsing config file and command line. 
*/
extern PomdpSolveParams parseCmdLineAndCfgFile( int argc, char **argv );

/* Display to stdout the current program parameters. */
extern PomdpSolveParams showPomdpSolveParams( PomdpSolveParams params );

#endif
