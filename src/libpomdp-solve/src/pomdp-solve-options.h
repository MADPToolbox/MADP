
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    pomdp-solve-options.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    February 2005
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: pomdp-solve-options.h,v $
 *    $Source: pomdp-solve-options.h,v $
 *    $Revision: 1.6 $
 *    $Date: February 2005 $
 *    $Author: arc $

*  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *

 *    2005, Anthony R. Cassandra
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

/*
 * This code was automatically generated on February 2005 by the program:
 *
 *     gen-program-opts.py
 */

#ifndef POMDP_SOLVE_OPTS_OPTIONS_H
#define POMDP_SOLVE_OPTS_OPTIONS_H

#include "global.h"

#include "program-options.h"

/*
 * Enumerated types for 'enum' parameters.
 */

typedef enum {
  POMDP_SOLVE_OPTS_Verbose_context,
  POMDP_SOLVE_OPTS_Verbose_lp,
  POMDP_SOLVE_OPTS_Verbose_global,
  POMDP_SOLVE_OPTS_Verbose_timing,
  POMDP_SOLVE_OPTS_Verbose_stats,
  POMDP_SOLVE_OPTS_Verbose_cmdline,
  POMDP_SOLVE_OPTS_Verbose_main,
  POMDP_SOLVE_OPTS_Verbose_alpha,
  POMDP_SOLVE_OPTS_Verbose_proj,
  POMDP_SOLVE_OPTS_Verbose_crosssum,
  POMDP_SOLVE_OPTS_Verbose_agenda,
  POMDP_SOLVE_OPTS_Verbose_enum,
  POMDP_SOLVE_OPTS_Verbose_twopass,
  POMDP_SOLVE_OPTS_Verbose_linsup,
  POMDP_SOLVE_OPTS_Verbose_witness,
  POMDP_SOLVE_OPTS_Verbose_incprune,
  POMDP_SOLVE_OPTS_Verbose_lpinterface,
  POMDP_SOLVE_OPTS_Verbose_vertexenum,
  POMDP_SOLVE_OPTS_Verbose_mdp,
  POMDP_SOLVE_OPTS_Verbose_pomdp,
  POMDP_SOLVE_OPTS_Verbose_param,
  POMDP_SOLVE_OPTS_Verbose_parsimonious,
  POMDP_SOLVE_OPTS_Verbose_region,
  POMDP_SOLVE_OPTS_Verbose_approx_mcgs,
  POMDP_SOLVE_OPTS_Verbose_zlz_speedup,
  POMDP_SOLVE_OPTS_Verbose_finite_grid,
  POMDP_SOLVE_OPTS_Verbose_mcgs,
  POMDP_SOLVE_OPTS_Verbose__END__
} POMDP_SOLVE_OPTS_Verbose_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Inc_Prune_normal,
  POMDP_SOLVE_OPTS_Inc_Prune_restricted_region,
  POMDP_SOLVE_OPTS_Inc_Prune_generalized,
  POMDP_SOLVE_OPTS_Inc_Prune__END__
} POMDP_SOLVE_OPTS_Inc_Prune_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Enum_Purge_none,
  POMDP_SOLVE_OPTS_Enum_Purge_domonly,
  POMDP_SOLVE_OPTS_Enum_Purge_normal_prune,
  POMDP_SOLVE_OPTS_Enum_Purge_epsilon_prune,
  POMDP_SOLVE_OPTS_Enum_Purge__END__
} POMDP_SOLVE_OPTS_Enum_Purge_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Fg_Type_simplex,
  POMDP_SOLVE_OPTS_Fg_Type_pairwise,
  POMDP_SOLVE_OPTS_Fg_Type_search,
  POMDP_SOLVE_OPTS_Fg_Type_initial,
  POMDP_SOLVE_OPTS_Fg_Type_file,
  POMDP_SOLVE_OPTS_Fg_Type__END__
} POMDP_SOLVE_OPTS_Fg_Type_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Q_Purge_none,
  POMDP_SOLVE_OPTS_Q_Purge_domonly,
  POMDP_SOLVE_OPTS_Q_Purge_normal_prune,
  POMDP_SOLVE_OPTS_Q_Purge_epsilon_prune,
  POMDP_SOLVE_OPTS_Q_Purge__END__
} POMDP_SOLVE_OPTS_Q_Purge_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Stop_Criteria_exact,
  POMDP_SOLVE_OPTS_Stop_Criteria_weak,
  POMDP_SOLVE_OPTS_Stop_Criteria_bellman,
  POMDP_SOLVE_OPTS_Stop_Criteria__END__
} POMDP_SOLVE_OPTS_Stop_Criteria_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Method_enum,
  POMDP_SOLVE_OPTS_Method_twopass,
  POMDP_SOLVE_OPTS_Method_linsup,
  POMDP_SOLVE_OPTS_Method_witness,
  POMDP_SOLVE_OPTS_Method_incprune,
  POMDP_SOLVE_OPTS_Method_grid,
  POMDP_SOLVE_OPTS_Method_mcgs,
  POMDP_SOLVE_OPTS_Method__END__
} POMDP_SOLVE_OPTS_Method_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Fg_Purge_none,
  POMDP_SOLVE_OPTS_Fg_Purge_domonly,
  POMDP_SOLVE_OPTS_Fg_Purge_normal_prune,
  POMDP_SOLVE_OPTS_Fg_Purge_epsilon_prune,
  POMDP_SOLVE_OPTS_Fg_Purge__END__
} POMDP_SOLVE_OPTS_Fg_Purge_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Proj_Purge_none,
  POMDP_SOLVE_OPTS_Proj_Purge_domonly,
  POMDP_SOLVE_OPTS_Proj_Purge_normal_prune,
  POMDP_SOLVE_OPTS_Proj_Purge_epsilon_prune,
  POMDP_SOLVE_OPTS_Proj_Purge__END__
} POMDP_SOLVE_OPTS_Proj_Purge_Type;

typedef enum {
  POMDP_SOLVE_OPTS_Vi_Variation_normal,
  POMDP_SOLVE_OPTS_Vi_Variation_zlz,
  POMDP_SOLVE_OPTS_Vi_Variation_adjustable_epsilon,
  POMDP_SOLVE_OPTS_Vi_Variation_fixed_soln_size,
  POMDP_SOLVE_OPTS_Vi_Variation__END__
} POMDP_SOLVE_OPTS_Vi_Variation_Type;

/*
 * String arrays for 'enum' parameters.
 */

#define POMDP_SOLVE_OPTS_OPT_VERBOSE_STRINGS { \
    "context", \
    "lp", \
    "global", \
    "timing", \
    "stats", \
    "cmdline", \
    "main", \
    "alpha", \
    "proj", \
    "crosssum", \
    "agenda", \
    "enum", \
    "twopass", \
    "linsup", \
    "witness", \
    "incprune", \
    "lpinterface", \
    "vertexenum", \
    "mdp", \
    "pomdp", \
    "param", \
    "parsimonious", \
    "region", \
    "approx_mcgs", \
    "zlz_speedup", \
    "finite_grid", \
    "mcgs", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_INC_PRUNE_STRINGS { \
    "normal", \
    "restricted_region", \
    "generalized", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_ENUM_PURGE_STRINGS { \
    "none", \
    "domonly", \
    "normal_prune", \
    "epsilon_prune", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_FG_TYPE_STRINGS { \
    "simplex", \
    "pairwise", \
    "search", \
    "initial", \
    "file", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_Q_PURGE_STRINGS { \
    "none", \
    "domonly", \
    "normal_prune", \
    "epsilon_prune", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_STOP_CRITERIA_STRINGS { \
    "exact", \
    "weak", \
    "bellman", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_METHOD_STRINGS { \
    "enum", \
    "twopass", \
    "linsup", \
    "witness", \
    "incprune", \
    "grid", \
    "mcgs", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_FG_PURGE_STRINGS { \
    "none", \
    "domonly", \
    "normal_prune", \
    "epsilon_prune", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_PROJ_PURGE_STRINGS { \
    "none", \
    "domonly", \
    "normal_prune", \
    "epsilon_prune", \
    "" \
  }

#define POMDP_SOLVE_OPTS_OPT_VI_VARIATION_STRINGS { \
    "normal", \
    "zlz", \
    "adjustable_epsilon", \
    "fixed_soln_size", \
    "" \
  }

/*
 * Structure to hold all parameters.
 */
typedef struct PomdpSolveProgOptionsStruct* PomdpSolveProgOptions;
struct PomdpSolveProgOptionsStruct {

  /* Executable name. */
  char __exec_name__[MAX_OPT_STRING_LEN];

  /* Executable version. */
  char __version__[MAX_OPT_STRING_LEN];

  /* Error flag for internal use only. */
  int __error__;

  /*
   *  This parameter allows you to set an upper bound on the
   * amount of time that this program will run. When this amount
   * of time has elapsed, the program execution is terminated.
   * Without specifying this parameter, there will be no upper
   * bound imposed by the pomdp-solve program.
   */
  int max_secs;

  /*
   *  Although the 'epsilon' precision value is used to determine
   * equality in an approximate manner, the solutions that are
   * saved and/or fed into the next value iteration epoch retain
   * their computed values. Thus, two value that are considered
   * the same from an epilon-approximate viewpoint might actually
   * have different values from a machine-precision viewpoint,
   * and thus wehn used to seed the next iteration may not lead
   * to the same values. This flag will force the alpha vector
   * coef values to be rounded in accordance with the 'epilson'
   * value after each iteration.
   */
  Boolean_Type force_rounding;

  /*
   *  The Monte-Carlo, Gauss-Seidel method using trajectories
   * through the belief space to lay down a grid of points that
   * we will compute the optimal value funciton for. This
   * parameter defines how frequently we should prune the set of
   * newly created value function facets during the generation of
   * the value function points.
   */
  int mcgs_prune_freq;

  /*
   *  Each main module of pomdp-solve can be separately
   * controlled as far as extra debugging output is concerned.
   * This option can be used more than once to turn on debugging
   * in more than one module.
   */
  POMDP_SOLVE_OPTS_Verbose_Type verbose;

  /*
   *  The pomdp-solve program displays much status and progress
   * information to stdout. If you want to have this redirected
   * to a file instead, provide the file name as this parameter.
   * Not specifying this option will simply make this information
   * go to normal stdout.
   */
  char report_filename[MAX_OPT_STRING_LEN];

  /*
   *  The incremental pruning algorithm has a number of
   * variations. This parameter selects the variation. We do not
   * yet discuss here the nuances of these variations.
   */
  POMDP_SOLVE_OPTS_Inc_Prune_Type ip_type;

  /*
   *  When using the 'adjustable_epsilon' value iteration
   * variant, we need to compare solution sizes from the the
   * rpevious epochs to see whethere or not the solutions are
   * staying relatively constant in size. To do this, we need to
   * define a past window length, as well as a tolerance on how
   * much variation in solution size we want to care about. This
   * parameter defines the length of the epoch window history to
   * use when determining whether it is time to adjust the
   * precision of the value iteration solution.
   */
  int epoch_history_window_length;

  /*
   *  There are a number of ways to prune sets of value function
   * components. Each uses a precision actor which is this
   * parameter.
   */
  double prune_epsilon;

  /*
   *  Normally, only the final solution is saved to a file, but
   * if you would like to write out the solution to every epoch
   * of value iteration, then set this flag to true. The epoch
   * number will be appened to the filenames that are output.
   */
  Boolean_Type save_all;

  /*
   *  All the information relevant to the solution of the POMDP
   * are written to files. This parameter allows you to set the
   * prefix to use for the given run of this program. The suffix
   * is generated internall by the program connected to the time
   * and contents of the various files.
   */
  char prefix_str[MAX_OPT_STRING_LEN];

  /*
   *  The finite grid method needs a set of belief points to
   * compute over. This parameter will turn on and off the saving
   * of these belief points to an external file.
   */
  Boolean_Type finite_grid_save;

  /*
   *  When using the enumeration method, there will be times
   * where the set of value function components will need to be
   * pruned or purged of useless components. This define the
   * pruning method to use for this algorithm.
   */
  POMDP_SOLVE_OPTS_Enum_Purge_Type enum_purge_option;

  /*
   *  The finite grid method needs a set of belief points to
   * compute over. There are a number of ways to generate this
   * grid, and this parameter selects the technique to use. We do
   * not yet here discuss the details of each of these.
   */
  POMDP_SOLVE_OPTS_Fg_Type_Type finite_grid_type;

  /*
   *  When using the finite grid approximation, as belief are
   * encountered, before they are put into the grid, they are
   * checked to make sure they are not already in the grid. This
   * comparison uses the value of this option in that comparison.
   * Therefore, you can limit the grid precision by twekaing this
   * parameter.
   */
  double fg_epsilon;

  /*
   *  The Monte-Carlo, Gauss-Seidel method using trajectories
   * through the belief space to lay down a grid of points that
   * we will compute the optimal value funciton for. This
   * parameter defines the number of value function update
   * iterations to use on a given set of trajectories.
   */
  int mcgs_traj_iter_count;

  /*
   *  Many solution procedures employ linear programming in their
   * algorithms. For those that do, thisk is the precision level
   * used inside the linear programming routines.
   */
  double lp_epsilon;

  /*
   *  When solving using the 'adjustable_epsilon' method of value
   * iteration, we need to specify both a staring and ending
   * precision. This is the ending precision.
   */
  double ending_epsilon;

  /*
   *  When solving using the 'adjustable_epsilon' method of value
   * iteration, we need to specify both a staring and ending
   * precision. This is the starting precision.
   */
  double starting_epsilon;

  /*
   *  There is a computationally simple, but not precision
   * domination check that can be done to discover useless
   * components of a value function. This is often useful, but
   * there are circumstances in which it is best to turn this
   * off.
   */
  Boolean_Type domination_check;

  /*
   *  When checking the stopping criteria at the end of each
   * value iteration epoch, some of the stopping condition types
   * use a tolerance/precision in their calculations. This
   * parameter allows you to set that precision.
   */
  double stop_delta;

  /*
   *  Some algorithms will separately solve the problem for
   * individual actions, then merge these results together. The
   * individual action solutions are referred to as the
   * "Q-functions". After merging, some pruning process will
   * likely take place, but we can also choose to do a pre-merge
   * pruning of these sets which often simplifies the merging
   * process. This parameter defines the method to use for this
   * pre-merge pruning.
   */
  POMDP_SOLVE_OPTS_Q_Purge_Type q_purge_option;

  /*
   *  All executions for solution of a POMDP needs a file as
   * input to the solution process. This filename by convention
   * will end with with or ".pomdp" or ".POMDP" and needs to
   * conform to the pomdp-solve file format (which is described
   * elsewhere.
   */
  char param_filename[MAX_OPT_STRING_LEN];

  /*
   *  The Monte-Carlo, Gauss-Seidel method using trajectories
   * through the belief space to lay down a grid of points that
   * we will compute the optimal value funciton for. This
   * parameter defines the number of trajectories to use.
   */
  int mcgs_num_traj;

  /*
   *  At the end of each epoch of value iteration, a check is
   * done to see whether the solutions have 'converged' to the
   * (near) optimal infinite horizon solution. there are more
   * than one way to determine this stopping condition. The exact
   * semantics of each are not described here at this time.
   */
  POMDP_SOLVE_OPTS_Stop_Criteria_Type stop_criteria;

  /*
   *  The pomdp-solve program implements a number of differnt
   * algorithms. This selects the one that should be used.
   * Details of each method not yet provided here.
   */
  POMDP_SOLVE_OPTS_Method_Type method;

  /*
   *  This parameter allows you to set an upper bound on the
   * amount of memory that this program uses. If the memory
   * threshold is met, the program execution is terminated.
   * Without specifying this parameter, there will be no upper
   * bound imposed by the pomdp-solve program (though the OS will
   * naturally have something to say about this).
   */
  int memory_limit;

  /*
   *  One can speed up the discovery of the initial shape of the
   * value function by randomly generating points and finding the
   * value function components needed for those points. This
   * technique is used if this parameter has a non-zero value.
   */
  int alg_init_rand_points;

  /*
   *  Value iteration assumes that at the end of the lifetime of
   * the decision maker that no more values will be accrued. This
   * corresponds to a terminal value function of all zeroes. This
   * is essentially the default starting point for the program.
   * However, with this parameter, you can set a different
   * terminal value function, which serves as the seed or initial
   * starting point for value iteration. Effectively, this allows
   * you to take the output of one value iteration run and send
   * it as input to the next. The file format for this input file
   * is identical to the output file format of this program (the
   * ".alpha" file).
   */
  char initial_policy_filename[MAX_OPT_STRING_LEN];

  /*
   *  The pomdp-solve program normally only saves the last
   * epoch's solution to a file. There is an option (save_all) to
   * save every iteration, but that can require a lot of space
   * for a long horizon. There are times when it is useful to
   * have the current and immediately previous epoch' solutions.
   * When this flag is set, you will always find the previous
   * epochs solution in *.prev.alpha (where '*' is the prefix
   * defined for the execution run.
   */
  Boolean_Type save_penultimate;

  /*
   *  This is the main precision setting parameter which will
   * effect the preciseness fo the solution procedures.
   */
  double epsilon;

  /*
   *  For any functionality that requires random numbers, we want
   * to be able to reproduce a given run by executing with the
   * same random number seed. This parameter allows you to set
   * the initial random seed by specifying a string consisting of
   * three integers separated by a colon (e.g.,
   * "34523:12987:50732" ) Not setting this value will result in
   * the random seed being pseudo-randomized based on the system
   * clock.
   */
  char rand_seed[MAX_OPT_STRING_LEN];

  /*
   *  This sets the discount factor to use during value iteration
   * which dictates the relative usefulness of future rewards
   * compared to immediate rewards.
   */
  double override_discount;

  /*
   *  The finite grid method needs a set of belief points to
   * compute over. There are a number of ways to generate this
   * grid, and this parameter selects the maximum number of
   * points that should be generated during this process.
   */
  int finite_grid_points;

  /*
   *  Defines the technique to use during pruning when the finite
   * grid method is being used.
   */
  POMDP_SOLVE_OPTS_Fg_Purge_Type fg_purge_option;

  /*
   *  Setting this flag will force the addition of an offset to
   * all immediate rewards to ensure that none of them are
   * negative.
   */
  Boolean_Type fg_nonneg_rewards;

  /*
   *  The first step for most algorithms is to compute the
   * forward projection of the previous iteration solution
   * components. Combinations of these will comprise the current
   * solution. Prior to emplying any algorithm to find which
   * combinations are needed (the heart of the POMDP solution
   * algorithms) we can employ a process of pruning the projected
   * set, often reducing the complexity of the algorithms. This
   * parameter decides what type of pruning to use at this step.
   * Details on the semantics of each type of pruning are not yet
   * given here.
   */
  POMDP_SOLVE_OPTS_Proj_Purge_Type proj_purge;

  /*
   *  The Monte-Carlo, Gauss-Seidel method using trajectories
   * through the belief space to lay down a grid of points that
   * we will compute the optimal value funciton for. This
   * parameter defines the lengths of the trajectories.
   */
  int mcgs_traj_length;

  /*
   *  When using the 'adjustable_epsilon' value iteration
   * variant, we need to compare solution sizes from the the
   * previous epochs to see whether or not the solutions are
   * staying relatively constant in size. To do this, we need to
   * define a past window length, as well as a tolerance on how
   * much variation in solution size we want to care about. This
   * parameter defines the tolerance on what we will consider all
   * solutions to be of the same size.
   */
  int epoch_history_window_delta;

  /*
   * What configuration file should be read.
   */
  /* Matthijs: changed from "true" to "True" as the former is a reserved keyword in C++. */
  char True[MAX_OPT_STRING_LEN];

  /*
   *  When solving using the 'adjustable_epsilon' method of value
   * iteration, we need to specify a staring and ending precision
   * as well as the increment to use for each adjustment. This is
   * the precision increment.
   */
  double epsilon_adjust_factor;

  /*
   *  If the 'fg_type' option is set to 'file', then this option
   * will define the filename of the file to read the grid from.
   * The grid is just a list of belief states.
   */
  char grid_filename[MAX_OPT_STRING_LEN];

  /*
   *  When pruning sets of value function components, we can use
   * a random set of points to help speed up the pruning process.
   * This parameter, if specified and non-zero, will define the
   * number of random points to use in this way.
   */
  int prune_init_rand_points;

  /*
   *  Independent of particular algortihms for computing one
   * iteration of value iteration are a number of variations of
   * value iteration meant to help speed up convergence. We do
   * not yet attempt to give a full description of the semantics
   * of each here.
   */
  POMDP_SOLVE_OPTS_Vi_Variation_Type vi_variation;

  /*
   *  Value iteration is iterative and thus we may want to find
   * 'finite horizon' solutions for various reasons. To make
   * pomdp-solve terminate after a fixed number of iterations
   * (aka epochs) set this value to be some positive number. By
   * default, value iteration will run for as many iterations as
   * it take to 'converge' on the infinite horizon solution.
   */
  int horizon;

  /*
   *  The pomdp-solve program is capable of keeping various
   * statistical information as it solves the problem. If you
   * want to track these stats and print them, set this flag to
   * true.
   */
  Boolean_Type stat_summary;

  /*
   *  When solving using the 'fixed_soln_size' method we need to
   * define what the maximal size of a soltuion we will tolerate.
   * This sets that limit.
   */
  double max_soln_size;

  /*
   *  Keeping 'witness points' means to track individual points
   * that have been found that gave rise to individual value
   * function components. These can often be used to help speed
   * up the solution process.
   */
  Boolean_Type use_witness_points;

}; /* end PomdpSolveProgOptionsProgOptionStruct */

/*
 * Default values for parameters.
 */
#define POMDP_SOLVE_OPTS_OPT_FORCE_ROUNDING_DEFAULT Boolean_false
#define POMDP_SOLVE_OPTS_OPT_MCGS_PRUNE_FREQ_DEFAULT 100
#define POMDP_SOLVE_OPTS_OPT_INC_PRUNE_DEFAULT POMDP_SOLVE_OPTS_Inc_Prune_normal
#define POMDP_SOLVE_OPTS_OPT_PRUNE_EPSILON_DEFAULT 1e-9
#define POMDP_SOLVE_OPTS_OPT_SAVE_ALL_DEFAULT Boolean_false
#define POMDP_SOLVE_OPTS_OPT_O_DEFAULT "solution"
#define POMDP_SOLVE_OPTS_OPT_FG_SAVE_DEFAULT Boolean_false
#define POMDP_SOLVE_OPTS_OPT_ENUM_PURGE_DEFAULT POMDP_SOLVE_OPTS_Enum_Purge_normal_prune
#define POMDP_SOLVE_OPTS_OPT_FG_TYPE_DEFAULT POMDP_SOLVE_OPTS_Fg_Type_initial
#define POMDP_SOLVE_OPTS_OPT_FG_EPSILON_DEFAULT 1e-9
#define POMDP_SOLVE_OPTS_OPT_MCGS_TRAJ_ITER_COUNT_DEFAULT 1
#define POMDP_SOLVE_OPTS_OPT_LP_EPSILON_DEFAULT 1e-9
#define POMDP_SOLVE_OPTS_OPT_DOM_CHECK_DEFAULT Boolean_true
#define POMDP_SOLVE_OPTS_OPT_STOP_DELTA_DEFAULT 1e-9
#define POMDP_SOLVE_OPTS_OPT_Q_PURGE_DEFAULT POMDP_SOLVE_OPTS_Q_Purge_normal_prune
#define POMDP_SOLVE_OPTS_OPT_MCGS_NUM_TRAJ_DEFAULT 1000
#define POMDP_SOLVE_OPTS_OPT_STOP_CRITERIA_DEFAULT POMDP_SOLVE_OPTS_Stop_Criteria_weak
#define POMDP_SOLVE_OPTS_OPT_METHOD_DEFAULT POMDP_SOLVE_OPTS_Method_incprune
#define POMDP_SOLVE_OPTS_OPT_SAVE_PENULTIMATE_DEFAULT Boolean_false
#define POMDP_SOLVE_OPTS_OPT_EPSILON_DEFAULT 1e-9
#define POMDP_SOLVE_OPTS_OPT_DISCOUNT_DEFAULT -1
#define POMDP_SOLVE_OPTS_OPT_FG_POINTS_DEFAULT 10000
#define POMDP_SOLVE_OPTS_OPT_FG_PURGE_DEFAULT POMDP_SOLVE_OPTS_Fg_Purge_normal_prune
#define POMDP_SOLVE_OPTS_OPT_FG_NONNEG_REWARDS_DEFAULT Boolean_false
#define POMDP_SOLVE_OPTS_OPT_PROJ_PURGE_DEFAULT POMDP_SOLVE_OPTS_Proj_Purge_normal_prune
#define POMDP_SOLVE_OPTS_OPT_MCGS_TRAJ_LENGTH_DEFAULT 100
#define POMDP_SOLVE_OPTS_OPT_VI_VARIATION_DEFAULT POMDP_SOLVE_OPTS_Vi_Variation_normal
#define POMDP_SOLVE_OPTS_OPT_STAT_SUMMARY_DEFAULT Boolean_false
#define POMDP_SOLVE_OPTS_OPT_WITNESS_POINTS_DEFAULT Boolean_false

/*
 * Strings for config file parameters.
 */
#define POMDP_SOLVE_OPTS_CFG_TIME_LIMIT_STR "time_limit"
#define POMDP_SOLVE_OPTS_CFG_FORCE_ROUNDING_STR "force_rounding"
#define POMDP_SOLVE_OPTS_CFG_MCGS_PRUNE_FREQ_STR "mcgs_prune_freq"
#define POMDP_SOLVE_OPTS_CFG_VERBOSE_STR "verbose"
#define POMDP_SOLVE_OPTS_CFG_STDOUT_STR "stdout"
#define POMDP_SOLVE_OPTS_CFG_INC_PRUNE_STR "inc_prune"
#define POMDP_SOLVE_OPTS_CFG_HISTORY_LENGTH_STR "history_length"
#define POMDP_SOLVE_OPTS_CFG_PRUNE_EPSILON_STR "prune_epsilon"
#define POMDP_SOLVE_OPTS_CFG_SAVE_ALL_STR "save_all"
#define POMDP_SOLVE_OPTS_CFG_O_STR "o"
#define POMDP_SOLVE_OPTS_CFG_FG_SAVE_STR "fg_save"
#define POMDP_SOLVE_OPTS_CFG_ENUM_PURGE_STR "enum_purge"
#define POMDP_SOLVE_OPTS_CFG_FG_TYPE_STR "fg_type"
#define POMDP_SOLVE_OPTS_CFG_FG_EPSILON_STR "fg_epsilon"
#define POMDP_SOLVE_OPTS_CFG_MCGS_TRAJ_ITER_COUNT_STR "mcgs_traj_iter_count"
#define POMDP_SOLVE_OPTS_CFG_LP_EPSILON_STR "lp_epsilon"
#define POMDP_SOLVE_OPTS_CFG_END_EPSILON_STR "end_epsilon"
#define POMDP_SOLVE_OPTS_CFG_START_EPSILON_STR "start_epsilon"
#define POMDP_SOLVE_OPTS_CFG_DOM_CHECK_STR "dom_check"
#define POMDP_SOLVE_OPTS_CFG_STOP_DELTA_STR "stop_delta"
#define POMDP_SOLVE_OPTS_CFG_Q_PURGE_STR "q_purge"
#define POMDP_SOLVE_OPTS_CFG_POMDP_STR "pomdp"
#define POMDP_SOLVE_OPTS_CFG_MCGS_NUM_TRAJ_STR "mcgs_num_traj"
#define POMDP_SOLVE_OPTS_CFG_STOP_CRITERIA_STR "stop_criteria"
#define POMDP_SOLVE_OPTS_CFG_METHOD_STR "method"
#define POMDP_SOLVE_OPTS_CFG_MEMORY_LIMIT_STR "memory_limit"
#define POMDP_SOLVE_OPTS_CFG_ALG_RAND_STR "alg_rand"
#define POMDP_SOLVE_OPTS_CFG_TERMINAL_VALUES_STR "terminal_values"
#define POMDP_SOLVE_OPTS_CFG_SAVE_PENULTIMATE_STR "save_penultimate"
#define POMDP_SOLVE_OPTS_CFG_EPSILON_STR "epsilon"
#define POMDP_SOLVE_OPTS_CFG_RAND_SEED_STR "rand_seed"
#define POMDP_SOLVE_OPTS_CFG_DISCOUNT_STR "discount"
#define POMDP_SOLVE_OPTS_CFG_FG_POINTS_STR "fg_points"
#define POMDP_SOLVE_OPTS_CFG_FG_PURGE_STR "fg_purge"
#define POMDP_SOLVE_OPTS_CFG_FG_NONNEG_REWARDS_STR "fg_nonneg_rewards"
#define POMDP_SOLVE_OPTS_CFG_PROJ_PURGE_STR "proj_purge"
#define POMDP_SOLVE_OPTS_CFG_MCGS_TRAJ_LENGTH_STR "mcgs_traj_length"
#define POMDP_SOLVE_OPTS_CFG_HISTORY_DELTA_STR "history_delta"
#define POMDP_SOLVE_OPTS_CFG_F_STR "f"
#define POMDP_SOLVE_OPTS_CFG_EPSILON_ADJUST_STR "epsilon_adjust"
#define POMDP_SOLVE_OPTS_CFG_GRID_FILENAME_STR "grid_filename"
#define POMDP_SOLVE_OPTS_CFG_PRUNE_RAND_STR "prune_rand"
#define POMDP_SOLVE_OPTS_CFG_VI_VARIATION_STR "vi_variation"
#define POMDP_SOLVE_OPTS_CFG_HORIZON_STR "horizon"
#define POMDP_SOLVE_OPTS_CFG_STAT_SUMMARY_STR "stat_summary"
#define POMDP_SOLVE_OPTS_CFG_MAX_SOLN_SIZE_STR "max_soln_size"
#define POMDP_SOLVE_OPTS_CFG_WITNESS_POINTS_STR "witness_points"

/*
 * Strings for cmd line parameters.
 */
#define POMDP_SOLVE_OPTS_ARG_TIME_LIMIT_STR "-time_limit"
#define POMDP_SOLVE_OPTS_ARG_FORCE_ROUNDING_STR "-force_rounding"
#define POMDP_SOLVE_OPTS_ARG_MCGS_PRUNE_FREQ_STR "-mcgs_prune_freq"
#define POMDP_SOLVE_OPTS_ARG_VERBOSE_STR "-verbose"
#define POMDP_SOLVE_OPTS_ARG_STDOUT_STR "-stdout"
#define POMDP_SOLVE_OPTS_ARG_INC_PRUNE_STR "-inc_prune"
#define POMDP_SOLVE_OPTS_ARG_HISTORY_LENGTH_STR "-history_length"
#define POMDP_SOLVE_OPTS_ARG_PRUNE_EPSILON_STR "-prune_epsilon"
#define POMDP_SOLVE_OPTS_ARG_SAVE_ALL_STR "-save_all"
#define POMDP_SOLVE_OPTS_ARG_O_STR "-o"
#define POMDP_SOLVE_OPTS_ARG_FG_SAVE_STR "-fg_save"
#define POMDP_SOLVE_OPTS_ARG_ENUM_PURGE_STR "-enum_purge"
#define POMDP_SOLVE_OPTS_ARG_FG_TYPE_STR "-fg_type"
#define POMDP_SOLVE_OPTS_ARG_FG_EPSILON_STR "-fg_epsilon"
#define POMDP_SOLVE_OPTS_ARG_MCGS_TRAJ_ITER_COUNT_STR "-mcgs_traj_iter_count"
#define POMDP_SOLVE_OPTS_ARG_LP_EPSILON_STR "-lp_epsilon"
#define POMDP_SOLVE_OPTS_ARG_END_EPSILON_STR "-end_epsilon"
#define POMDP_SOLVE_OPTS_ARG_START_EPSILON_STR "-start_epsilon"
#define POMDP_SOLVE_OPTS_ARG_DOM_CHECK_STR "-dom_check"
#define POMDP_SOLVE_OPTS_ARG_STOP_DELTA_STR "-stop_delta"
#define POMDP_SOLVE_OPTS_ARG_Q_PURGE_STR "-q_purge"
#define POMDP_SOLVE_OPTS_ARG_POMDP_STR "-pomdp"
#define POMDP_SOLVE_OPTS_ARG_MCGS_NUM_TRAJ_STR "-mcgs_num_traj"
#define POMDP_SOLVE_OPTS_ARG_STOP_CRITERIA_STR "-stop_criteria"
#define POMDP_SOLVE_OPTS_ARG_METHOD_STR "-method"
#define POMDP_SOLVE_OPTS_ARG_MEMORY_LIMIT_STR "-memory_limit"
#define POMDP_SOLVE_OPTS_ARG_ALG_RAND_STR "-alg_rand"
#define POMDP_SOLVE_OPTS_ARG_TERMINAL_VALUES_STR "-terminal_values"
#define POMDP_SOLVE_OPTS_ARG_SAVE_PENULTIMATE_STR "-save_penultimate"
#define POMDP_SOLVE_OPTS_ARG_EPSILON_STR "-epsilon"
#define POMDP_SOLVE_OPTS_ARG_RAND_SEED_STR "-rand_seed"
#define POMDP_SOLVE_OPTS_ARG_DISCOUNT_STR "-discount"
#define POMDP_SOLVE_OPTS_ARG_FG_POINTS_STR "-fg_points"
#define POMDP_SOLVE_OPTS_ARG_FG_PURGE_STR "-fg_purge"
#define POMDP_SOLVE_OPTS_ARG_FG_NONNEG_REWARDS_STR "-fg_nonneg_rewards"
#define POMDP_SOLVE_OPTS_ARG_PROJ_PURGE_STR "-proj_purge"
#define POMDP_SOLVE_OPTS_ARG_MCGS_TRAJ_LENGTH_STR "-mcgs_traj_length"
#define POMDP_SOLVE_OPTS_ARG_HISTORY_DELTA_STR "-history_delta"
#define POMDP_SOLVE_OPTS_ARG_F_STR "-f"
#define POMDP_SOLVE_OPTS_ARG_EPSILON_ADJUST_STR "-epsilon_adjust"
#define POMDP_SOLVE_OPTS_ARG_GRID_FILENAME_STR "-grid_filename"
#define POMDP_SOLVE_OPTS_ARG_PRUNE_RAND_STR "-prune_rand"
#define POMDP_SOLVE_OPTS_ARG_VI_VARIATION_STR "-vi_variation"
#define POMDP_SOLVE_OPTS_ARG_HORIZON_STR "-horizon"
#define POMDP_SOLVE_OPTS_ARG_STAT_SUMMARY_STR "-stat_summary"
#define POMDP_SOLVE_OPTS_ARG_MAX_SOLN_SIZE_STR "-max_soln_size"
#define POMDP_SOLVE_OPTS_ARG_WITNESS_POINTS_STR "-witness_points"

/*
 * String arrays for cmd line parameters.
 */

extern char* POMDP_SOLVE_OPTS_Verbose_Str[];

extern char* POMDP_SOLVE_OPTS_Inc_Prune_Str[];

extern char* POMDP_SOLVE_OPTS_Enum_Purge_Str[];

extern char* POMDP_SOLVE_OPTS_Fg_Type_Str[];

extern char* POMDP_SOLVE_OPTS_Q_Purge_Str[];

extern char* POMDP_SOLVE_OPTS_Stop_Criteria_Str[];

extern char* POMDP_SOLVE_OPTS_Method_Str[];

extern char* POMDP_SOLVE_OPTS_Fg_Purge_Str[];

extern char* POMDP_SOLVE_OPTS_Proj_Purge_Str[];

extern char* POMDP_SOLVE_OPTS_Vi_Variation_Str[];

/*
 * Function prototyeps.
 */

extern PomdpSolveProgOptions
POMDP_SOLVE_OPTS_new( );

extern void
POMDP_SOLVE_OPTS_delete( PomdpSolveProgOptions );

extern ConfigFile
POMDP_SOLVE_OPTS_toConfigFile( PomdpSolveProgOptions );

extern void 
POMDP_SOLVE_OPTS_showUsageBrief( FILE*, char* );

extern void 
POMDP_SOLVE_OPTS_showUsage( FILE*, char* );

extern PomdpSolveProgOptions
POMDP_SOLVE_OPTS_parse( ProgramOptions );

extern PomdpSolveProgOptions
POMDP_SOLVE_OPTS_create( int, char** );

#endif
/* end header file */
