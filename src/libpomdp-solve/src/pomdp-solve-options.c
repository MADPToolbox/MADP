
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    pomdp-solve-options.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    February 2005
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: pomdp-solve-options.c,v $
 *    $Source: pomdp-solve-options.c,v $
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "pomdp-solve-options.h"

/*
 * Strings arrays for parameters.
 */

char* POMDP_SOLVE_OPTS_Verbose_Str[] = POMDP_SOLVE_OPTS_OPT_VERBOSE_STRINGS;

char* POMDP_SOLVE_OPTS_Inc_Prune_Str[] = POMDP_SOLVE_OPTS_OPT_INC_PRUNE_STRINGS;

char* POMDP_SOLVE_OPTS_Enum_Purge_Str[] = POMDP_SOLVE_OPTS_OPT_ENUM_PURGE_STRINGS;

char* POMDP_SOLVE_OPTS_Fg_Type_Str[] = POMDP_SOLVE_OPTS_OPT_FG_TYPE_STRINGS;

char* POMDP_SOLVE_OPTS_Q_Purge_Str[] = POMDP_SOLVE_OPTS_OPT_Q_PURGE_STRINGS;

char* POMDP_SOLVE_OPTS_Stop_Criteria_Str[] = POMDP_SOLVE_OPTS_OPT_STOP_CRITERIA_STRINGS;

char* POMDP_SOLVE_OPTS_Method_Str[] = POMDP_SOLVE_OPTS_OPT_METHOD_STRINGS;

char* POMDP_SOLVE_OPTS_Fg_Purge_Str[] = POMDP_SOLVE_OPTS_OPT_FG_PURGE_STRINGS;

char* POMDP_SOLVE_OPTS_Proj_Purge_Str[] = POMDP_SOLVE_OPTS_OPT_PROJ_PURGE_STRINGS;

char* POMDP_SOLVE_OPTS_Vi_Variation_Str[] = POMDP_SOLVE_OPTS_OPT_VI_VARIATION_STRINGS;

/*******************************************************/
PomdpSolveProgOptions
POMDP_SOLVE_OPTS_new( )
{

  PomdpSolveProgOptions options;

  options = (PomdpSolveProgOptions) XMALLOC( sizeof( *options ));

  strcpy( options->__exec_name__, "pomdp-solve" );

  strcpy( options->__version__, "5.3" );

  options->__error__ = 0;

  options->max_secs = 0;
  options->force_rounding = POMDP_SOLVE_OPTS_OPT_FORCE_ROUNDING_DEFAULT;
  options->mcgs_prune_freq = POMDP_SOLVE_OPTS_OPT_MCGS_PRUNE_FREQ_DEFAULT;
  options->verbose = 0;
  options->report_filename[0] = '\0';
  options->ip_type = POMDP_SOLVE_OPTS_OPT_INC_PRUNE_DEFAULT;
  options->epoch_history_window_length = 0;
  options->prune_epsilon = POMDP_SOLVE_OPTS_OPT_PRUNE_EPSILON_DEFAULT;
  options->save_all = POMDP_SOLVE_OPTS_OPT_SAVE_ALL_DEFAULT;
  strcpy( options->prefix_str, POMDP_SOLVE_OPTS_OPT_O_DEFAULT );
  options->finite_grid_save = POMDP_SOLVE_OPTS_OPT_FG_SAVE_DEFAULT;
  options->enum_purge_option = POMDP_SOLVE_OPTS_OPT_ENUM_PURGE_DEFAULT;
  options->finite_grid_type = POMDP_SOLVE_OPTS_OPT_FG_TYPE_DEFAULT;
  options->fg_epsilon = POMDP_SOLVE_OPTS_OPT_FG_EPSILON_DEFAULT;
  options->mcgs_traj_iter_count = POMDP_SOLVE_OPTS_OPT_MCGS_TRAJ_ITER_COUNT_DEFAULT;
  options->lp_epsilon = POMDP_SOLVE_OPTS_OPT_LP_EPSILON_DEFAULT;
  options->ending_epsilon = 0.0;
  options->starting_epsilon = 0.0;
  options->domination_check = POMDP_SOLVE_OPTS_OPT_DOM_CHECK_DEFAULT;
  options->stop_delta = POMDP_SOLVE_OPTS_OPT_STOP_DELTA_DEFAULT;
  options->q_purge_option = POMDP_SOLVE_OPTS_OPT_Q_PURGE_DEFAULT;
  options->param_filename[0] = '\0';
  options->mcgs_num_traj = POMDP_SOLVE_OPTS_OPT_MCGS_NUM_TRAJ_DEFAULT;
  options->stop_criteria = POMDP_SOLVE_OPTS_OPT_STOP_CRITERIA_DEFAULT;
  options->method = POMDP_SOLVE_OPTS_OPT_METHOD_DEFAULT;
  options->memory_limit = 0;
  options->alg_init_rand_points = 0;
  options->initial_policy_filename[0] = '\0';
  options->save_penultimate = POMDP_SOLVE_OPTS_OPT_SAVE_PENULTIMATE_DEFAULT;
  options->epsilon = POMDP_SOLVE_OPTS_OPT_EPSILON_DEFAULT;
  options->rand_seed[0] = '\0';
  options->override_discount = POMDP_SOLVE_OPTS_OPT_DISCOUNT_DEFAULT;
  options->finite_grid_points = POMDP_SOLVE_OPTS_OPT_FG_POINTS_DEFAULT;
  options->fg_purge_option = POMDP_SOLVE_OPTS_OPT_FG_PURGE_DEFAULT;
  options->fg_nonneg_rewards = POMDP_SOLVE_OPTS_OPT_FG_NONNEG_REWARDS_DEFAULT;
  options->proj_purge = POMDP_SOLVE_OPTS_OPT_PROJ_PURGE_DEFAULT;
  options->mcgs_traj_length = POMDP_SOLVE_OPTS_OPT_MCGS_TRAJ_LENGTH_DEFAULT;
  options->epoch_history_window_delta = 0;
  options->True[0] = '\0';
  options->epsilon_adjust_factor = 0.0;
  options->grid_filename[0] = '\0';
  options->prune_init_rand_points = 0;
  options->vi_variation = POMDP_SOLVE_OPTS_OPT_VI_VARIATION_DEFAULT;
  options->horizon = 0;
  options->stat_summary = POMDP_SOLVE_OPTS_OPT_STAT_SUMMARY_DEFAULT;
  options->max_soln_size = 0.0;
  options->use_witness_points = POMDP_SOLVE_OPTS_OPT_WITNESS_POINTS_DEFAULT;

  return( options );

}  /* POMDP_SOLVE_OPTS_new */

/*******************************************************/
void
POMDP_SOLVE_OPTS_delete( PomdpSolveProgOptions options )
{

  XFREE( options );

}  /* POMDP_SOLVE_OPTS_delete */

/*******************************************************/
ConfigFile
POMDP_SOLVE_OPTS_toConfigFile( PomdpSolveProgOptions options )
{
  ConfigFile cfg;
  char str[256];

  cfg = CF_new();

  sprintf( str, "%d", options->max_secs );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_TIME_LIMIT_STR, str );

  sprintf( str, "%s", Boolean_Str[options->force_rounding] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FORCE_ROUNDING_STR, str );

  sprintf( str, "%d", options->mcgs_prune_freq );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_MCGS_PRUNE_FREQ_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Verbose_Str[options->verbose] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_VERBOSE_STR, str );

  sprintf( str, "%s", options->report_filename );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_STDOUT_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Inc_Prune_Str[options->ip_type] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_INC_PRUNE_STR, str );

  sprintf( str, "%d", options->epoch_history_window_length );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_HISTORY_LENGTH_STR, str );

  sprintf( str, "%.6f", options->prune_epsilon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_PRUNE_EPSILON_STR, str );

  sprintf( str, "%s", Boolean_Str[options->save_all] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_SAVE_ALL_STR, str );

  sprintf( str, "%s", options->prefix_str );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_O_STR, str );

  sprintf( str, "%s", Boolean_Str[options->finite_grid_save] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FG_SAVE_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Enum_Purge_Str[options->enum_purge_option] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_ENUM_PURGE_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Fg_Type_Str[options->finite_grid_type] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FG_TYPE_STR, str );

  sprintf( str, "%.6f", options->fg_epsilon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FG_EPSILON_STR, str );

  sprintf( str, "%d", options->mcgs_traj_iter_count );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_MCGS_TRAJ_ITER_COUNT_STR, str );

  sprintf( str, "%.6f", options->lp_epsilon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_LP_EPSILON_STR, str );

  sprintf( str, "%.6f", options->ending_epsilon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_END_EPSILON_STR, str );

  sprintf( str, "%.6f", options->starting_epsilon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_START_EPSILON_STR, str );

  sprintf( str, "%s", Boolean_Str[options->domination_check] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_DOM_CHECK_STR, str );

  sprintf( str, "%.6f", options->stop_delta );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_STOP_DELTA_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Q_Purge_Str[options->q_purge_option] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_Q_PURGE_STR, str );

  sprintf( str, "%s", options->param_filename );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_POMDP_STR, str );

  sprintf( str, "%d", options->mcgs_num_traj );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_MCGS_NUM_TRAJ_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Stop_Criteria_Str[options->stop_criteria] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_STOP_CRITERIA_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Method_Str[options->method] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_METHOD_STR, str );

  sprintf( str, "%d", options->memory_limit );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_MEMORY_LIMIT_STR, str );

  sprintf( str, "%d", options->alg_init_rand_points );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_ALG_RAND_STR, str );

  sprintf( str, "%s", options->initial_policy_filename );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_TERMINAL_VALUES_STR, str );

  sprintf( str, "%s", Boolean_Str[options->save_penultimate] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_SAVE_PENULTIMATE_STR, str );

  sprintf( str, "%.6f", options->epsilon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_EPSILON_STR, str );

  sprintf( str, "%s", options->rand_seed );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_RAND_SEED_STR, str );

  sprintf( str, "%.6f", options->override_discount );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_DISCOUNT_STR, str );

  sprintf( str, "%d", options->finite_grid_points );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FG_POINTS_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Fg_Purge_Str[options->fg_purge_option] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FG_PURGE_STR, str );

  sprintf( str, "%s", Boolean_Str[options->fg_nonneg_rewards] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_FG_NONNEG_REWARDS_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Proj_Purge_Str[options->proj_purge] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_PROJ_PURGE_STR, str );

  sprintf( str, "%d", options->mcgs_traj_length );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_MCGS_TRAJ_LENGTH_STR, str );

  sprintf( str, "%d", options->epoch_history_window_delta );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_HISTORY_DELTA_STR, str );

  sprintf( str, "%s", options->True );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_F_STR, str );

  sprintf( str, "%.6f", options->epsilon_adjust_factor );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_EPSILON_ADJUST_STR, str );

  sprintf( str, "%s", options->grid_filename );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_GRID_FILENAME_STR, str );

  sprintf( str, "%d", options->prune_init_rand_points );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_PRUNE_RAND_STR, str );

  sprintf( str, "%s", POMDP_SOLVE_OPTS_Vi_Variation_Str[options->vi_variation] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_VI_VARIATION_STR, str );

  sprintf( str, "%d", options->horizon );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_HORIZON_STR, str );

  sprintf( str, "%s", Boolean_Str[options->stat_summary] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_STAT_SUMMARY_STR, str );

  sprintf( str, "%.6f", options->max_soln_size );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_MAX_SOLN_SIZE_STR, str );

  sprintf( str, "%s", Boolean_Str[options->use_witness_points] );
  CF_addParam( cfg, POMDP_SOLVE_OPTS_CFG_WITNESS_POINTS_STR, str );

  return cfg;

} /* POMDP_SOLVE_OPTS_toConfigFile */

/*******************************************************/
void 
POMDP_SOLVE_OPTS_showUsageBrief( FILE* file, char* exec_name )
{
  fprintf( file, "Usage: %s [opts...] [args...]\n", exec_name );
  fprintf( file, "Use '-h' for help.\n");

}  /* POMDP_SOLVE_OPTS_showUsageBrief */

/*******************************************************/
void 
POMDP_SOLVE_OPTS_showUsage( FILE* file, char* exec_name )
{
  fprintf( file, "Usage: %s [opts...] [args...]\n", exec_name );

  /*******************************/
  /* __MAIN__ parameters  */
  /*******************************/
  fprintf( file, "General options:\n" );

  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_FORCE_ROUNDING_STR,
                     Boolean_Str );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_STDOUT_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_SAVE_PENULTIMATE_STR,
                     Boolean_Str );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_RAND_SEED_STR );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_F_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_STAT_SUMMARY_STR,
                     Boolean_Str );

  /*******************************/
  /* Resource Limits parameters  */
  /*******************************/
  fprintf( file, "Resource Limits options:\n" );

  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_TIME_LIMIT_STR );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_MEMORY_LIMIT_STR );

  /*******************************/
  /* Algorithm parameters  */
  /*******************************/
  fprintf( file, "Algorithm options:\n" );

  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_MCGS_PRUNE_FREQ_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_INC_PRUNE_STR,
                     POMDP_SOLVE_OPTS_Inc_Prune_Str );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_FG_SAVE_STR,
                     Boolean_Str );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_ENUM_PURGE_STR,
                     POMDP_SOLVE_OPTS_Enum_Purge_Str );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_FG_TYPE_STR,
                     POMDP_SOLVE_OPTS_Fg_Type_Str );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_MCGS_TRAJ_ITER_COUNT_STR );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_MCGS_NUM_TRAJ_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_METHOD_STR,
                     POMDP_SOLVE_OPTS_Method_Str );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_FG_POINTS_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_FG_PURGE_STR,
                     POMDP_SOLVE_OPTS_Fg_Purge_Str );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_FG_NONNEG_REWARDS_STR,
                     Boolean_Str );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_MCGS_TRAJ_LENGTH_STR );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_GRID_FILENAME_STR );

  /*******************************/
  /* Optimization parameters  */
  /*******************************/
  fprintf( file, "Optimization options:\n" );

  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_PRUNE_EPSILON_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_FG_EPSILON_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_LP_EPSILON_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_DOM_CHECK_STR,
                     Boolean_Str );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_Q_PURGE_STR,
                     POMDP_SOLVE_OPTS_Q_Purge_Str );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_ALG_RAND_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_EPSILON_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_PROJ_PURGE_STR,
                     POMDP_SOLVE_OPTS_Proj_Purge_Str );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_PRUNE_RAND_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_WITNESS_POINTS_STR,
                     Boolean_Str );

  /*******************************/
  /* Debug parameters  */
  /*******************************/
  fprintf( file, "Debug options:\n" );

  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_VERBOSE_STR,
                     POMDP_SOLVE_OPTS_Verbose_Str );

  /*******************************/
  /* Value Iteration parameters  */
  /*******************************/
  fprintf( file, "Value Iteration options:\n" );

  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_HISTORY_LENGTH_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_SAVE_ALL_STR,
                     Boolean_Str );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_O_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_END_EPSILON_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_START_EPSILON_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_STOP_DELTA_STR );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_POMDP_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_STOP_CRITERIA_STR,
                     POMDP_SOLVE_OPTS_Stop_Criteria_Str );
  fprintf( file, "\t%s <string>\n", POMDP_SOLVE_OPTS_ARG_TERMINAL_VALUES_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_DISCOUNT_STR );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_HISTORY_DELTA_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_EPSILON_ADJUST_STR );
  PO_showUsageEnumType( file,
                     POMDP_SOLVE_OPTS_ARG_VI_VARIATION_STR,
                     POMDP_SOLVE_OPTS_Vi_Variation_Str );
  fprintf( file, "\t%s <int>\n", POMDP_SOLVE_OPTS_ARG_HORIZON_STR );
  fprintf( file, "\t%s <double>\n", POMDP_SOLVE_OPTS_ARG_MAX_SOLN_SIZE_STR );

}  /* POMDP_SOLVE_OPTS_showUsage */

/*******************************************************/
PomdpSolveProgOptions
POMDP_SOLVE_OPTS_parse( ProgramOptions opts )
{
  PomdpSolveProgOptions options;
  int enum_idx;
  int ret_value;

  options = POMDP_SOLVE_OPTS_new();

  PO_startValidate( opts );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_TIME_LIMIT_STR,
                            &(options->max_secs),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'max_secs' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_FORCE_ROUNDING_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->force_rounding = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'force_rounding' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_MCGS_PRUNE_FREQ_STR,
                            &(options->mcgs_prune_freq),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'mcgs_prune_freq' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_VERBOSE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Verbose_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->verbose = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'verbose' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_STDOUT_STR,
                         options->report_filename,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'report_filename' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_INC_PRUNE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Inc_Prune_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->ip_type = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'ip_type' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_HISTORY_LENGTH_STR,
                            &(options->epoch_history_window_length),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'epoch_history_window_length' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_PRUNE_EPSILON_STR,
               &(options->prune_epsilon),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'prune_epsilon' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_SAVE_ALL_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->save_all = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'save_all' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_O_STR,
                         options->prefix_str,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'prefix_str' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_FG_SAVE_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->finite_grid_save = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'finite_grid_save' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_ENUM_PURGE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Enum_Purge_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->enum_purge_option = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'enum_purge_option' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_FG_TYPE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Fg_Type_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->finite_grid_type = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'finite_grid_type' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_FG_EPSILON_STR,
               &(options->fg_epsilon),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'fg_epsilon' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_MCGS_TRAJ_ITER_COUNT_STR,
                            &(options->mcgs_traj_iter_count),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'mcgs_traj_iter_count' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_LP_EPSILON_STR,
               &(options->lp_epsilon),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'lp_epsilon' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_END_EPSILON_STR,
               &(options->ending_epsilon),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'ending_epsilon' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_START_EPSILON_STR,
               &(options->starting_epsilon),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'starting_epsilon' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_DOM_CHECK_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->domination_check = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'domination_check' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_STOP_DELTA_STR,
               &(options->stop_delta),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'stop_delta' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_Q_PURGE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Q_Purge_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->q_purge_option = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'q_purge_option' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_POMDP_STR,
                         options->param_filename,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'param_filename' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_MCGS_NUM_TRAJ_STR,
                            &(options->mcgs_num_traj),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'mcgs_num_traj' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_STOP_CRITERIA_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Stop_Criteria_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->stop_criteria = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'stop_criteria' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_METHOD_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Method_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->method = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'method' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_MEMORY_LIMIT_STR,
                            &(options->memory_limit),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'memory_limit' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_ALG_RAND_STR,
                            &(options->alg_init_rand_points),
                           0,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'alg_init_rand_points' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_TERMINAL_VALUES_STR,
                         options->initial_policy_filename,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'initial_policy_filename' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_SAVE_PENULTIMATE_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->save_penultimate = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'save_penultimate' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_EPSILON_STR,
               &(options->epsilon),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'epsilon' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_RAND_SEED_STR,
                         options->rand_seed,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'rand_seed' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_DISCOUNT_STR,
               &(options->override_discount),
                           0,
                           1 );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'override_discount' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_FG_POINTS_STR,
                            &(options->finite_grid_points),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'finite_grid_points' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_FG_PURGE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Fg_Purge_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->fg_purge_option = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'fg_purge_option' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_FG_NONNEG_REWARDS_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->fg_nonneg_rewards = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'fg_nonneg_rewards' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_PROJ_PURGE_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Proj_Purge_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->proj_purge = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'proj_purge' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_MCGS_TRAJ_LENGTH_STR,
                            &(options->mcgs_traj_length),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'mcgs_traj_length' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_HISTORY_DELTA_STR,
                            &(options->epoch_history_window_delta),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'epoch_history_window_delta' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_F_STR,
                         options->True,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'true' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_EPSILON_ADJUST_STR,
               &(options->epsilon_adjust_factor),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'epsilon_adjust_factor' has invalid value." );

  ret_value = PO_getStringOption( opts,
                         POMDP_SOLVE_OPTS_ARG_GRID_FILENAME_STR,
                         options->grid_filename,
                         NULL,
                         NULL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'grid_filename' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_PRUNE_RAND_STR,
                            &(options->prune_init_rand_points),
                           0,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'prune_init_rand_points' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_VI_VARIATION_STR,
                         &(enum_idx),
                         POMDP_SOLVE_OPTS_Vi_Variation_Str );
  if ( ret_value == PO_OPT_PRESENT_VALID )
    options->vi_variation = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'vi_variation' has invalid value." );

  ret_value = PO_getIntegerOption( opts,
                           POMDP_SOLVE_OPTS_ARG_HORIZON_STR,
                            &(options->horizon),
                           1,
                           INT_MAX );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'horizon' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_STAT_SUMMARY_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->stat_summary = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'stat_summary' has invalid value." );

  ret_value = PO_getDoubleOption( opts,
                            POMDP_SOLVE_OPTS_ARG_MAX_SOLN_SIZE_STR,
               &(options->max_soln_size),
                           0,
                           HUGE_VAL );
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'max_soln_size' has invalid value." );

  ret_value = PO_getEnumOption( opts,
                         POMDP_SOLVE_OPTS_ARG_WITNESS_POINTS_STR,
                         &(enum_idx),
                         Boolean_Str );
  options->use_witness_points = enum_idx;
  if ( ret_value == PO_OPT_PRESENT_ERROR )
    PO_handleError( opts, "Option 'use_witness_points' has invalid value." );

  PO_endValidate( opts );

  if( ! PO_isValid( opts ))
    options->__error__ = 1;
  return options;

}  /* POMDP_SOLVE_OPTS_parse */

/*******************************************************/
PomdpSolveProgOptions
POMDP_SOLVE_OPTS_create( int argc, char** argv )
{
  PomdpSolveProgOptions options;
  ProgramOptions opts;
  ConfigFile cfg;

  opts = PO_create( argc, argv );

  if ( opts->usage )
    {
      POMDP_SOLVE_OPTS_showUsage( stdout,
          opts->cmd_line->exec_name );
      PO_delete( opts );
      exit( 1 );
    }

  if ( ! PO_isValid( opts ))
    {
      POMDP_SOLVE_OPTS_showUsageBrief( stdout, 
          opts->cmd_line->exec_name );
      PO_delete( opts );
      exit( 1 );
    }

  options = POMDP_SOLVE_OPTS_parse( opts );

  if ( options->__error__ )
    {
      POMDP_SOLVE_OPTS_showUsageBrief( stdout, 
          opts->cmd_line->exec_name );
      PO_delete( opts );

      POMDP_SOLVE_OPTS_delete( options );
      exit( 1 );
    }

  PO_delete( opts );

  return options;

} /* POMDP_SOLVE_OPTS_create */

/* end C code */
