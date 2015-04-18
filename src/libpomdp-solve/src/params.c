/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    params.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: params.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/params.c,v $
 *    $Revision: 1.10 $
 *    $Date: 2004/11/01 04:27:34 $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    1994-1997, Brown University
 *    1998-2003 Anthony R. Cassandra
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
 *   Stuff to specify all POMDP solution parameters.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mdp/mdp.h"

#include "global.h"
#include "timing.h"
#include "random.h"
#include "pomdp.h"
#include "alpha.h"
#include "stats.h"
#include "cmd-line.h"
#include "lp-interface.h"
#include "params.h"

/* Strings for the various stopping criteria */
char *purge_option_str[] = PURGE_OPTION_STRINGS;

/* Strings for the various incremental pruning variations. */
char *inc_prune_type_str[] = INC_PRUNE_TYPE_STRINGS;

/* Strings for the various value iterations variations. */
char *vi_variation_type_str[] = VI_VARIATION_TYPE_STRINGS;

/**********************************************************************/
PomdpSolveParams 
newPomdpSolveParams(  ) 
{
  /*
    Creates the memory for the structure to hold the parameters used in
    solving a POMDP.  Also sets the fields to the default values.
  */
  PomdpSolveParams params;
  int i;

  params = (PomdpSolveParams) XMALLOC( sizeof( *params ));

  /* We do not allocate the memory for this structure.  We just ensure
	it starts with a sane NULL value.  We will clean this up if it is
	non-null in the destructor.
  */

  params->opts = NULL;

  params->cur_epoch = 0;
  params->update_count = 0;
  params->stat_summary = FALSE;
  params->report_filename[0] = NULL_CHAR;
  params->report_file = stdout;
  params->param_filename[0] = NULL_CHAR;
  params->override_discount = -1.0;
  params->horizon = DEFAULT_HORIZON;
  params->alpha_filename[0] = NULL_CHAR;
  params->pg_filename[0] = NULL_CHAR;
  params->initial_policy_filename[0] = NULL_CHAR;
  params->initial_policy = NULL;
  params->max_secs = 0;
  params->memory_limit = 0;
  params->save_all = FALSE;
  params->proj_purge = DEFAULT_PROJECTION_PURGE;
  params->use_witness_points = DEFAULT_USE_WITNESS_POINTS;
  params->backup_file[0] = NULL_CHAR;
  params->penultimate_filename[0] = NULL_CHAR;
  params->q_purge_option = DEFAULT_Q_PURGE_OPTION;
  params->domination_check = DEFAULT_DOMINATION_CHECK;
  params->alg_init_rand_points = DEFAULT_ALG_INIT_RAND_POINTS;
  params->prune_init_rand_points = DEFAULT_PRUNE_INIT_RAND_POINTS;

  params->prune_epsilon = DEFAULT_PRUNE_EPSILON; 
  params->epsilon = DEFAULT_EPSILON; 
  params->lp_epsilon = DEFAULT_LP_EPSILON; 

  params->stop_delta = DEFAULT_STOP_DELTA;

  params->alpha_epsilon = DEFAULT_ALPHA_EPSILON; 
  params->vertex_epsilon = DEFAULT_VERTEX_EPSILON; 
  params->impossible_obs_epsilon = DEFAULT_IMPOSSIBLE_OBS_EPSILON;
  params->double_equality_precision = DEFAULT_DOUBLE_EQUALITY_PRECISION;
  
  /* Default value to use when considering whether to include a
     coefficient in a sparse representation. Note that we don't really
     want to tie this to the precision that is being used to solve the
     problem, because this value can change the problem being solved.
     Thus this should just be fixed for all time at the minimum
     precision. */
  params->sparse_epsilon = SMALLEST_PRECISION; 

  /* Place to hang statistics off of (optional) */
  params->stat = NULL;

  /****************************************/
  /****  Algorithm specific section  ******/
  /****************************************/

  params->ip_type = DEFAULT_INC_PRUNE_TYPE;
  params->enum_purge_option = DEFAULT_ENUM_PURGE_OPTION;
  params->fg_purge_option = DEFAULT_FG_PURGE_OPTION;

  /****************************************/
  /****  VI variation specific section  ***/
  /****************************************/

  params->vi_variation = DEFAULT_VI_VARIATION;
  params->starting_epsilon = DEFAULT_STARTING_EPSILON;
  params->ending_epsilon = DEFAULT_ENDING_EPSILON;
  params->epsilon_adjust_factor = DEFAULT_EPSILON_ADJUST_FACTOR;
  params->max_soln_size = DEFAULT_MAX_SOLN_SIZE;
  params->epoch_history_window_length = DEFAULT_HISTORY_WINDOW_LENGTH; 
  params->epoch_history_window_delta = DEFAULT_HISTORY_WINDO_DELTA;

  return ( params );

}  /* newPomdpSolveParams */
/**********************************************************************/
void 
destroyPomdpSolveParams( PomdpSolveParams params ) 
{

  if ( params->opts != NULL )
    POMDP_SOLVE_OPTS_delete( params->opts );
   
  /* 
     Frees the memory for pointers in the params and the param structure
     itself.
  */
  if ( params->stat != NULL )
    destroySolutionStats( params->stat );

  if ( params->initial_policy != NULL )
    destroyAlphaList( params->initial_policy );

  XFREE( params );

}  /* destroyPomdpSolveParams */
/**********************************************************************/
void 
parseMethodAliases( int argc, char **argv, int *mark_arg,
		    PomdpSolveParams param ) 
{

  /* zzz Add aliases here and see obsolete/old-parse-alias.c */

}  /* methodAliases */
/**********************************************************************/
void 
enforceSmallestPrecision( double *value, char *name ) 
{
  /*
    Takes a value and makes sure it is not less than the smallest
    allowable precision the program uses.  It will give a message if
    it needs to be changed.  
  */
  char msg[MAX_MSG_LENGTH];

  if ( *value >= SMALLEST_PRECISION )
    return;
  
  *value = SMALLEST_PRECISION;
  
  sprintf( msg, 
           "The value for %s is below the smallest precision.\n\tSetting to %.3e." ,
           name, *value );

  Warning( msg )

}  /* enforceSmallestPrecision */

/**********************************************************************/
void
doPreOptionParseActions( ) {
  /*
    The very first routine that is called. Put anything that needs to
    happen before parsing the command line in this routine. 
   */

  /****************/
  /* I used to initialize this variable during its declaration, but I
	found that when updating to version 4.1, this was no longer
	allowed. Thus, I do it here first thing. */
  gStdErrFile = stderr;

} /* doPreOptionParseActions */

/**********************************************************************/
void
doPostOptionParseActions( PomdpSolveParams params ) {
  /*
    Does more customized checking of the options to the program.
  */
  PomdpSolveProgOptions opts;
  char tmp_str[MAX_OPT_STRING_LEN];
  int idx;

  /* just for convenience within this routine */
  opts = params->opts;

  /****************/
   /* First see if a random number seed is given, and if it is set the
      seed to this value. */
  if( opts->rand_seed[0] != '\0' )
    setRandomSeedFromString( opts->rand_seed );

  /* Otherwise initialize the random number generator with
	psuedo-random seed. */
  else
    randomize();

  /****************/
  /* Set if we want to redirect everything to a file.  Note that we
	must do this early on and actually open the file here because we
	might shortly get error messages that will need to be printed
	out to the file. */
  if( opts->report_filename[0] != '\0' ) {
     
    if (( params->report_file 
		= fopen( opts->report_filename , "w")) == NULL) {
	 params->report_file = stdout;
	 fprintf( gStdErrFile, 
			"** Error: Cannot write to output file %s.\n",
			opts->report_filename );
	 fprintf( gStdErrFile, 
			"\tUsing stdout instead.\n" );
    }  /* if can't open report file */
	
    /* If they desire to put all the output into a specific file,
	  then we will also output all stderr messages here as well. */
    gStdErrFile = params->report_file;

  } /* if redirecting output to a file */

  /****************/
  /* Try to make the prefix be the prefix of the POMDP file if the
	default is chosen. */
  if ( strcmp( opts->prefix_str, POMDP_SOLVE_OPTS_OPT_O_DEFAULT ) == 0 ) {

    strcpy( tmp_str, opts->param_filename );

    /* This will point to null term at first */
    idx = strlen( tmp_str );  

    /* Start at the end and move left until we see the first
	  "period". */
    while (( idx > 0 ) && ( tmp_str[idx] != '.'))
	 idx--;

    /* Only override if we found a period in param filename */
    if ( idx > 0 ) {

	 /* Null terminate at the period */
	 tmp_str[idx] = '\0';

	 sprintf( opts->prefix_str, "%s-%d", tmp_str, getPid() );
	 
    } /* if we can override the default */

  } /* if default prefix is being used */

  
    /* Start at the end of the param_filename string

  /****************/
  /* Make sure nothing dips below some maximum precision setting. */
  
  enforceSmallestPrecision( &(opts->prune_epsilon),
                            POMDP_SOLVE_OPTS_ARG_PRUNE_EPSILON_STR );
  enforceSmallestPrecision( &(opts->epsilon),
                            POMDP_SOLVE_OPTS_ARG_EPSILON_STR );
  enforceSmallestPrecision( &(opts->lp_epsilon),
                            POMDP_SOLVE_OPTS_ARG_LP_EPSILON_STR );

  /* Don't want to have the LPs be less precise than the rest of the
     program's operations. More precise is alright, since it can
     filter things out, but less precise makes little sense. */
  if ( opts->lp_epsilon > opts->epsilon ) {
    
    Warning( "LP epsilon must be no greater than general epsilon." );
    
    opts->lp_epsilon = opts->epsilon;

  } /* if lp_epsilon greater than general epsilon */
  
  /* Set the global LP precision */
  LP_setPrecision( opts->lp_epsilon );
  
} /* doPostOptionParseActions */

/**********************************************************************/
void
tempOptsToParamConversion( PomdpSolveParams params ) {
  /*
    FIXME: Remove when migration to new configuration scheme is
    complete.

    Does temporary conversion from the new options-based configuration
    to the older data structure.
  */
  PomdpSolveProgOptions opts;

  opts = params->opts;

  params->stat_summary = opts->stat_summary;
  strcpy( params->report_filename, opts->report_filename );
  strcpy( params->param_filename, opts->param_filename );
  params->override_discount = opts->override_discount;
  params->horizon = opts->horizon;
  strcpy( params->initial_policy_filename, opts->initial_policy_filename );
  params->max_secs = opts->max_secs;
  params->memory_limit = opts->memory_limit;
  params->save_all = opts->save_all;
  params->use_witness_points = opts->use_witness_points;
  params->domination_check = opts->domination_check;
  params->alg_init_rand_points = opts->alg_init_rand_points;
  params->prune_init_rand_points = opts->prune_init_rand_points;
  params->prune_epsilon = opts->prune_epsilon;
  params->epsilon = opts->epsilon;
  params->lp_epsilon = opts->lp_epsilon;
  params->stop_delta = opts->stop_delta;
  params->starting_epsilon = opts->starting_epsilon;
  params->ending_epsilon = opts->ending_epsilon;
  params->epsilon_adjust_factor = opts->epsilon_adjust_factor;
  params->max_soln_size = opts->max_soln_size;
  params->epoch_history_window_length = opts->epoch_history_window_length;
  params->epoch_history_window_delta = opts->epoch_history_window_delta;

  switch( opts->ip_type )
    {
    case POMDP_SOLVE_OPTS_Inc_Prune_normal: params->ip_type = NormalIp; break;
    case POMDP_SOLVE_OPTS_Inc_Prune_restricted_region: params->ip_type = RestrictedRegionIp; break;
    case POMDP_SOLVE_OPTS_Inc_Prune_generalized: params->ip_type = GeneralizedIp; break;
    }

  switch( opts->proj_purge )
    {
    case POMDP_SOLVE_OPTS_Proj_Purge_none: params->proj_purge = purge_none; break;
    case POMDP_SOLVE_OPTS_Proj_Purge_domonly: params->proj_purge = purge_dom; break;
    case POMDP_SOLVE_OPTS_Proj_Purge_normal_prune: params->proj_purge = purge_prune; break;
    case POMDP_SOLVE_OPTS_Proj_Purge_epsilon_prune: params->proj_purge = purge_epsilon_prune; break;
    }
  
  switch( opts->q_purge_option )
    {
    case POMDP_SOLVE_OPTS_Q_Purge_none: params->q_purge_option = purge_none; break;
    case POMDP_SOLVE_OPTS_Q_Purge_domonly: params->q_purge_option = purge_dom; break;
    case POMDP_SOLVE_OPTS_Q_Purge_normal_prune: params->q_purge_option = purge_prune; break;
    case POMDP_SOLVE_OPTS_Q_Purge_epsilon_prune: params->q_purge_option = purge_epsilon_prune; break;
    }
  
  switch( opts->enum_purge_option )
    {
    case POMDP_SOLVE_OPTS_Enum_Purge_none: params->enum_purge_option = purge_none; break;
    case POMDP_SOLVE_OPTS_Enum_Purge_domonly: params->enum_purge_option = purge_dom; break;
    case POMDP_SOLVE_OPTS_Enum_Purge_normal_prune: params->enum_purge_option = purge_prune; break;
    case POMDP_SOLVE_OPTS_Enum_Purge_epsilon_prune: params->enum_purge_option = purge_epsilon_prune; break;
    }

  switch( opts->fg_purge_option )
    {
    case POMDP_SOLVE_OPTS_Fg_Purge_none: params->fg_purge_option = purge_none; break;
    case POMDP_SOLVE_OPTS_Fg_Purge_domonly: params->fg_purge_option = purge_dom; break;
    case POMDP_SOLVE_OPTS_Fg_Purge_normal_prune: params->fg_purge_option = purge_prune; break;
    case POMDP_SOLVE_OPTS_Fg_Purge_epsilon_prune: params->fg_purge_option = purge_epsilon_prune; break;
    }

  switch( opts->vi_variation )
    {
    case POMDP_SOLVE_OPTS_Vi_Variation_normal: params->vi_variation = NormalVi; break;
    case POMDP_SOLVE_OPTS_Vi_Variation_zlz: params->vi_variation = ZlzSpeedup; break;
    case POMDP_SOLVE_OPTS_Vi_Variation_adjustable_epsilon: params->vi_variation = AdjustableEpsilonVi; break;
    case POMDP_SOLVE_OPTS_Vi_Variation_fixed_soln_size: params->vi_variation = FixedSolnSizeVi; break;
    }


} /* tempOptsToParamConversion */

/**********************************************************************/
PomdpSolveParams 
parseCmdLineAndCfgFile( int argc, char **argv ) 
{
  /*
    Parses the pomdp-solve command line, setting variables for the
    different settings.  The actual actions taken because of these
    settings are done later.

    Returns the structure with all this information, or exits the
    program iwth an error message.  
  */
  PomdpSolveParams params;

  params = newPomdpSolveParams();

  /* Put anything that needs to happen before parsing the command line
	in this routine. */   
  doPreOptionParseActions();

   /* Parse command line to check for validity and to extract all the
	 things we need. The program will exit and/or print a message
	 indicating problems with the command line parsing, so we need
	 not take any action here. */ 
  params->opts = POMDP_SOLVE_OPTS_create( argc, argv );

  /* Customized option handling for this program. */
  doPostOptionParseActions( params );

  /* FIXME: Remove this when fully migrated to new config scheme. */
  tempOptsToParamConversion( params );

  return ( params );

}  /* parseCmdLineAndCfgFile */

/************************************************************/
PomdpSolveParams 
showPomdpSolveParams( PomdpSolveParams params ) 
{
  ConfigFile cfg;

  fprintf( params->report_file, 
		 " //****************\\\\\n" );
  fprintf( params->report_file, 
		 "||   %s    ||\n", params->opts->__exec_name__ );
  fprintf( params->report_file, 
		 "||     v. %s       ||\n", params->opts->__version__ );
  fprintf( params->report_file, 
		 " \\\\****************//\n" );
  fprintf( params->report_file, 
		 "      PID=%d\n", getpid() );

  cfg = POMDP_SOLVE_OPTS_toConfigFile( params->opts );

  fprintf( params->report_file, 
		 "- - - - - - - - - - - - - - - - - - - -\n" );

  CF_fprintf( cfg, params->report_file );
 
  fprintf( params->report_file, 
		 "- - - - - - - - - - - - - - - - - - - -\n" );

  CF_delete( cfg );

} /* showPomdpSolveParams */
