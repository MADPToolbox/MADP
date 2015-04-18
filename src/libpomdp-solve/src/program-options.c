/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    program-options.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: program-options.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/program-options.c,v $
 *    $Revision: 1.4 $
 *    $Date: 2005/01/19 18:11:02 $
 *    $Author: arc $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    2004, Anthony R. Cassandra
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
  Module: PO

  Handles all the details for managing a C program that you need to
  control through a combination of configuration file and command line
  options.  Command line options will override the configuration file
  options.  

  General usagage is to do something like this:

    ProgramOptions opts = PO_create( argc, argv );

    if ( ! PO_isValid( opts ))
       abort( "Aaargh! Something's wrong." );

    myarg_str = PO_getArgument( opts, 2 )

    myvalue_str = PO_getOption( opts, "myoption" )

    if ( PO_hasFlag( opts, "myflag" ))
       doSomething();

  TERMINOLOGY:

     Flags - things that appear on command line with a dash by no
     following value.  In the configuration file they wil have values
     true or false only.

     Options - things that appear on the command line with a dahs and
     a following value.  These are the same as the name-value opairs
     that appear in the configuration file.

	Arguments (aka "proper arguments" or "args_proper") - these are
	everything else on the comand line.  Things that are not values
	associated with a paricular command line option.  There is no way
	to specify these in the configuration file.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "command-line.h"
#include "program-options.h"

char* Boolean_Str[] = OPT_BOOLEAN_STRINGS;

/*******************************************************/
ProgramOptions  
PO_new( )
{
  ProgramOptions opts;
  
  opts = (ProgramOptions) XMALLOC( sizeof(*opts));

  opts->error_count = 0;
  opts->usage = 0;

  opts->cmd_line = CL_new( );

  /* These do not get created until the structure is validated, which
	follows it being populated. */
  opts->flag_seen = NULL;     
  opts->option_seen = NULL;
  opts->arg_seen = NULL;

  return opts;
} /* PO_new */

/*******************************************************/
void  
PO_delete( ProgramOptions opts )
{
  if ( opts == NULL )
    return;

  CL_delete( opts->cmd_line );

  if ( opts->flag_seen != NULL )
    XFREE( opts->flag_seen );

  if ( opts->option_seen != NULL )
    XFREE( opts->option_seen );

  if ( opts->arg_seen != NULL )
    XFREE( opts->arg_seen );

  XFREE( opts );

} /* PO_delete */

/*******************************************************/
void 
PO_handleError( ProgramOptions opts, char* err_msg )
{
  /* Prints the message to the screen and increments the error count
	on the data structure.
  */
  if ( err_msg == NULL )
      fprintf( stderr, "*Error* <unknown reason>\n" );
  else
    fprintf( stderr, "*Error* %s\n", err_msg );

  if ( opts != NULL )
    opts->error_count += 1;

} /* PO_handleError */

/*******************************************************/
int
PO_isValid( ProgramOptions opts )
{
  return opts->error_count < 1;

} /* PO_isValid */

/*******************************************************/
int
PO_getNumArguments( ProgramOptions opts )
{
  return CL_getNumArgProper( opts->cmd_line );

} /* PO_getNumArguments */

/*******************************************************/
char*
PO_getArgument( ProgramOptions opts, int arg_num )
{
  return CL_getArgProper( opts->cmd_line, arg_num );

} /* PO_getArgument */

/*******************************************************/
char*
PO_getOption( ProgramOptions opts, char* arg )
{
  return CL_getArgOption( opts->cmd_line, arg );

} /* PO_getOption */

/*******************************************************/
int
PO_hasFlag( ProgramOptions opts, char* flag )
{
  return CL_hasFlag( opts->cmd_line, flag );

} /* PO_hasFlag */

/*******************************************************/
void
PO_overlayAssocArray( ProgramOptions opts, AssocArray src_assoc )
{
  AssocArray dest_assoc;
  int i;

  if (( opts == NULL )
	 || ( src_assoc == NULL ))
    return;

  dest_assoc = opts->cmd_line->args_opt;

  for ( i = 0; i < src_assoc->cur_size; i++ )
    {
	 AA_put( dest_assoc, src_assoc->keys[i], src_assoc->values[i] );
    } /* for i */
  
} /* PO_overlayAssocArray */

/*******************************************************/
void
PO_overlayConfigFile( ProgramOptions opts, ConfigFile cfg )
{
  if (( opts == NULL )
	 || ( cfg == NULL ))
    return;

  PO_overlayAssocArray( opts, cfg->params );
  
} /* PO_overlayConfigFile */

/*******************************************************/
void
PO_overlayCommandLine( ProgramOptions opts, CommandLine src_cmd_line )
{
  int i;
  CommandLine dest_cmd_line;
    
  if (( opts == NULL )
	 || ( src_cmd_line == NULL ))
    return;

  dest_cmd_line = opts->cmd_line;
  
  strcpy( dest_cmd_line->exec_name, src_cmd_line->exec_name );

  PO_overlayAssocArray( opts, src_cmd_line->args_opt );
  
  /* Command line flags are a secial case to be handled.  In a config
	file, they must have a value, and this must be 'true' or 'false'.
	But on the command line, just the presence of the flag is
	asserting that it is true.  We normalize these views by adding
	all the flags into the options section with value "true".  This
	either adds the flag anew, or will override the config file
	option.  Note that there is no way to set a flag to be false from
	the command line if the config file says it should be true
	(actually I think by having option "-opt_name false" will work).

	NOte that this means that the ProgramOptions structure leaves
	this routine *never* having any 'flags': they are all converted
	to options.
  */

  for ( i = 0; i < src_cmd_line->num_flags; i++ )
    AA_put( dest_cmd_line->args_opt, 
		  src_cmd_line->flags[i], PO_FLAG_TRUE_VALUE );

  for ( i = 0; i < src_cmd_line->num_args_proper; i++ )
    CL_addArgProper( dest_cmd_line, src_cmd_line->args_proper[i] );

} /* PO_overlayCommandLine */

/*******************************************************/
ProgramOptions
PO_create( int argc, char **argv )
{

  CommandLine cmd_line;
  ConfigFile cfg = NULL;
  ProgramOptions opts;

  char *cfg_filename;

  opts = PO_new();

  /* First parse command line, because that is where the configuration
	file would be specified.
  */
  cmd_line = CL_parseCommandLine( argc, argv );

  /* In case we bail early, we want to save this because it is useful
	for the error and usage statements. */
  if ( cmd_line != NULL )
    strcpy( opts->cmd_line->exec_name, cmd_line->exec_name );

  if (( cmd_line == NULL )
	 || ( ! CL_isValid( cmd_line )))
    {
	 PO_handleError( opts, "Command line parsing problems."  );
	 CL_delete( cmd_line );
	 return opts;
    }

  /* Check the special case of the '-h' flag, since this will indicate
	that we need to only show the usage and nothing else.
  */
  if ( CL_hasFlag( cmd_line, "h" ))
    {
	 opts->usage = 1;
	 CL_delete( cmd_line );
	 return opts;
    }

  /* Now see if we should get a configuration file. */
  cfg_filename = CL_getArgOption( cmd_line, "f" );

  if ( cfg_filename != NULL )
    {
	 cfg = CF_read( cfg_filename );

	 if (( cfg == NULL )
		|| ( ! CF_isValid( cfg )))
	   PO_handleError( opts, "Configuration file reading problems." );
	 
    } /* if have cfg_filename */
  
  /* Now we merge the two by taking the condif file parameters, and
	then overlaying the command line options. */

  PO_overlayConfigFile( opts, cfg );

  PO_overlayCommandLine( opts, cmd_line );

  CF_delete( cfg );
  CL_delete( cmd_line );

  return opts;

} /* PO_create */

/*******************************************************/
void 					 
PO_fprintf( ProgramOptions opts, FILE* file )
{
  if ( opts == NULL )
    {
	 fprintf( file, "PO = NULL" );
	 return;
    }
  
  fprintf( file, "PO = [\n" );

  if ( opts->error_count > 0 )
    fprintf( file, "  ERRORS = %d\n", opts->error_count );
  
  CL_fprintf( opts->cmd_line, file );

  fprintf( file, "]\n" );

} /* PO_fprintf */

/*******************************************************/
void 				
PO_printf( ProgramOptions opts )
{
  PO_fprintf( opts, stdout );
} /* PO_printf */

/*******************************************************/
void 				
PO_startValidate( ProgramOptions opts )
{
  CommandLine cmd_line;

  if ( opts == NULL )
    return;

  /* First handle the case where we might be called more than once by
	cleaning up the validation checking memory. */
  if ( opts->flag_seen != NULL )
    XFREE( opts->flag_seen );
  if ( opts->option_seen != NULL )
    XFREE( opts->option_seen );
  if ( opts->arg_seen != NULL )
    XFREE( opts->arg_seen );

  cmd_line = opts->cmd_line;

  opts->flag_seen = (int *) XCALLOC( cmd_line->num_flags,
							   sizeof(int));
  opts->option_seen = (int *) XCALLOC( AA_size( cmd_line->args_opt),
								sizeof(int));
  opts->arg_seen = (int *) XCALLOC( cmd_line->num_args_proper,
							  sizeof(int));

} /* PO_startValidate */

/*******************************************************/
void 				
PO_endValidate( ProgramOptions opts )
{
  int i;
  CommandLine cmd_line;
  AssocArray assoc;
  char err_msg[256];

  if (( opts == NULL )
	 || ( opts->flag_seen == NULL )
	 || ( opts->option_seen == NULL )
	 || ( opts->arg_seen == NULL ))
    return;

  /* Find all flags anbd options that were not processed. */
  cmd_line = opts->cmd_line;

  for ( i = 0; i < cmd_line->num_flags; i++ )
    { 
	 if ( ! opts->flag_seen )
	   {
		sprintf( err_msg, "Unreckognized command line flag '%s'.",
			    cmd_line->flags[i] );
		PO_handleError( opts, err_msg );
	   }
    } /* loop over flags */

  assoc = cmd_line->args_opt;

  for ( i = 0; i < assoc->cur_size; i++ )
    { 
	 if ( ! opts->option_seen )
	   {
		sprintf( err_msg, "Unreckognized command line option '%s'.",
			    cmd_line->flags[i] );
		PO_handleError( opts, err_msg );
	   }
    } /* loop over flags */

  XFREE( opts->flag_seen );
  XFREE( opts->option_seen );
  XFREE( opts->arg_seen );

  opts->flag_seen = NULL;
  opts->option_seen = NULL;
  opts->arg_seen = NULL;

} /* PO_endValidate */

/*******************************************************/
int
PO_getFlag( ProgramOptions opts, char* opt_name )
{
  /*
    Checks the program options structure for the prsence of the flag
    of the given opt_name.  If it exists, this function returns
    PO_OPT_PRESENT_VALID, else it returns PO_OPT_NOT_PRESENT or other
    appropriate error condition.  On success, this routine also marks
    the flag as seen (if the 'seen' structure has been allocated.)
   */
  int i;
  char *cmp_str;

  /* Strip leading dash if present. */
  if ( opt_name[0] == '-' )
    cmp_str = opt_name + 1;
  else
    cmp_str = opt_name;

  if ( opts == NULL )
    return PO_OPT_INTERNAL_ERROR;

  for ( i = 0; i < opts->cmd_line->num_flags; i++ )
    {
	 if ( strcmp( opts->cmd_line->flags[i], cmp_str ) == 0 )
	   {
		if ( opts->flag_seen != NULL )
		  opts->flag_seen[i] += 1;
		return PO_OPT_PRESENT_VALID;
	   }
    } /* for i */

  return PO_OPT_NOT_PRESENT;

} /* PO_getFlag */

/*******************************************************/
int
PO_getStringOption( ProgramOptions opts, char* opt_name,
				char* ret_value, char* min, char* max  )
{
  /*
    Checks the program options structure for the command line option
    given by opt_name.  If not found, this function returns 0.  If
    found, and if min and/or max are non-NULL, then it wil also
    validate that the sring falls within this lexographical range. If
    out of range, this returns 0.  On success, this routine also marks
    the flag as seen (if the 'seen' structure has been allocated.)
   */
  int i;
  char *cmp_str;

  if (( opts == NULL )
	 || ( opt_name == NULL )
	 || ( ret_value == NULL ))
    return PO_OPT_INTERNAL_ERROR;

  /* Strip leading dash if present. */
  if ( opt_name[0] == '-' )
    cmp_str = opt_name + 1;
  else
    cmp_str = opt_name;

  for ( i = 0; i < opts->cmd_line->args_opt->cur_size; i++ )
    {
	 if ( strcmp( opts->cmd_line->args_opt->keys[i], cmp_str ) == 0 )
	   {
		strcpy( ret_value, opts->cmd_line->args_opt->values[i] );

		if ( opts->flag_seen != NULL )
		  opts->option_seen[i] += 1;

		if ( min != NULL )
		  {
		    if ( strcmp( ret_value, min ) < 0 )
			 return PO_OPT_PRESENT_ERROR;
		  }

		if ( max != NULL )
		  {
		    if ( strcmp( ret_value, max ) > 0 )
			 return PO_OPT_PRESENT_ERROR;
		  }

		return PO_OPT_PRESENT_VALID;

	   } /* if found option */

    } /* for i */

  return PO_OPT_NOT_PRESENT;

} /* PO_getStringOption */

/*******************************************************/
int
PO_getEnumOption( ProgramOptions opts, char* opt_name,
			   int *ret_idx, char** valid_values )
{
  /*
    Checks the program options for the presence of command line option
    given by opt_name.  If not present, this routine returns 0.  If
    present, it them validated the value by checking it against the
    array of strings 'valid_names'.  This valis_values array of
    strings must have a terminal sentinel that is either a NULL string
    or an empty string. On success, this routine also marks the flag
    as seen (if the 'seen' structure has been allocated.)
   */
  int i;
  int str_result;
  char found_str[MAX_OPT_STRING_LEN];

  /* Be safe to make sure this has some sanbe value if this routine
   * fails. */
  if ( ret_idx != NULL )
    *ret_idx = 0;

  if (( opts == NULL )
	 || ( opt_name == NULL )
	 || ( ret_idx == NULL )
	 || ( valid_values == NULL ))
    return PO_OPT_INTERNAL_ERROR;

  str_result = PO_getStringOption( opts, opt_name, found_str, NULL, NULL );

  if ( str_result != PO_OPT_PRESENT_VALID )
    return str_result;

  i = 0;
  while( 1 )
    {
	 if (( valid_values[i] == NULL )
		|| ( strlen( valid_values[i] ) < 1 ))
	   {
		return PO_OPT_PRESENT_ERROR;
	   }

	 if ( strcmp( valid_values[i], found_str ) == 0 )
	   {
		*ret_idx = i;
		return PO_OPT_PRESENT_VALID;
	   }

	 i += 1;

    } /* while 1 */
	 
  return PO_OPT_INTERNAL_ERROR;

} /* PO_getEnumOption */

/*******************************************************/
int
PO_getIntegerOption( ProgramOptions opts, char* opt_name,
				 int* ret_value, int min, int max  )
{
  /*
    Checks the program options structure for the command line option
    given by opt_name.  If not found, this function returns 0.  If
    found, and if min and/or max are both non-zero, then it wil also
    validate that the integer falls within this numeric range. If
    out of range, this returns 0.  On success, this routine also marks
    the flag as seen (if the 'seen' structure has been allocated.)
   */
  int i;
  int str_result;
  char found_str[MAX_OPT_STRING_LEN];

  if (( opts == NULL )
	 || ( opt_name == NULL )
	 || ( ret_value == NULL ))
    return PO_OPT_INTERNAL_ERROR;

  str_result = PO_getStringOption( opts, opt_name, found_str, NULL, NULL );

  if ( str_result != PO_OPT_PRESENT_VALID )
    return str_result;

  *ret_value = atoi( found_str );

  /* This is the special case where we have no bounds on the value. */
  if (( min == 0 ) && ( max == 0 )) {
    return PO_OPT_PRESENT_VALID;
  }

  if (( *ret_value < min ) || ( *ret_value > max )) 
    {
	 return PO_OPT_PRESENT_ERROR;
    }

  return PO_OPT_PRESENT_VALID;

} /* PO_getIntegerOption */

/*******************************************************/
int
PO_getDoubleOption( ProgramOptions opts, char* opt_name,
				double* ret_value, double min, double max )
{
  /*
    Checks the program options structure for the command line option
    given by opt_name.  If not found, this function returns 0.  If
    found, and if min and/or max are both non-zero, then it wil also
    validate that the double value falls within this numeric range. If
    out of range, this returns 0.  On success, this routine also marks
    the flag as seen (if the 'seen' structure has been allocated.)
   */
  int i;
  int str_result;
  char found_str[MAX_OPT_STRING_LEN];

  if (( opts == NULL )
	 || ( opt_name == NULL )
	 || ( ret_value == NULL ))
    return PO_OPT_INTERNAL_ERROR;

  str_result = PO_getStringOption( opts, opt_name, found_str, NULL, NULL );

  if ( str_result != PO_OPT_PRESENT_VALID )
    return str_result;

  *ret_value = strtod( found_str, NULL );

  /* This is the special case where we have no bounds on the value. */
  if (( min == 0 ) && ( max == 0 )) {
    return PO_OPT_PRESENT_VALID;
  }

  if (( *ret_value < min ) || ( *ret_value > max )) 
    {
	 return PO_OPT_PRESENT_ERROR;
    }

  return PO_OPT_PRESENT_VALID;

} /* PO_getDoubleOption */

/**********************************************************************/
void 
PO_showUsageEnumType( FILE* file, char* option_str, 
				  char** value_list ) 
{
  /*
    Displays a list of enumerated command line options using comma
    separation and inserting newlines where appropriate.  The value
    list should have a sentinel NULL or empty string at the end.
  */
  int max_char_on_line = MAX_ENUM_USAGE_WIDTH;
  int i, j, num_char_on_line;

  fprintf( file, "\t%s [ ", option_str );

  /* Need 5 characters for the tab and 3 for ' [ '. */
  num_char_on_line = 8 + strlen( option_str );

  i = 0;
  while( 1 )
    {
	 if (( value_list[i] == NULL )
		|| ( strlen( value_list[i] ) < 1 ))
	   break;
  
	 /* We will print this option on this line if it fits and on thr
	    next line if it does not. */
	 if ( (num_char_on_line + strlen( value_list[i] )) 
		 > max_char_on_line ) {

	   /* Print enough stuff so this option aligns with previous line's
		 first option. */
	   fprintf( file, "\n\t" );
	   for ( j = 0; j < (strlen( option_str )+3); j++ )
		fprintf( file, " " );

	   num_char_on_line = 8 + strlen( option_str );;
      
	 } /* if must print on next line. */
    
	 fprintf( file, "%s", value_list[i] );
	 num_char_on_line += strlen( value_list[i] );

	 /* Print a comma afterward, but only if this is not the last
	    iteration of the loop, otherwise just close the
	    brackets. */
	 if (( value_list[i+1] != NULL )
		&& ( strlen( value_list[i+1] ) > 0 ))
	   {
		fprintf( file, ", " );
		num_char_on_line += 2;
	   }
	 else
	   fprintf( file, " ]\n" );
		
	 i += 1;
    } /* while 1 */

}  /* PO_showUsageEnumType */
