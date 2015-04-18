/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    cmd-line.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: command-line.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/command-line.c,v $
 *    $Revision: 1.2 $
 *    $Date: 2004/10/10 03:44:53 $
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
  Module: CL

  Data structure for holding command line information, and functions
  to manipulate its contents.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "command-line.h"

/*******************************************************/
CommandLine
CL_new( )
{
  /*
    Constructor that allocated memory for data structure.
  */
  CommandLine cmd_line;
  int i;

  cmd_line = (CommandLine) XMALLOC( sizeof( *cmd_line ));

  cmd_line->error_count = 0;

  cmd_line->exec_name =  (char *) XMALLOC( MAX_CMD_LINE_STRING_LEN
								  * sizeof(char));

  cmd_line->args_opt = AA_new( MAX_CMD_LINE_ARGS, 
						 MAX_CMD_LINE_STRING_LEN );

  cmd_line->max_flags = MAX_CMD_LINE_FLAGS;
  cmd_line->num_flags = 0;
  cmd_line->flags = (char **) XMALLOC( cmd_line->max_flags
							   * sizeof(char*));
  for ( i = 0; i < cmd_line->max_flags; i++ )
    {
	 cmd_line->flags[i] = (char *) XMALLOC( MAX_CMD_LINE_STRING_LEN
								    * sizeof(char));
    } /* for i */

  cmd_line->max_args_proper = MAX_CMD_LINE_ARGS;
  cmd_line->num_args_proper = 0;
  cmd_line->args_proper = (char **) XMALLOC( cmd_line->max_args_proper
								    * sizeof(char*));
  for ( i = 0; i < cmd_line->max_args_proper; i++ )
    {
	 cmd_line->args_proper[i] = (char *) XMALLOC( MAX_CMD_LINE_STRING_LEN
										* sizeof(char));
    } /* for i */

  return cmd_line;
} /* CL_new */

/*******************************************************/
void
CL_delete( CommandLine cmd_line )
{
  /*
    Destructor that free the memory that was dynamically created for
    this data structure.
   */
  int i;

  if ( cmd_line == NULL )
    return;

  XFREE( cmd_line->exec_name );

  AA_delete( cmd_line->args_opt );

  for ( i = 0; i < cmd_line->max_flags; i++ )
    {
	 XFREE( cmd_line->flags[i] );
    }
  XFREE( cmd_line->flags );

  for ( i = 0; i < cmd_line->max_args_proper; i++ )
    {
	 XFREE( cmd_line->args_proper[i] );
    }
  XFREE( cmd_line->args_proper );

  XFREE( cmd_line );

} /* CL_delete */

/*******************************************************/
void 
CL_handleError( CommandLine cmd_line, char* err_msg )
{
  /* Prints the message to the screen and increments the error count
	on the data structure.
  */
  if ( err_msg == NULL )
      fprintf( stderr, "*Error* <unknown reason>\n" );
  else
    fprintf( stderr, "*Error* %s\n", err_msg );

  if ( cmd_line != NULL )
    cmd_line->error_count += 1;

} /* PO_handleError */

/*******************************************************/
int
CL_isValid( CommandLine cmd_line )
{
  return cmd_line->error_count < 1;

} /* CL_isValid */

/*******************************************************/
int
CL_addExecName( CommandLine cmd_line, char* exec_name )
{
  /*
    Adds the executable name to the CommandLine data structure.
    Returns 1 if successful and 0 if there was an error (bad args,
    name too long)
   */
  if (( cmd_line == NULL ) || ( exec_name == NULL ))
    return 0;

  if ( strlen( exec_name ) >= MAX_CMD_LINE_STRING_LEN )
    {
	 CL_handleError( cmd_line, 
				  "Exec name too long. Alter MAX_CMD_LINE_STRING_LEN?" );
	 return 0;
    }

  strcpy( cmd_line->exec_name, exec_name );

  return 1;
} /* CL_addExecName */

/*******************************************************/
int
CL_addFlag( CommandLine cmd_line, char* dashed_flag )
{
  /*
    Adds a flag to the CommandLine data structure.  Returns 1 if
    successful and 0 if there was an error (bad args, name too long,
    out of space, etc.)
   */

  char err_str[256];

  if (( cmd_line == NULL ) || ( dashed_flag == NULL ))
    {
	 CL_handleError( cmd_line, 
				  "CL_addFlag() had NULL parameters" );
	 return 0;
    }

  if ( cmd_line->num_flags >= cmd_line->max_flags )
    {
	 CL_handleError( cmd_line, 
				  "Too many command line flags." );
	 return 0;
    }

  if ( strlen( dashed_flag ) >= MAX_CMD_LINE_STRING_LEN )
    {
	 sprintf( err_str, "Command line flag '%s' is too long. %s",
			dashed_flag, "Change MAX_CMD_LINE_STRING_LEN?" );
	 CL_handleError( cmd_line, err_str );
	 return 0;
    }

  /* Strip the dash only if present. */
  if ( dashed_flag[0] == '-' )
    strcpy( cmd_line->flags[cmd_line->num_flags], dashed_flag+1 );
  else
    strcpy( cmd_line->flags[cmd_line->num_flags], dashed_flag );

  cmd_line->num_flags += 1;

  return 1;
} /* CL_addFlag */

/*******************************************************/
int
CL_addArgOption( CommandLine cmd_line, 
			  char* dashed_opt, char* value )
{
  /*
    Adds an argument options to the CommandLine data structure.
    Returns 1 if successful and 0 if there was an error (bad args,
    name too long, out of space, etc.)
   */
  int aa_ret;
  char err_str[256];

   if (( cmd_line == NULL ) 
	  || ( dashed_opt == NULL )
	  || ( value == NULL ))
    {
	 CL_handleError( cmd_line, 
				  "CL_addOption() had NULL parameters" );
	 return 0;
    }

  if ( strlen( dashed_opt ) >= MAX_CMD_LINE_STRING_LEN )
    {
	 sprintf( err_str, "Command line flag '%s' is too long. %s",
			dashed_opt, "Change MAX_CMD_LINE_STRING_LEN?" );
	 CL_handleError( cmd_line, err_str );
	 return 0;
    }

  if ( strlen( value ) >= MAX_CMD_LINE_STRING_LEN )
    {
	 sprintf( err_str, "Command line value '%s' is too long. %s",
			value, "Change MAX_CMD_LINE_STRING_LEN?" );
	 CL_handleError( cmd_line, err_str );
	 return 0;
    }

   if ( dashed_opt[0] == '-' )
	aa_ret = AA_put( cmd_line->args_opt, dashed_opt+1, value );
   else
	aa_ret = AA_put( cmd_line->args_opt, dashed_opt, value );

   if ( aa_ret != 1 )
    {
	 CL_handleError( cmd_line, 
				  "Too many command line options." );
	 return 0;
    }

   return 1;

} /* CL_addArgOption */

/*******************************************************/
int
CL_addArgProper( CommandLine cmd_line, char* value )
{
  /*
    Adds an proper argument to the CommandLine data structure.
    Returns 1 if successful and 0 if there was an error (bad args,
    name too long, out of space, etc.)
   */
  char err_str[256];

  if (( cmd_line == NULL ) || ( value == NULL ))
    {
	 CL_handleError( cmd_line, 
				  "CL_addArgProper() had NULL parameters" );
	 return 0;
    }

  if ( cmd_line->num_args_proper >= cmd_line->max_args_proper )
    {
	 CL_handleError( cmd_line, 
				  "Too many command line arguments." );
	 return 0;
    }

  if ( strlen( value ) >= MAX_CMD_LINE_STRING_LEN )
    {
	 sprintf( err_str, "Command line argument '%s' is too long. %s",
			value, "Change MAX_CMD_LINE_STRING_LEN?" );
	 CL_handleError( cmd_line, err_str );
	 return 0;
    }

  strcpy( cmd_line->args_proper[cmd_line->num_args_proper], value );

  cmd_line->num_args_proper += 1;

  return 1;
} /* CL_addFlag */

/*******************************************************/
extern int 
CL_hasFlag( CommandLine cmd_line, char* flag)
{
  /*
    Returns 1 if the flag of the given name is present and 0
    otherwise.
  */
  int i;

  for ( i = 0; i < cmd_line->num_flags; i++ )
    {
	 if ( strcmp( cmd_line->flags[i], flag ) == 0 )
	   return 1;
    } /* for i */

  return 0;
} /* CL_hasFlag */

/*******************************************************/
extern char*
CL_getArgOption( CommandLine cmd_line, char* arg )
{
  int i;
  char *value;

  if (( cmd_line == NULL )
	 || ( arg == NULL ))
    return NULL;

  value = AA_get( cmd_line->args_opt, arg );
  
  if ( value != NULL )
    return value;

  return NULL;

} /* CL_getArgOption */


/*******************************************************/
int
CL_getNumArgProper( CommandLine cmd_line )
{
  if ( cmd_line == NULL )
    return -1;

  return cmd_line->num_args_proper;

} /* CL_getNumArgProper */

/*******************************************************/
extern char*
CL_getArgProper( CommandLine cmd_line, int arg_num )
{
  int i;
  char *value;

  if (( cmd_line == NULL )
	 || ( arg_num < 0 )
	 || ( arg_num >= cmd_line->num_args_proper ))
    return NULL;

  return cmd_line->args_proper[arg_num];
  
} /* CL_getArgProper */

/*******************************************************/
void
CL_fprintf( CommandLine cmd_line, FILE* file )
{
  int i;

  if ( cmd_line == NULL )
    {
	 fprintf( file, "CL = NULL" );
	 return;
    }
  
  fprintf( file, "CL = [\n" );

  if ( cmd_line->error_count > 0 )
    fprintf( file, "  ERRORS = %d\n", cmd_line->error_count );

  fprintf( file, "  exec_name -> %s\n", cmd_line->exec_name );

  if ( cmd_line->num_flags > 0 )
    {
	 fprintf( file, "  Flags:\n" );
	 for ( i = 0; i < cmd_line->num_flags; i++ )
	   fprintf( file, "    %s\n", cmd_line->flags[i] );

    } /* if have at least one flag */

  else
    {
	 fprintf( file, "  Flags: none\n" );
    }

  if ( AA_size( cmd_line->args_opt ) > 0 )
    {
	 fprintf( file, "  Options:\n" );
	 AA_fprintf( cmd_line->args_opt, file );

    } /* if have at least one option */

  else
    {
	 fprintf( file, "  Options: none\n" );
    }

  if ( cmd_line->num_args_proper > 0 )
    {
	 fprintf( file, "  Arguments:\n" );
	 for ( i = 0; i < cmd_line->num_args_proper; i++ )
	   fprintf( file, "    %s\n", cmd_line->args_proper[i] );

    } /* if have at least one proper argument */

  else
    {
	 fprintf( file, "  Arguments: none\n" );
    }
  
  fprintf( file, "]\n" );
  
} /* CL_fprintf */

/*******************************************************/
void
CL_printf( CommandLine cmd_line )
{
  CL_fprintf( cmd_line, stdout );
} /* CL_printf */

/*******************************************************/
CommandLine 
CL_parseCommandLine( int argc, char** argv )
{
  /*
    Converts the typical (argc,argv) command line structure into the
    ComandLine data structure defined in the CL module.
  */
  CommandLine cmd_line;
  int i;

  cmd_line = CL_new();

  CL_addExecName( cmd_line, argv[0] );

  for( i = 1; i < argc; i++ )
    { 
	 if ( argv[i][0] == '-' )
	   {
		/* CASE: This is the last thing on the command line.
		   Resolution: Assume this is a flag option.
		 */
		if ( i == (argc-1) )
		  {
		    CL_addFlag( cmd_line, argv[i] );
		    continue;
		  } /* if last thing on line */

		/* CASE: The next thing on the command line also starts with
		   a dash.
		   Resolution: Assume this is a flag option.
		 */
		if ( argv[i+1][0] == '-' )
		  {
		    CL_addFlag( cmd_line, argv[i] );
		    continue;
		  }

		/* CASE: The next thing on the command line does not start with
		   a dash.
		   Resolution: Assume this is an argument option.
		 */
		    CL_addArgOption( cmd_line, argv[i], argv[i+1] );
		    i += 1;
		    continue;

	   } /* If this arg starts with a dash */

	 /* The way we parse this, any time we get here (the current argv
	    does not begin with a dash), then it must be an 'args
	    proper' */
	 CL_addArgProper( cmd_line, argv[i] );
	 continue;

    } /* for i */
  
  return cmd_line;

} /* CL_parseCommandLine */

