
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    cmd-line.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: cmd-line.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/cmd-line.c,v $
 *    $Revision: 1.1 $
 *    $Date: 2003/05/13 21:46:39 $
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
 *   Routines for parsing the main command line for pomdp-solve.  Also
 *   calls some algorithmic specific command line parsing routines.  
 */

#define CMD_LINE_C

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "global.h"
#include "cmd-line.h"

/**********************************************************************/
int 
getFlagParam( int argc, char **argv, char *arg_str, int *mark_arg )
{
  /* 
     Parses the command line looking for an occurence of the string
     'arg_str'.  If there is a string, it checks the range of the
     number it marks its position in the mark_arg array.  
     Returns the argument nuber if successful and zero otherwise.  
  */
  int i;

  if (( argc < 1 ) || ( argv == NULL ) ||
      (mark_arg == NULL) || ( arg_str == NULL ))
    return ( 0 );
  
  /* Need room for null terminator character. */
  if ( strlen( arg_str ) > ( MAX_CMD_ARG_LENGTH - 1 )) {
    Warning ( "Argument string too large (change 'MAX_CMD_ARG_LENGTH'?)." );
    return ( 0 );
  }
  
  /* Make sure there is no whitespace in the string. */
  for ( i = 1; i < strlen( arg_str ); i++ )
    if (( arg_str[i] == ' ' ) 
        || ( arg_str[i] == '\t' ) 
        || ( arg_str[i] == '\n' )) {
      Warning( " getDoubleParam() Argument string has whitespace." );
      return ( 0 );
    }
  
  for ( i = 0; i < argc; i++ )
    
    /* See if this is the argument we are looking for. */
    if ( strcmp( argv[i], arg_str ) == 0 ) {
      mark_arg[i]++;
      return ( i );
    }
  
  return ( 0 );

}  /* getFlagParam */
/**********************************************************************/
int 
getStringParam( int argc, char **argv, char *arg_str, 
		int *mark_arg, char *value ) 
{
  /* 
     Parses the command line looking for an occurence of the string
     'arg_str'.  If there is no string that follows it, then the
     argument is not incremented in the mark_arg array.  If there is a
     string, it checks the range of the number it marks its position in
     the mark_arg array.  Returns the argument nuber if successful and
     zero otherwise.  
  */
  int arg_num;

  if (( argc < 1 ) || ( argv == NULL ) 
      || (mark_arg == NULL) || ( value == NULL ) 
      || ( arg_str == NULL ))
    return ( 0 );
  
  /* Search through command line to see if the arg_str is there
     and find what position it is.  Zero means it is not there.  */
  arg_num = getFlagParam( argc, argv, arg_str, mark_arg );
  
  /* First see if flag was found. */
  if ( arg_num <= 0 )
    return ( 0 );
  
  /* Next see if this option is the last argv, because in this
     case, it must be missing its argument. */
  if ( arg_num == (argc-1)) {
    mark_arg[(argc-1)] = 0;
    return ( 0 );
  }
  
  strcpy( value, argv[arg_num+1] );
  mark_arg[arg_num+1]++;
  return ( arg_num );
    
}  /* getStringParam */
/**********************************************************************/
int 
getIntParam( int argc, char **argv, 
	     char *arg_str, int *mark_arg,
	     int *value, int min, int max  ) 
{
  /*
    Does a getStringParam and then converts the string into a int.
    It then checks it against the min and max values.  If the min and max 
    values are both zero, then any number is allowed. 
  */
  char str[MAX_CMD_ARG_LENGTH];
  int temp, temp_value;
  int arg_num;

  arg_num = getStringParam( argc, argv, arg_str, mark_arg, str );

  if ( arg_num == 0 )
    return ( 0 );

  temp_value = atoi( str );

  /* This is the case where we have no bounds on the value. */
  if (( min == 0 ) && ( max == 0 )) {
    *value = temp_value;
    return ( 1 );
  }

  /* We don't mind if they reversed the order of the min and max.
     We just want to check the range between min and max. */
  if ( min > max ) {
    temp = min;
    min = max;
    max = temp;
  }

  /* If the value is out of range, then we want to decrement the
     marked array (since it was incremented when it was found in
     the getStringParam() routine. */
  if (( temp_value < min ) || ( temp_value > max )) {
    mark_arg[arg_num+1]--;
    return ( 0 );
  }

  *value = temp_value;
  return ( 1 );

}  /* getIntParam */
/**********************************************************************/
int 
getDoubleParam( int argc, char **argv, 
		char *arg_str, int *mark_arg,
		double *value, double min, double max  ) 
{
/*
  Does a getStringParam and then converts the string into a double.
  It then checks it against the min and max values.  If the min and max 
  values are both zero, then any number is allowed. 
  
  Note that we do not want to set the 'value' parameter until we know
  that we have a valid value.  We want to leave the current value
  alone, so that if it is initially set to some default value, then
  we will get the default if we do not find that it should be
  overridden.  
*/
  char str[MAX_CMD_ARG_LENGTH];
  double temp, temp_value;
  int arg_num;

  arg_num = getStringParam( argc, argv, arg_str, mark_arg, str );

  if ( arg_num == 0 )
    return ( 0 );

  temp_value = strtod( str, NULL );

  /* This is the case where we have no bounds on the value. */
  if (( min == 0.0 ) && ( max == 0.0 )) {
    *value = temp_value;
    return ( 1 );
  }

  /* We don't mind if they reversed the order of the min and max.
     We just want to check the range between min and max. */
  if ( min > max ) {
    temp = min;
    min = max;
    max = temp;
  }

  /* If the value is out of range, then we want to decrement the
     marked array (since it was incremented when it was found in
     the getStringParam() routine. */
  if (( temp_value < min ) || ( temp_value > max )) {
    mark_arg[arg_num+1]--;
    return ( 0 );
  }

  *value = temp_value;
  return ( 1 );

}  /* getDoubleParam */
/**********************************************************************/
int 
getStringParamValidate( int argc, char **argv, char *arg_str, 
			int *mark_arg, int *valid_match,
			char **valid_str, int num_valid_str ) 
{
  /* 
     Parses the command line looking for an occurence of the string
     'arg_str'.  If there is no string that follows it, then the
     argument is not incremented in the mark_arg array.  If there is a
     string, it checks the range of the number it marks its position
     in the mark_arg array.  Returns the argument nuber if successful
     and zero otherwise.  
  */
  char str[MAX_CMD_ARG_LENGTH];
  int arg_num;
  int i;

  arg_num = getStringParam( argc, argv, arg_str, mark_arg, str );

  if ( arg_num == 0 )
    return ( 0 );

  for ( i = 0; i < num_valid_str; i++)
    if ( strcmp( str, valid_str[i]) == 0 ) {
      *valid_match = i;
      return ( arg_num );
    }
  
  /* Unmark the string that follows if it is not valid. */
  mark_arg[arg_num+1]--;

  return ( 0 );
}  /* getStringParamValidate */
/**********************************************************************/
void 
showUsageEnumType( FILE *file, char *option_str, 
		   int num_str, char **str ) 
{
  /*
    Displays a list of enumerated command line options using comma
    separation and inserting newlines where appropriate.
  */
  int max_char_on_line = 75;
  int i, j, num_char_on_line;

  fprintf( file, "\t%s [ ", option_str );

  /* Need 5 characters for the tab and 3 for ' [ '. */
  num_char_on_line = 8 + strlen( option_str );

  for( i = 0; i < num_str; i++ ) {
  
    /* We will print this option on this line if it fits and on thr
       next line if it does not. */
    if ( (num_char_on_line + strlen( str[i] )) > max_char_on_line ) {

      /* Print enough stuff so this option aligns with previous line's
         first option. */
      fprintf( file, "\n\t" );
      for ( j = 0; j < (strlen( option_str )+3); j++ )
        fprintf( file, " " );

      num_char_on_line = 8 + strlen( option_str );;
      
    } /* If must print on next line. */
    
    fprintf( file, "%s", str[i] );
    num_char_on_line += strlen( str[i] );

    /* Print a comma afterward, but only if this is not the last
       iteration of the loop, otherwise just close the brackets. */
    if ( i != (num_str-1)) {
      fprintf( file, ", " );
      num_char_on_line += 2;
    }
    else
      fprintf( file, " ]\n" );

  } /* for i */

}  /* showUsageEnumType */
/**********************************************************************/
void 
parseVerboseModes( char *list ) 
{
  /*
    Take a string of comma separated verbose mneomics from the command
    line and set the verbose mode to true for each mode specified.
    Ignores any bad mnemonics.
  */
  int i;
  
  for ( i = 0; i < NUM_VERBOSE_MODES; i++ ) {
    
    /* Just see if each mode mnemonic is a substring of this
       string. */
    if ( strstr( list, verbose_mode_str[i] ) != NULL )
      gVerbose[i] = TRUE;

  } /* for i */

}  /* parseVerboseModes */
/**********************************************************************/

