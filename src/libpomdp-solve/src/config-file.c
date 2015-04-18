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
 *    $RCSfile: config-file.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/config-file.c,v $
 *    $Revision: 1.3 $
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

#include <stdio.h>

#include "global.h"
#include "config-file.h"

/*
  Module: CF

  A very simple configuration file reader and writer. Uses an
  associative array to store the key-value pairs.
 */

/*******************************************************/
ConfigFile  
CF_new( )
{
  ConfigFile cfg;
  
  cfg = (ConfigFile) XMALLOC( sizeof(*cfg));

  cfg->error_count = 0;

  cfg->params = AA_new( MAX_CFG_FILE_PARAMS, 
				    MAX_CFG_FILE_STRING_LEN );

  return cfg;
} /* CF_new */

/*******************************************************/
void  
CF_delete( ConfigFile cfg )
{
  if ( cfg == NULL )
    return;

  AA_delete( cfg->params );

  XFREE( cfg );

} /* CF_delete */

/*******************************************************/
int
CF_isValid( ConfigFile cfg )
{
  return cfg->error_count < 1;
} /* CF_isValid */

/*******************************************************/
int
CF_addParam( ConfigFile cfg, char* attr, char* value )
{
  if (( cfg == NULL )
	 || ( attr == NULL )
	 || ( value == NULL ))
    {
	 return 0;
    }

  if ( AA_put( cfg->params, attr, value ) != 1 )
    {
	 cfg->error_count += 1;
	 return 0;
    }

  return 1;
} /* CF_isValid */

/*******************************************************/
ConfigFile 
CF_fscanf( FILE* file )
{
  ConfigFile cfg;
  char attr[MAX_CFG_FILE_STRING_LEN];
  char value[MAX_CFG_FILE_STRING_LEN];

  int state;
  int file_done = 0;
  int line_done;
  int line_no = 0;
  char ch;

  int char_idx;

  cfg = CF_new();

  /* This outer loop executes one iteration per line of the file. */
  while( ! file_done  )
    {
	 line_done = 0;
	 line_no += 1;
	 char_idx = 0;
	 state = PARSE_PRE_ATTRIBUTE;

	 /* This inner loop executes until the line is exhausted. */
	 while( ! line_done )
	   {

		if ( ( ch = fgetc( file )) == EOF )
		  {
		    line_done = 1;
		    file_done = 1;
		    break;
		  }

		switch( ch )
		  {
		    /* Comments end the line */
		  case '#':

		    switch ( state )
			 {
			 case PARSE_IN_ATTRIBUTE:
			 case PARSE_POST_ATTRIBUTE:
			   cfg->error_count += 1;
			   state = PARSE_IN_ERROR;
			   break;

			 case PARSE_POST_VALUE:
			 case PARSE_PRE_ATTRIBUTE:
			   state = PARSE_IN_COMMENT;
			   break;

			   /* Allow empty strings (char_idx should be 0) */
			 case PARSE_PRE_VALUE:
			 case PARSE_IN_VALUE:
			   value[char_idx] = '\0';
			   CF_addParam( cfg, attr, value );
			   state = PARSE_IN_COMMENT;
			   break;

			 default:
			   break;
			 } /* switch state for ch='#' */
		    break;
		    
		  case '=':
		    switch ( state )
			 {
			 case PARSE_IN_VALUE:
			 case PARSE_POST_VALUE:
			 case PARSE_PRE_ATTRIBUTE:
			 case PARSE_PRE_VALUE:
			   cfg->error_count += 1;
			   state = PARSE_IN_ERROR;
			   break;

			 case PARSE_POST_ATTRIBUTE:
			   state = PARSE_PRE_VALUE;
			   break;

			 case PARSE_IN_ATTRIBUTE:
			   attr[char_idx] = '\0';
			   char_idx = 0;
			   state = PARSE_PRE_VALUE;
			   break;

			 default:
			   break;
			 } /* switch state for ch='=' */

		    break;

		    /* This '\r' is to cope with crappy MS text files.  We
			  treat it just like '\n'. This will screw up the line
			  numbering of this function, but that is the least of
			  your concerns if you are somehow reliant on MS
			  software. */
		  case '\r':
		  case '\n':
		    switch ( state )
			 {
			 case PARSE_IN_ATTRIBUTE:
			 case PARSE_POST_ATTRIBUTE:
			   cfg->error_count += 1;
			   state = PARSE_IN_ERROR;
			   break;

			 case PARSE_PRE_VALUE:
			 case PARSE_IN_VALUE:
			   value[char_idx] = '\0';
			   CF_addParam( cfg, attr, value );
			   break;

			 default:
			   break;
			 } /* switch state for ch='\n' or '\r' */

		    line_done = 1;
		    break;
		    
		    /* Spaces (any number) are delimiters. They wil end
			  and atribute or a value, but otherwise we just
			  move on. */ 
		  case ' ':
		  case '\t':
		    switch ( state )
			 {
			 case PARSE_IN_ATTRIBUTE:
			   attr[char_idx] = '\0';
			   char_idx = 0;
			   state = PARSE_POST_ATTRIBUTE;
			   break;

			 case PARSE_IN_VALUE:
			   value[char_idx] = '\0';
			   CF_addParam( cfg, attr, value );
			   state = PARSE_POST_VALUE;
			   break;

			 default:
			   break;
			 } /* switch state for ch=' ' or '\t' */

		    break;
		    
		    /* The default case here is really the common case of
			  seeing an ordinary character */ 
		  default:

		    switch ( state )
			 {
			 case PARSE_IN_ATTRIBUTE:
			   attr[char_idx] = ch;
			   char_idx += 1;
			   break;

			 case PARSE_IN_VALUE:
			   value[char_idx] = ch;
			   char_idx += 1;
			   break;

			 case PARSE_POST_ATTRIBUTE:
			 case PARSE_POST_VALUE:
			   cfg->error_count += 1;
			   state = PARSE_IN_ERROR;
			   break;

			 case PARSE_PRE_ATTRIBUTE:
			   attr[0] = ch;
			   char_idx = 1;
			   state = PARSE_IN_ATTRIBUTE;
			   break;

			 case PARSE_PRE_VALUE:
			   value[0] = ch;
			   char_idx = 1;
			   state = PARSE_IN_VALUE;
			   break;

			 default:
			   break;
			 } /* switch state for ch=default */

		  } /* switch ch */
		  
	   } /* inner loop over chars of a line */
    }  /* outer loop over lines */
  

  return cfg;
} /* CF_fscanf */

/*******************************************************/
ConfigFile 
CF_read( char* filename )
{
  FILE *file;
  ConfigFile ret_val;

  if (( file = fopen( filename , "r")) == NULL) 
    {
	 return NULL;
    }

  ret_val = CF_fscanf( file );

  fclose( file );

  return ret_val;
} /* CF_read */

/*******************************************************/
int
CF_fprintf( ConfigFile cfg, FILE* file )
{

  if ( cfg == NULL )
    {
	 fprintf( file, "# CF = NULL\n" );
	 return 0;
    }

  if ( cfg->error_count > 0 )
	   fprintf( file, "#  ERRORS = %d\n", cfg->error_count );
  
  AA_fprintf( cfg->params, file );
  
  return 1;
	 
} /* CF_fprintf */

/*******************************************************/
int 
CF_printf( ConfigFile cfg )
{
  return CF_fprintf( cfg, stdout );

} /* CF_printf */

/*******************************************************/
int 
CF_write( ConfigFile cfg, char* filename )
{
  FILE *file;
  int ret_val;

  if (( file = fopen( filename , "w")) == NULL) 
    {
	 return 0;
    }

  ret_val = CF_fprintf( cfg, file );

  fclose( file );

  return ret_val;

} /* CF_write */

