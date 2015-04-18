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
 *    $RCSfile: command-line.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/command-line.h,v $
 *    $Revision: 1.2 $
 *    $Date: 2004/10/07 04:41:01 $
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

#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include "associative-array.h"

/*******************************************************/
/* Private Data */
/*******************************************************/

#define MAX_CMD_LINE_FLAGS          100
#define MAX_CMD_LINE_STRING_LEN     1024
#define MAX_CMD_LINE_ARGS           256

typedef struct CommandLineStruct* CommandLine;
struct CommandLineStruct {

  /* Anytime and operation fails on a mutator method of this data
	structure, this count is incremented.
  */
  int error_count;

  char* exec_name;

  /* These are the flags that begin with a dash, but do not have an
	argument after them. Note that you cannot have a flag followed by
	an "proper argument" as this is ambiguous.
  */
  int max_flags;        /* Amount of space allocated */
  int num_flags;        /* Number of locations used. */ 
  char** flags;

  /* This is an associative array mapping command line options
	(beginning with a 'dash' and followed by an option.) 
  */
  AssocArray args_opt;

  /* These are non-options command line arguments in the order they
	appear on the command line. 
  */
  int max_args_proper;   /* Amount of space allocated */
  int num_args_proper;   /* Number of locations used. */ 
  char** args_proper;

};

/*******************************************************/
/* Public Data */
/*******************************************************/

extern CommandLine 
CL_new( );

extern void 
CL_delete( CommandLine );

extern void 
CL_handleError( CommandLine, char* );

extern int
CL_isValid( CommandLine cmd_line );

extern int 
CL_addExecName( CommandLine, char* );

extern int 
CL_addFlag( CommandLine, char* );

extern int 
CL_addArgOption( CommandLine, char*, char* );

extern int 					   
CL_addArgProper( CommandLine, char* );

extern int 
CL_hasFlag( CommandLine, char* );

extern char*
CL_getArgOption( CommandLine, char* );

extern int
CL_getNumArgProper( CommandLine );

extern char*
CL_getArgProper( CommandLine, int );

extern void 					 
CL_fprintf( CommandLine, FILE* );

extern void 				
CL_printf( CommandLine );

extern CommandLine
CL_parseCommandLine( int, char** );

#endif
