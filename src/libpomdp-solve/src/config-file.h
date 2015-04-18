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
 *    $RCSfile: config-file.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/config-file.h,v $
 *    $Revision: 1.1 $
 *    $Date: 2004/06/17 00:00:31 $
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
  Module: CF

  A very simple configuration file reader and writer. Uses an
  associative array to store the key-value pairs.
 */

#ifndef CONFIG_FILE_H
#define CONFIG_FILE_H

#include "associative-array.h"

/*******************************************************/
/* Private Data */
/*******************************************************/

#define MAX_CFG_FILE_STRING_LEN     100
#define MAX_CFG_FILE_PARAMS         256

/* Parse sates for parsing configuration file. See CF_fscanf(). */

#define PARSE_IN_ATTRIBUTE          0
#define PARSE_IN_COMMENT            1 
#define PARSE_IN_VALUE              2 
#define PARSE_POST_ATTRIBUTE        3       
#define PARSE_POST_VALUE            4   
#define PARSE_PRE_ATTRIBUTE         5      
#define PARSE_PRE_VALUE             6  
#define PARSE_IN_ERROR              7 

typedef struct ConfigFileStruct* ConfigFile;
struct ConfigFileStruct {

  int error_count;

  AssocArray params;

};

/*******************************************************/
/* Public Data */
/*******************************************************/

extern ConfigFile  
CF_new( );

extern void  
CF_delete( ConfigFile );

extern int
CF_isValid( ConfigFile );

extern int
CF_addParam( ConfigFile, char*, char* );

extern ConfigFile 
CF_fscanf( FILE* );

extern ConfigFile 
CF_read( char* );

extern int
CF_fprintf( ConfigFile, FILE* );

extern int 
CF_printf( ConfigFile );

extern int 
CF_write( ConfigFile, char* );

#endif
