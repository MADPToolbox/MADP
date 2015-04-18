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
 *    $RCSfile: associative-array.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/associative-array.h,v $
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
  Module: AA

  A very simple associative array that maps string keys to string
  values. 
 */

#ifndef ASSOCIATIVE_ARRAY_H
#define ASSOCIATIVE_ARRAY_H

/*******************************************************/
/* Private Data */
/*******************************************************/

typedef struct AssocArrayStruct* AssocArray;
struct AssocArrayStruct {

  int max_size;     /* Size of memory allocated */
  int cur_size;     /* Number of used locations */

  char** keys;
  char** values;
};

/*******************************************************/
/* Public Data */
/*******************************************************/

extern AssocArray  
AA_new( int, int );

extern void  
AA_delete( AssocArray );

extern int 
AA_size( AssocArray );

extern int 
AA_put( AssocArray, char*, char* );

extern char* 
AA_get( AssocArray, char* );

extern void 
AA_fprintf( AssocArray, FILE* );

extern void 
AA_printf( AssocArray );

#endif
