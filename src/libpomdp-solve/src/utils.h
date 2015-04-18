/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    utils.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    April, 2003
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: utils.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/utils.h,v $
 *    $Revision: 1.7 $
 *    $Date: 2005/01/25 21:20:46 $
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
#ifndef UTILS_H
#define UTILS_H

/* Rather than worry about dynamically alocating arrays for tracking
   ties in best comparisons, we will make an assumption that there are
   at most a fixed number of ties.  This variable sets this amount and
   the code that uses it should check to see if this is exceeding and
   spit out a warning/error message if it has so that the user can
   increase this value for their problem.
*/
#define MAX_BEST_COUNT   100

extern void UTIL_mapBeliefList( char *belief_filename,
						  char *alpha_filename,
						  double epsilon,
						  char *map_filename );
  
extern void UTIL_sortAlphaFile( char *in_alpha_filename,
						  char *out_alpha_filename );

extern void UTIL_purgeAlphaFile( char *in_alpha_filename,
						   char *out_alpha_filename,
						   PomdpSolveParams param );

extern void UTIL_compareAlphaFiles( char *alpha1_filename,
							 char *alpha2_filename,
							 double epsilon,
							 char *out_filename );

extern void UTIL_relinkPolicyGraph( char *alpha_filename,
							 char *pg_filename,
							 char *prev_alpha_filename,
							 char *out_pg_filename );

extern void UTIL_doBeliefUpdates();

#endif
