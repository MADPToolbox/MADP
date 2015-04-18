
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    belief.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    April, 2003
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: belief.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/belief.h,v $
 *    $Revision: 1.2 $
 *    $Date: 2004/03/08 08:33:39 $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    2003, Anthony R. Cassandra
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
#ifndef BELIEF_H
#define BELIEF_H

typedef struct BeliefListStruct *BeliefList;
struct BeliefListStruct {

  double *b;

  int mark;  /* can be used to mark elements of the list as used or
                unused. */

  BeliefList next;
};


/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/* How many decimal places to show when write grid to file/screen */
#define BELIEF_FILE_DECIMAL_PRECISION          25

/**********************************************************************/
/********************   DEFAULT VALUES       **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

extern double *newBelief( );
extern double *duplicateBelief( double *b );
extern void copyBelief( double *dest, double *src );
extern void destroyBelief( double *b );
extern int sameBelief( double *b1, double *b2, double epsilon );
extern BeliefList readBeliefList( char *filename, int max_beliefs ); 
extern void writeBeliefList( FILE *file, BeliefList belief_list );
extern void saveBeliefList( BeliefList list, char *filename );
extern void showBeliefList( BeliefList list );
extern BeliefList readBeliefList( char *filename, int max );
extern BeliefList newBeliefNode( double *b );
extern void destroyBeliefNode( BeliefList temp );
extern BeliefList prependBeliefList( BeliefList list,
							  double *b );
extern BeliefList appendBeliefList( BeliefList list, double *b );
extern void destroyBeliefList( BeliefList list );
extern int sizeBeliefList( BeliefList list );
extern BeliefList findBeliefState( BeliefList list, 
							double *b,
							double epsilon );

  
#endif
