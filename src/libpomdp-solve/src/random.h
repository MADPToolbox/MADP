
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    random.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: random.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/random.h,v $
 *    $Revision: 1.1 $
 *    $Date: 2003/05/13 21:46:40 $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    1994-1997, Brown University
 *    1998-2003, Anthony R. Cassandra
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
#ifndef RANDOM_H
#define RANDOM_H

/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/* How big the array of short int's should be for the random seed */
#define SEED_SIZE                  3

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/* Seeds the psuedo-random number generated if it has not already been
  seeded.  */
extern void randomize();

/* Returns a uniform psuedo-random number between 0 and 1 */
extern double fran();

/* Returns the current random seed.  Useful if you want to reproduce 
   the sequence (e.g., debugging) */
extern void getRandomSeed( unsigned short seed[SEED_SIZE] );

/* Allows you to reproduce a psuedo-random sequence by explicitly 
   setting the seed value for the random number generator */
extern void setRandomSeed( unsigned short seed[SEED_SIZE] );

/* Sets seed from a string specification. */
extern void setRandomSeedFromString ( char *str );

/* Displays the current random seed to file stream. */
extern void displayRandomSeed( FILE *file );

/* Displays the current random seed on stdout. */
extern void showRandomSeed( );

/* Returns a uniform psuedo-random number between min and max in the 
 form of a double precision number */
extern double getRandomDouble( double min, 
                               double max );

/* Returns a uniform psuedo-random number between min and max in the 
 form of an integer */
extern int getRandomInt( int min, 
                         int max );

/* This routine sets the discrete probability distribution so that
   each distribution is equally likely.  */
extern void setRandomDistribution( double *x, 
                                   int num_probs );

/* Sets the given vector of doubles with random values betwen the min
  and max values given.  'num' is the number of elements in the
  vector.  */
extern void setRandomDoubleVector( double *vect, 
                                   int num, 
                                   double min, 
                                   double max );
 
#endif


