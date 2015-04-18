
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    common.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: common.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/common.h,v $
 *    $Revision: 1.2 $
 *    $Date: 2004/09/03 19:14:42 $
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

#ifndef COMMON_H
#define COMMON_H

#include "params.h"

/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/**********************************************************************/
/********************   DEFAULT VALUES       **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/* Many places in the code can use temporary memory storage for
  calculations. This just allocates some useful data structures for
  this purpose.  */
extern void initCommon(  );

/* Free up any temporary memory that was allocated.  */
extern void cleanUpCommon(  );

/* Computes the Bellman residual between two successive value
   funcitons, finding the point of maximal difference between the two
   sets.  */
extern double bellmanError( AlphaList prev_list, 
                            AlphaList cur_list,
                            PomdpSolveParams param );

/* This routine will first create all of the alpha vectors (one for
   each action) for this point in the global array gCurAlphaVector,
   then it will determine which one is best for this point.  It will
   return the best value and set the parameter 'action' to be the
   action that was best.  If there are ties...(they are currently
   deterministically broken) Assumes gValueType is reward.  */
extern double oneStepValue( double *b, 
                            AlphaList **projection,
                            AlphaList *best_vector,
                            double epsilon );

/* This routine will actually create the new alpha vector for the
   point 'b' sent in.  It will add the vector to the list if it is not
   already there, and either way it will return the pointer into
   new_alpha_list for the vector.  It first constructs all the vectors
   (for each action), then it finds which one is best (via dot
   product) finally it checks to see if the vector is in the list or
   not, and adds if if it isn't.  */
extern AlphaList makeAlphaVector( AlphaList new_alpha_list, 
                                  AlphaList **projection,
                                  double *b,
                                  double epsilon );

/* This initializes the given list with vectors that are constructed
  from the projection sets sent in at the belief simplex corners.
  Will loop through all the belief simplex vertices and add the
  vectors at these points to the list.  Only adds the vector if they
  are not already in the list and returns the number of vectors that
  were added. Essentially this routine just calls addVectorAtBeliefQ()
  for each simplex corner.  */
extern AlphaList addVectorAtBeliefQ( AlphaList list, double *belief,
                                     AlphaList *projection,
                                     int save_witness_point,
                                     double epsilon );
 
/* This initializes the given list with vectors that are constructed
  from the projection sets sent in at the belief simplex corners.
  Will loop through all the belief simplex vertices and add the
  vectors at these points to the list.  Only adds the vector if they
  are not already in the list and returns the number of vectors that
  were added. Essentially this routine just calls addVectorAtBeliefQ()
  for each simplex corner.  */
extern int initWithSimplexCornersQ( AlphaList list, 
                                    AlphaList *projection,
                                    int save_witness_point,
                                    double epsilon );

/* Will generate 'num_points' random belief points and add the vectors
  at these points to the list, if they are not already there.  */
extern int initWithRandomBeliefPointsQ( AlphaList list, 
                                        int num_points,
                                        AlphaList *projection,
                                        int save_witness_point,
                                        double epsilon );

/* For algorithms that search belief space incremntally (e.g.,
   witness, two-pass) adding vectors, we usually have the ability to
   initialize the set with vectors known to be in the final
   parsimonious set. This routine encapsulates all the ways in which
   this set could be initialized.  This includes checking the simplex
   corners and optionally checking an arbitrary set of random points.
   It uses the 'param' argument to decided how to initialize the
   set. This routine returns the number of vectors added.  */
extern int initListSimpleQ( AlphaList list, 
                            AlphaList *projection,
                            PomdpSolveParams param );
     
extern int shouldTerminateEarly( AlphaList list, 
						   PomdpSolveParams param );

#endif
