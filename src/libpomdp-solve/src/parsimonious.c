
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    parsimonious.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: parsimonious.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/parsimonious.c,v $
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

/*
 *   All the routines that help to create a parsimonious representation,
 *   including optimizations and top-level pruning routines.  
 *   
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"
#include "alpha.h"
#include "stats.h"
#include "params.h"
#include "region.h"
#include "random.h"
#include "parsimonious.h"

/**********************************************************************/
/*******  Routines that are optimizations to speed up pruning   *******/
/**********************************************************************/

/**********************************************************************/
int 
isEmptyRegionSimpleCheck( AlphaList list, 
			  double *alpha,
			  double epsilon,
			  int domination_check ) 
{
  /*
    There are a number of simple checks that can be made to determine if
    the region will be non-empty.  First, if the 'alpha' vector is
    already in the list.  Second, if something in the list
    component-wise dominates the vector.  This routine does these checks
    and returns TRUE if region can easily be determined to be empty and
    FALSE if the simple check reveals nothing.
  */
  AlphaList node;

  if (( list == NULL )
      || ( alpha == NULL ))
    return ( TRUE );

  /* Check if item is already in the list. */
  node = findAlphaVector( list, alpha, epsilon );
  
  if ( node != NULL )
    return ( TRUE );

  /* See if it is component-wise dominated. */
  if ( domination_check
       && dominatedAlphaList( alpha, list ))
    return ( TRUE );

  return ( FALSE );
 
}  /* isEmptyRegionSimpleCheck */
/**********************************************************************/
void 
markBestAtSimplexVertices( AlphaList list, 
			   int save_witness_points, 
			   double epsilon ) 
{
  /* 
     Sets the 'mark' field of each vector that dominates at some belief
     simplex vertex.
     
     A difference between this routine and initWithSimplexCornersQ()
     is that this vector does not have to construct the vectors for the
     points, it simply picks them out of the list provided.  This is
     used in the prune algorithm to initialize the list.
     
     Will loop through all the belief simplex vertices and find the
     best vectors at these points from the list.  
  */
  AlphaList node;
  double best_value;
  int i;

  Assert( list != NULL, "Bad (NULL) parameter(s)." );

  /* If the list is empty, we really shouldn't be calling this
     routine.  CHances are something went wrong somewhere. */
  Assert ( list->length != 0, "Cannot mark an empty list." )

  /* We will actually need a belief point to generate the vector, so
     we will initialize it to all zeroes and set each component to 1.0
     as we need it. */
  for( i = 0; i < gNumStates; i++ ) 
    gTempBelief[i] = 0.0;
  
  for( i = 0; i < gNumStates; i++ ) {
    
    /* Set this so we actually have a simplex corner in 'b'. */
    gTempBelief[i] = 1.0;

    node = bestVector( list, gTempBelief, &best_value, epsilon );

    /* It is possible that this vector had already been marked, so
       only need to mark it and consider whether to save a witness
       point for it if it is not yet marked. */
    if ( node->mark == FALSE ) {

      node->mark = TRUE;

      if ( save_witness_points == TRUE )
        addWitnessToAlphaNode( node, gTempBelief );
    
    } /* if node->mark == FALSE */

    /* Clear the 'i'th component so we maintain a belief corner point
       during i+1. */
    gTempBelief[i] = 0.0;
    
   }  /* for i */
  
}  /* markBestAtSimplexVertices */
/**********************************************************************/
void 
markBestAtRandomPoints( AlphaList list, 
			int num_points, 
			int save_witness_points,
			double epsilon ) 
{
  /* 
     Will generate 'num_points' random belief points and mark the
     vectors at these points to the list.  
  */
  AlphaList node;
  double best_value;
  int i;
  
  Assert (  list != NULL, "Bad (NULL) parameter(s)." );
  
  if ( num_points < 1 )
    return;
 
  for( i = 0; i < num_points; i++ ) {
    
    /* Get a random belief point, uniformly distributed over the
       belief simplex. */
    setRandomDistribution( gTempBelief, gNumStates );
    
    node = bestVector( list, gTempBelief, &best_value, epsilon  );
    
    /* It is possible that this vector had already been marked, so
       only need to mark it and consider whether to save a witness
       point for it if it is not yet marked. */
    if ( node->mark == FALSE ) {
      
      node->mark = TRUE;
      
      if ( save_witness_points == TRUE )
        addWitnessToAlphaNode( node, gTempBelief );
      
    } /* if node->mark == FALSE */
    
  }  /* for i */
  
}  /* markBestAtRandomPoints */
/**********************************************************************/



/**********************************************************************/
/*******   Main routines for finding parsimonious sets   **************/
/**********************************************************************/

/**********************************************************************/
int 
isEpsilonApproximation( AlphaList test_list,
			AlphaList orig_list,
			double *max_diff,
			PomdpSolveParams param  ) 
{
/*
  Determines where or not test_list is an epsilon approximation of the
  original list.  Returns TRUE if there is no place where orig_list is
  more than epsilon better than test_list and FALSE if test_list is
  not an epsilon approximation.
*/
  AlphaList vector;
  double diff;

  *max_diff = 0.0;
  
  /* We check every vector in the original list to see if it yields a
     value that is better than the test_list because this tells us
     what we need to know. */
  for(  vector = orig_list->head;
        vector != NULL;
        vector = vector->next ) {

    /* Simple optimization is to see if that vector is already in the
       list, because then we know that there is no region point and
       the difference is zero. Because this is used to compare sets
       that will often be overlapping, this is a good optimization. We
       use a very small epsilon because this optimization is focused
       on when the values really should be identical. */
    if ( queryAlphaList( test_list, vector->alpha, 
                         SMALLEST_PRECISION ))
      continue;

    if ( findRegionPoint( vector->alpha, test_list, 
                          gTempBelief, &diff, param )) {

      /* We want to keep track of the maximal difference between the
         two sets so we know the true error. */
      *max_diff = Max( *max_diff, diff );

      if ( diff > param->prune_epsilon )
        return ( FALSE );

    } /* if LP found a region point */
    
  } /* for vector */

  return ( TRUE );

} /* isEpsilonApproximation */
/**********************************************************************/
int 
epsilonPrune( AlphaList list, 
	      PomdpSolveParams param  ) 
{
  /*
    This is the first implementation of a real epsilon pruning algorithm
    and is correct, but not that efficient.  It operates by determining
    whether removing a vector leaves a set of vectors that is still
    epilson approximates the original.  This requires a copy of the
    original set.  Note that although the input set may not be minimal,
    it still represents the "exact" value function and we can use it for
    comparison. 
  */
  int num_pruned = 0;
  AlphaList orig_list, test_vector;
  double diff;

  /* Keep track of the actual computed difference between the returned
     pruned set and the original set sent in. */
  double max_diff = 0.0;

  /* Because we will need to check the sets that result from removing
     vectors against the "true" set, we need to always maintain the
     true set. */
  orig_list = duplicateAlphaList( list );
  
  /* Unmark all vector in the list so we can keep track of which ones
     we have checked.  Because the set will be dynamically changing as
     we test, it is easiest to just marke them as we use them rather
     than have to worry about which ones we remove and where we put
     them back. */
  clearMarkAlphaList( list );

  /* Need to try each vector. */
  while( sizeUnmarkedAlphaList( list ) > 0 ) {

    /* Extract an unchecked vector and mark it. */
    test_vector = extractUnmarkedVector( list );
    test_vector->mark = TRUE;

    /* If the resulting list with the vector removed is an epsilon
       approximation then we can just get rid of it.  Otherwise we
       need to add it back into the set. This routine returns the
       actual computed maximal difference between the two sets. */
    if ( isEpsilonApproximation( list, orig_list, 
                                 &diff, param )) {
      destroyAlphaNode( test_vector );
      num_pruned++;
      max_diff = Max( max_diff, diff );

    } /* if set is still an epsilon approximation */

    else
      enqueueAlphaNode( list, test_vector );
    
  } /* while no more unmarked vectors */

  param->epsilon_diff_of_last_prune = max_diff;

  return( num_pruned );

}  /* epsilonPrune */
/**********************************************************************/

/**********************************************************************/
/*******   Main routines for epsilon approximate sets    **************/
/**********************************************************************/

/**********************************************************************/
int 
dominationCheck( AlphaList orig_list ) 
{
  /*
    Removes all vectors from the list that can be determined to have a
    non-empty region using only the simple component-wise domination
    check with some other vector in the list.
  */
  AlphaList list;

  Assert( orig_list != NULL, "List is NULL." );

  /* There is no way there can be anything to remove unless there are
     at least two elements. */
  if ( orig_list->length < 2 )
    return ( 0 );

  /* We will first set the 'mark' field of the list and then delete
     the 'mark'ed nodes.  So first we need to clear the 'mark' field,
     just in case they are set. */
  clearMarkAlphaList( orig_list );

  /* Now we simply go through each node in the list and mark any in
     the list that are dominated by it. Note that at some point in the
     domination check, the same vector will be comared to itself.  Since
     the dominated check does not consider a vector to dominate
     itself, we don't have to worry about it being inadvertently
     marked. */
  list = orig_list->head;
  while ( list != NULL ) {

    /* If we have already marked this vector as being dominated by
       something in the list, then there is no point in looking for
       vectors in the list which it dominates.  By transitivity of the
       domination check, any that this vector might dominated would
       have already been marked. */
    if ( list->mark != TRUE )
      markDominatedAlphaList( list->alpha, orig_list );

    list = list->next;
  } /* while list != NULL */
  
  return( removeMarkedAlphaList( orig_list ));
  
}  /* dominationCheck */
/**********************************************************************/
int 
normalPrune( AlphaList orig_list, PomdpSolveParams param ) 
{
  /* 
     Will use linear programming to prune the list to a unique
     parsimonious represenation.  If the save_points flag is set, then
     each vector in the resulting set (with a non-empty) region will
     have the witness point used to verify its non-empty region saved in
     the node containing the vector. Returns the number of nodes pruned.
     
     if the save_witness_points flag is TRUE, then for every useful
     vector found, we will also save the witness point that was found
     for this vector.
     
     If the init_num_random_points value is > 0, then it will preceed
     the LP computation with a check for useful vectors at random
     points. 

     This uses the scheme propose by Lark and White, which is mentioned
     in White's 1991 Operations Research POMDP survey article.
  */
  AlphaList new_alpha_list, cur_node, best_node;
  int num_pruned = 0;
  
  Assert( orig_list != NULL, "List is NULL." );

  /* Want to allow variations on the epsilon pruning.
     */
  /* We will mark the best node for each simplex vertex and ranodm
     point initialization, so first clear the 'mark' field. */
  clearMarkAlphaList( orig_list );
  
  /* First we select vectors using this simple test. This will
     only mark the best vectors. */
  markBestAtSimplexVertices( orig_list, 
                             param->use_witness_points,
                             param->alpha_epsilon );

  /* Use random points to initialize the list, but this will only do
     something if param->prune_init_rand_points > 0 */
  markBestAtRandomPoints( orig_list, 
                          param->prune_init_rand_points,
                          param->use_witness_points,
                          param->alpha_epsilon );

  /* Now we actually initialize the parsimonious list with those
     vectors found through the simpler checks. */
  new_alpha_list = extractMarkedAlphaList( orig_list );

  while ( orig_list->length > 0 ) {

    /* Remove a node from the original list. */
    cur_node = dequeueAlphaNode( orig_list );

    /* See if this node gives us a witness point that there must be a
       vector to be added to new list from original list. */
    if ( findRegionPoint( cur_node->alpha, new_alpha_list, 
                          gTempBelief, NULL, param )) {

      /* Note that the finding of a witness point does *not*
         necessarily mean that cur_node is the best vector at this
         point.  Since we only compare cur_node to the new list, we do
         not know whether there are vectors in the original list which
         might be even better still. */

      /* Therefore, we first put this node back into the list and then
         find the vector in the list that is maximal for this
         point. */
      enqueueAlphaNode( orig_list, cur_node );

      best_node = removebestVectorNode( orig_list, gTempBelief,
                                        param->alpha_epsilon );

      /* zzz Should this be addUniqueAlphaList()?  What if the list
         sent in has duplictaes?  Then the precision issue becomes
         important. This is theoretically not necessary since two
         equivalent vectors should not give a region point, but unless
         we can really ensure this , we should add the check here. */
      appendNodeToAlphaList( new_alpha_list, best_node );

      if ( param->use_witness_points )
        addWitnessToAlphaNode( best_node, gTempBelief );

    } /* If we did find a witness point. */

    /* Otherwise, no witness point was found which mean we can simply
       get rid of this node. */
    else {
      destroyAlphaNode( cur_node );
      num_pruned++;
    } /* else no witness point was found. */

  } /* while orig_list->length > 0 */

  /* Now the orig_list should be empty, so now we need to move the
     new_alpha_list into the original list. */
  orig_list->head = new_alpha_list->head;
  orig_list->tail = new_alpha_list->tail;
  orig_list->length = new_alpha_list->length;

  /* Now free up the header memory for the new list, since it was just
     being used as temporary storage. */
  new_alpha_list->head = NULL;
  new_alpha_list->tail = NULL;
  destroyAlphaList( new_alpha_list );

  return( num_pruned );
}  /* normalPrune */
/**********************************************************************/
int 
prune( AlphaList orig_list, 
       PurgeOption purge_option,
       PomdpSolveParams param ) 
{
  /*
    This routine just serves as a multiplexor for the particular type of
    pruning option specified. 
  */
  int num_pruned;

  switch ( purge_option ) {

  case purge_epsilon_prune:
    num_pruned = epsilonPrune( orig_list, param );
    break;

  case purge_prune:
  default:
    num_pruned = normalPrune( orig_list, param );
    break;

  } /* switch */
  
  return ( num_pruned );
  
}  /* prune */
/**********************************************************************/
void 
purgeAlphaList( AlphaList list, 
		PurgeOption purge_option,
		PomdpSolveParams param ) 
{
  /*
    Removes vectors from the list according to the purging option sent
    it.  It can do anything from nothing, to simple domination checks,
    to full blown 'pruning'.
    
    It needs to param structure because it uses some of those fileds to
    decide how to do the pruning.
  */
  Assert ( list != NULL, "List is NULL." );
  
  switch ( purge_option ) {
  case purge_dom:
    dominationCheck( list ); 
    break;
    
  case purge_prune:
  case purge_epsilon_prune:
    /* We assume that pruning always does a domination check first. */
    dominationCheck( list ); 
    prune( list, purge_option, param );
    break;
    
  case purge_none:
  default:
    /* Do nothing. */
    break;
  } /* switch */

 }  /* purgeAlphaList */
/**********************************************************************/
void 
purgeProjections( AlphaList **projection, 
		  PomdpSolveParams param ) 
{
  /*
    Runs the purgeAlphaList() routine on all the projection sets using
    the purging option for projections as set on the command line (or
    with default.)
  */
  int a, z;
  
  if ( projection == NULL )
    return;

  for ( a = 0; a < gNumActions; a++ ) 
    for ( z = 0; z < gNumObservations; z++ )

      /* If an observation is not possible, then we will have an empty
         projection list. Also, there is no need to purge a list of
         length 1. */
      if ( projection[a][z]->length > 1 )
        purgeAlphaList( projection[a][z], 
                        param->proj_purge,
                        param );
      
}  /* purgeProjections */
/**********************************************************************/
