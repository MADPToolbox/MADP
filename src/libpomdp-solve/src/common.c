
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    common.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: common.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/common.c,v $
 *    $Revision: 1.7 $
 *    $Date: 2005/01/25 21:20:46 $
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
 *   Many algorithms share some common operations to solve a POMDP.
 *   This file is meant to combine the common elements which appear in
 *   more than one algorithm.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"
#include "alpha.h"
#include "params.h"
#include "projection.h"
#if 0 // exclude solvers
#include "enumeration.h"
#include "linear-support.h"
#include "two-pass.h"
#include "witness.h"
#include "inc-prune.h"
#endif
#include "random.h"
#include "common.h"

/* These two variables comprise a little tricky business that goes on
   for creating alpha vectors from belief points.  Since this is a
   common operation, we do not want to have to allocate memory each
   time we need to do this calculation.  Therefore, we will use these
   as global temporary work spaces.  Essentially, we want a vector for
   each action, but as we build the vectors, we also want to keep
   other information about it.  This information can be kept in a
   AlphaList node so we will have one of these for each action.  We
   create an array gCurAlphaVector so we can easily access them.
   Howevere, it will also prove useful to be able to deal with these
   vectors a an normal AlphaList (e.g., for finding the maximal
   vector.) For this reason we define gCurAlphaHeader to be a header
   node which will contain all those alpha vectors.  All this is all
   set up in the routine allocateCommonTempMemory() */
AlphaList gCurAlphaHeader;
AlphaList *gCurAlphaVector;

/**********************************************************************/
void 
relinkObsSources( AlphaList list ) 
{
  /* 
     Changes the obs_source array from pointing at projection nodes to
     pointing at the actual sources of the projection vectors in the
     previous iteration's alpha list.  During construction of the Q_a
     sets we need the obs_source array to point to the projection, but
     we will be destroying these sets.  However, the projection
     elements themselves have a pointer into the previous alpha list
     for which they were created from. Before destroying the projection
     sets, we want to redirect the choice pointers to their previous
     alpha list sources.  This is the basis for coming up with the
     policy graph. 
  */
  int z;

  Assert( list != NULL, "List is NULL" );
  
  if ( list->length < 1 )
    return;

  list = list->head;
  while ( list != NULL ) { 
    
    for ( z = 0; z < gNumObservations; z++ )
      if ( list->obs_source[z] != NULL )
        list->obs_source[z] = list->obs_source[z]->prev_source;

    list = list->next;
  }  /* while */

}  /* relinkObsSources */
/**********************************************************************/
void 
initCommon(  ) 
{
  /*
    Many places in the code can use temporary memory storage for
    calculations. This just allocates some useful data structures for
    this purpose.
  */
  int a;
   
  /* We want to have an AlphaList node for each action to temporarily
     hold all necessary information as we build vectors from belief
     points. In addition, we will want to treat this as a list.  We
     thus create an array of these nodes so we can access them as an
     array, but then also create a header node for them to be able to
     access them as a list. */
   gCurAlphaVector 
     = (AlphaList *) XMALLOC ( gNumActions
                              * sizeof( *gCurAlphaVector ));
   for ( a = 0; a < gNumActions; a++ ) {
     gCurAlphaVector[a] = newAlphaNode( newAlpha(), a );
     gCurAlphaVector[a]->obs_source = newObsSourceArray();
   } /* for a */

   /* Make the array of nodes look like a list. */
   gCurAlphaHeader = newAlphaList();
   for ( a = 0; a < gNumActions; a++ )
     appendNodeToAlphaList( gCurAlphaHeader, gCurAlphaVector[a] );
}  /* initCommon */
/**********************************************************************/
void 
cleanUpCommon(  ) 
{
  /*
    Free up any temporary memory that was allocated.
  */

  /* This will get rid of the individual nodes, the arrays they
     contain and the header as well. */
  destroyAlphaList ( gCurAlphaHeader );
  XFREE( gCurAlphaVector );

}  /* cleanUpCommon */
/**********************************************************************/
 
/**********************************************************************/
/**************    Lower Level Solution Routines      *****************/
/**********************************************************************/

/**********************************************************************/
double 
bellmanError( AlphaList prev_list, 
	      AlphaList cur_list,
	      PomdpSolveParams param ) 
{
/*
  Computes the Bellman residual between two successive value
  funcitons, finding the point of maximal difference between the two
  sets. 


*/
  AlphaList vector;
  double max_diff, diff;

  max_diff = -1.0 * HUGE_VAL;

  for(  vector = cur_list->head;
        vector != NULL;
        vector = vector->next ) {

    /* We assume that not finding a region point is "zero" error, and
	  that if we find a difference, then findRegionPoint() will set
	  "diff" variable appropriately.  */
    if ( ! findRegionPoint( vector->alpha, prev_list, 
                          gTempBelief, &diff, param )) {

	 diff = 0.0;
    } /* if LP found a region point */
    
      /* We want to keep track of the maximal difference between the
         two sets so we know the true error. */
      max_diff = Max( max_diff, diff );

  } /* for vector */

  /* If there are negative rewards, then we must also compare each old
     vector to the new vectors. This also requires that there was no
     initial value function used (since it could have started with a
     negative value (which we do not check for.) */
  if (( ( gMinimumImmediateReward >= 0.0 )
        || ( gRequireNonNegativeRewards ))
      && ( param->initial_policy == NULL ))
    return ( max_diff );

  /* Otherwise, check the old against the new. */

  for(  vector = prev_list->head;
        vector != NULL;
        vector = vector->next ) {
    
    /* We assume that not finding a region point is "zero" error, and
	  that if we find a difference, then findRegionPoint() will set
	  "diff" variable appropriately.  */
    if ( ! findRegionPoint( vector->alpha, cur_list, 
					   gTempBelief, &diff, param )) {
      
	 diff = 0.0;
	 
    } /* if LP found a region point */
    
    /* We want to keep track of the maximal difference between the
	  two sets so we know the true error. */
    max_diff = Max( max_diff, diff );
      
  } /* for vector */

  return ( max_diff );

}  /* bellmanError */

/**********************************************************************/
int 
bestAlphaForBeliefQ( AlphaList node, double *b, 
		     AlphaList *projection,
		     double epsilon ) 
{
/*
  Constructs the alpha vector for the point 'b' using the projection
  sets for a particular action.  It will use an existing AlphaList
  node with allocated 'alpha' and 'obs_source' fields.' Returns TRUE
  if the node had its values set and FALSE if there was a problem.
*/
   AlphaList best_proj_vector;
   int i, z;
   double best_value;
   
   /* In theory, for a properly specified POMDP, it should not be
	 possible for all the projections to be NULL, since there has to be
	 some observation that is possible. However, just as a sanity check,
	 we will track to ensure this is true using this flag. */
   int non_null_proj = 0;

   Assert ( ( node != NULL ) 
            && ( b != NULL )
            && ( projection != NULL )
            && ( node->alpha != NULL)
            && ( node->obs_source != NULL ),
            "Bad (NULL) parameter(s)." );

   /* Initialize the alpha vector to all zeroes. */
   for ( i = 0; i < gNumStates; i++ ) 
     node->alpha[i] = 0.0;
     
   /* Now pick out the best vector for the projection set for each
      observation.  The best overall vector is just the sum of these
      individual best vectors. */
   for ( z = 0; z < gNumObservations; z++ ) {

     if ( projection[z] != NULL ) {

	  /* Find the best vector for all the observation 'z' projections.
		If projection[z] is NULL, then this returns NULL. */
	  best_proj_vector = bestVector( projection[z], b, 
							   &best_value, epsilon  );

	  non_null_proj = 1;  /* The sanity check flag gets set to 'ok' */	  
	}

	else {
	  /* By defnition, if projection[z] is NULL, then this means
        that the observation is not possible for this action. In
        this case, that observation will not contribute anything to
        the value of the state, so we can safely skip it.  Since it
        is impossible for all observations not to occur, we don't
        have to worry about all the projection[z] being NULL. Note
        that we also need to set the obs_source to NULL to indicate
        this. */
       node->obs_source[z] = NULL;
       continue;
     } /* if observation not possible */
     
     /* We want to see where each projection source came from and set
        this vectors obs_choice to that vector.  This gives us the
        policy graph information we desire. */
     node->obs_source[z] = best_proj_vector;

     /* Now add this best projection vector's component values to the
        components in the node. */
     for ( i = 0; i < gNumStates; i++ ) 
       node->alpha[i] += best_proj_vector->alpha[i];
     
     /* Note that the immediate rewards have already been taken into 
        account for the projection vectors. */
     
   }  /* for z */

   /* Here is where we put the sanity check. */
   Assert ( non_null_proj != 0, "All projections are NULL." );
   
   return ( TRUE );
}  /* bestAlphaForBeliefQ */
/**********************************************************************/
int 
setBestAlphaForBeliefQ( double *b, AlphaList *projection, double epsilon ) 
{
  /*
    Just uses the bestAlphaForBeliefQ() routine, but uses the global
    temporary variable gCurAlphaVector[0] to hold it.
  */
  
  /* Note that we are using gCurAlphaVector[0] just for the storage
     and that this does not necessarily mean we are computing for
     action '0'. */
  return ( bestAlphaForBeliefQ( gCurAlphaVector[0], b, projection,
                                epsilon ));
   
}  /* setBestAlphaForBeliefQ */
/**********************************************************************/
int 
setBestAlphasForBeliefV( double *b, AlphaList **projection, double epsilon )
{
  /*
    Loops through each action and constructs the alpha vector for the point
    'b' for each action.  It sets the global variable gCurAlphaVector
    which can then be used for other purposes (e.g., adding a vector
    to the list, finding the best action for the belief point, etc.)
    Returns TRUE if all vectors were created successfully and FALSE if
    any one of them had a problem.
  */
  int a;
  int result = TRUE;

  Assert( b != NULL && projection != NULL, 
          "Bad (NULL) parameter(s)." );

  for( a = 0; a < gNumActions; a++ )
    result = result
      && bestAlphaForBeliefQ( gCurAlphaVector[a], b, 
                              projection[a], epsilon );
   
  return ( result );
}  /* setBestAlphasForBeliefV */
/**********************************************************************/
double 
oneStepValue( double *b, AlphaList **projection,
	      AlphaList *best_vector, double epsilon ) 
{
  /*
    This routine will first create all of the alpha vectors (one for
    each action) for this point in the global array gCurAlphaVector,
    then it will determine which one is best for this point.  It will
    return the best value and set the parameter 'action' to be the action
    that was best.  
    
    If there are ties...(they are currently deterministically broken)
    
    Assumes gValueType is reward (which is true when rewards are
    accessed through the getImmediateReward() routine in global.c.)
  */
  double best_value;
  
  /* Construct the vector for each action and put them in global
     array gCurAlphaVector and gCurAlphaHeader list.
  */
  setBestAlphasForBeliefV( b, projection, epsilon );
  
  /* Use the global array as a list and get the best one. */
  *best_vector = bestVector( gCurAlphaHeader, b, 
                             &best_value, epsilon );
  
  return( best_value );
}  /* oneStepValue */
/**********************************************************************/
AlphaList 
makeAlphaVector( AlphaList new_alpha_list, 
		 AlphaList **projection,
		 double *b, double epsilon ) 
{
  /* This routine will actually create the new alpha vector for the
     point 'b' sent in.  It will add the vector to the list if it is not
     already there, and either way it will return the pointer into
     new_alpha_list for the vector.
     
     It first constructs all the vectors (for each action), then it finds
     which one is best (via dot product) finally it checks to see if
     the vector is in the list or not, and adds if if it isn't.
  */
  AlphaList best_vector, new_alpha_node;
  
  /* This has the effect of creating the vectors in the global array
     gCurAlphaVector and returning an AlphaList pointer to the one
     that was best. We aren't really interested in the value here. */
  oneStepValue( b, projection, &best_vector, epsilon );
  
  /* See if this vector is already in the list or not. */
  new_alpha_node = findAlphaVector( new_alpha_list, 
                                    best_vector->alpha,
                                    epsilon );
  
  if ( new_alpha_node != NULL )
    return ( NULL );

  /* Otherwise it isn't in the list yet. Here we are essentially just
     copying the stuff from the gCurAlphaVector temporary space to
     something we can use to put into a list.  Then we add it to the
     list. */
  new_alpha_node
    = appendDuplicateNodeToAlphaList( new_alpha_list, best_vector );

  return( new_alpha_node );

}  /* makeAlphaVector */
/**********************************************************************/
AlphaList 
addVectorAtBeliefQ( AlphaList list, 
		    double *belief,
		    AlphaList *projection,
		    int save_witness_point ,
		    double epsilon ) 
{
  /*
    This routine will construct the vector for the belief point sent in
    and add this vector to 'list' if it is better than all the other
    vectors at this point.  The routine returns TRUE if the vector ws
    added.  Note that this routine will *not* remove vectors from
    'list'.  It assumes that anything in list has been demonstrated to
    be the best vector for at least some belief point.  This does this
    considering only the action for the projections sent in.
    
    Returns a pointer to the new node added.
  */
  AlphaList node;

  Assert ( ( list != NULL )
           && ( belief != NULL )
           && ( projection != NULL ),
           "Bad (NULL) parameter(s)." );

  /* Construct the vector for this point. Note that we are using
     gCurAlphaVector[0] just for the storage and that this does
     not necessarily mean we are computing for action '0'. */
  setBestAlphaForBeliefQ( belief, projection, epsilon );

  /* So we know that at this point, the vector stored in
     gCurAlphaVector[0] is the absolute best vector. The only question
     now is whether or not this vector is already in the list or
     not. */
  
  node = findAlphaVector( list, gCurAlphaVector[0]->alpha,
                          epsilon );

  if ( node != NULL )
    return ( NULL );

  /* This will make a copy of the node with the vector and append it
     to the list. */
  node = appendDuplicateNodeToAlphaList( list, gCurAlphaVector[0] );

  /* Want every vector we add to have the proper action set. Since the
     projection sets have the action stored, we can use this to tell
     what action we are processing. */
  node->action = projection[0]->action;

  /* If we have specified the use-witness-points option, then we will
     save this point as a witness for this vector. */
  if ( save_witness_point == TRUE )
    addWitnessToAlphaNode( node, belief );

  return ( node );

}  /* addVectorAtBeliefQ */
/**********************************************************************/
int 
initWithSimplexCornersQ( AlphaList list, AlphaList *projection,
			 int save_witness_point,
			 double epsilon ) 
{
  /*
    This initializes the given list with vectors that are constructed
    from the projection sets sent in at the belief simplex corners.

    Will loop through all the belief simplex vertices and add the
    vectors at these points to the list.  Only adds the vector if they
    are not already in the list and returns the number of vectors that
    were added. Essentially this routine just calls addVectorAtBeliefQ()
    for each simplex corner. 
  */
  int i;
  int num_added = 0;

  Assert( list != NULL && projection != NULL,
          "Bad (NULL) parameter(s)." );
  
  /* We will actually need a belief point to generate the vector, so
     we will initialize it to all zeroes and set each component to 1.0
     as we need it. */
  for( i = 0; i < gNumStates; i++ ) 
    gTempBelief[i] = 0.0;
  
  for( i = 0; i < gNumStates; i++ ) {
    
    /* In normal initWithSimplexCorners() we loop over the full list
       to find the vector that has the highest value for component
       'i'.  Here we just need to see if the vector generated from
       this simplex corner is any better than what we have. */
    
    /* Set this so we actually have a simplex corner in 'b'. */
    gTempBelief[i] = 1.0;
    
    /* Note that if we are using the option of saving witness points,
       this addVectorAtBeliefQ() will do this. */
    if ( addVectorAtBeliefQ( list, gTempBelief, projection,
                             save_witness_point, epsilon  ) != NULL )
      num_added++;

    /* Clear the 'i'th component so we maintain a belief corner point
        during i+1. */
    gTempBelief[i] = 0.0;
    
   }  /* for i */
  
  return ( num_added );

}  /* initWithSimplexCornersQ */
/**********************************************************************/
int 
initWithRandomBeliefPointsQ( AlphaList list, int num_points,
			     AlphaList *projection,
			     int save_witness_point,
			     double epsilon) 
{
  /*
    Will generate 'num_points' random belief points and add the vectors
    at these points to the list, if they are not already there.
  */
  int i;
  int num_added = 0;

  Assert ( ( list != NULL )
           && ( projection != NULL ),
           "Bad (NULL) parameter(s)." );

  if ( num_points < 1 )
    return ( 0 );

  for( i = 0; i < num_points; i++ ) {
    
    /* Get a random belief point, uniformly distributed over the
       belief simplex. */
    setRandomDistribution( gTempBelief, gNumStates );
    
    /* Note that if we are using the option of saving witness points,
       this addVectorAtBeliefQ() will do this. */
    if ( addVectorAtBeliefQ( list, gTempBelief, projection,
                             save_witness_point, epsilon ) != NULL )
      num_added++;

  }  /* for i */
  
  return ( num_added );

}  /* initWithRandomBeliefPointsQ */
/**********************************************************************/
int 
initListSimpleQ( AlphaList list, 
		 AlphaList *projection,
		 PomdpSolveParams param ) 
{
  /*
    For algorithms that search belief space incremntally (e.g., witness,
    two-pass) adding vectors, we usually have the ability to initialize
    the set with vectors known to be in the final parsimonious set. This
    routine encapsulates all the ways in which this set could be
    initialized.  This includes checking the simplex corners and
    optionally checking an arbitrary set of random points.  It uses the
    'param' argument to decided how to initialize the set. This routine
    returns the number of vectors added.
  */

  /* Initialize using simplex corners. Assume we always want
     this. Note that if we only get one vector, then this vector is
     maximal at every belief simplex vertex and thus must be maximal
     everywhere. For this case, we can bail out right here knowing
     that the Q-function list must be of size '1'. */
  if ( initWithSimplexCornersQ( list, 
                                projection,
                                param->use_witness_points,
                                param->alpha_epsilon ) < 2 )
    return( list->length );
  
  /* Use random points to initialize the list, but this will only do
     something if param->init_rand_points > 0 */
  initWithRandomBeliefPointsQ( list, 
                               param->alg_init_rand_points,
                               projection,
                               param->use_witness_points,
                               param->alpha_epsilon );

  return ( list->length );

}  /* initListSimple */
/**********************************************************************/
int
shouldTerminateEarly( AlphaList list, PomdpSolveParams param )
{
  /* This should be called at points in the solution procedures where
	you might wantr to terminate early based on onno-algorithmic
	decisions, e.g., size of solution has grown too large. 
  */

  /* FIXME: Currently, this feature is not implemented, so we never
	want to terminate early.
  */

  return 0;
}
