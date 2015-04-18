
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    projection.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: projection.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/projection.c,v $
 *    $Revision: 1.4 $
 *    $Date: 2005/04/22 21:24:13 $
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
 *   Routines for constructing the projection sets {\bar \Gamma}^{a,z}_t.
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"
#include "alpha.h"
#include "projection.h"

void dumpProjections( AlphaList **projection );

/**********************************************************************/
AlphaList **
allocateAllProjections(  ) 
{
  /*
    Just creates and returns the storage to hold all the projection
    sets.  They will initially all be NULL. 
  */
  int a;
  AlphaList **projection;
  
  /* We need an arrary of projections for each action. */
  projection = (AlphaList **) XMALLOC( gNumActions 
                                      * sizeof( *projection ));
  
  /* For each action, we need a projection for each  observation. */
  for ( a = 0; a < gNumActions; a++ )
    projection[a] = (AlphaList *) XCALLOC( gNumObservations,
                                          sizeof( **projection ));
  
  return ( projection );

}  /* **allocateAllProjections */
/**********************************************************************/
void 
clearAllProjections( AlphaList **projection ) 
{
  /*
    Just removes the memory for the list themselves and not for the
    2D array which is holding them.
  */
  int a, z;
  
  if ( projection == NULL )
    return;

  /* First free the individual alpha vector lists. */
  for ( a = 0; a < gNumActions; a++ )
    for ( z = 0; z < gNumObservations; z++ ) {
      
      if ( projection[a][z] == NULL )
        continue;
      
      destroyAlphaList( projection[a][z] );
      projection[a][z] = NULL;
    } /* for z */

}  /* clearAllProjections */
/**********************************************************************/
void 
freeAllProjections( AlphaList **projection ) 
{
  /*
    Discards all the projection lists and memory associated with them.
  */
  int a;

  if ( projection == NULL )
    return;
  
  /* Deallocate the memory for the individual projections first. */
  clearAllProjections( projection );
  
  /* Deallocate the array of projections we have for each action. */
  for ( a = 0; a < gNumActions; a++ )
    free ( projection[a] );
  
  XFREE( projection );
  
}  /* freeAllProjections */
/**********************************************************************/
AlphaList 
projectVector( AlphaList node, int a, int z ) 
{
  AlphaList proj_node;
  double *alpha;
  int j, cur_state;

  Assert ( node != NULL,
           "Bad parameters" );

  /* If this observation is impossible, then there is no projection,
     so we return NULL. Note that we SHOULD NOT get here in the normal
     course of events, since the projectList() routine handles this
     case specially, which should result in this function *not* being
     called when the observation is impossible. This is especially
     important due to the assumption we make below about dividing the
     immediate reward into one piece per observation.
  */
  Assert ( gObservationPossible[a][z],
		 "Shouldn't be projecting vector when obs is not possible" );
  
  alpha = newAlpha();

  /* Set projection values */
  for ( cur_state = 0; cur_state < gNumStates; cur_state++) {
    alpha[cur_state] = 0.0;
    
    for ( j = P[a]->row_start[cur_state]; 
          j < P[a]->row_start[cur_state] +  P[a]->row_length[cur_state];
          j++ ) 

      alpha[cur_state] +=  P[a]->mat_val[j] 
        * getEntryMatrix( R[a], P[a]->col[j], z )
        * node->alpha[P[a]->col[j]];
    
    alpha[cur_state] *= gDiscount;
      
    /* Now we add a piece of the immediate rewards. This may seem a
       little odd to add only a portion of the immediate rewards
       here.  In fact, the actual values here don't make complete
       sense, but the effect will be that a vector from each
       observations' projection will sum to be the actual new alpha
       vector. Without adding this, we would need to add the extra
       step of adding the immediate rewards making the code not an
       nice. It turns out that adding this constant vector does not
       change any of the properties of the sets that we are
       interested in. IMPORTANT: Because of the way we define the
       projection set for impossible observations, we can also use
       the 1/|Z| weighting of rewards here.  If we did not define
       the impossible observation projections to exist at all then
       it would not enough to use gNumObservations in the
       denominator since some observations are not possible, meaning
       the sum will consist of less vectors than there are
       observations.  If this were the case we would need to use the
       precomputed total non-zero prob. observations for each
       action. */
    alpha[cur_state] 
      += getAdjustedReward( a, cur_state ) 
      / ((double) gNumObservations);
      
  }   /* for i */

  /* Create a node for this vector. Note that the action we want to
	associate with this vector should be the same as the original
	vector.  The 'a' defining how it was projected is an attribute of
	the projected list as a whole.  */
  proj_node = newAlphaNode( alpha, node->action );

  /* We will also store the action and observation for each
     individual vector.  This will come in handy when we enumerate
     vectors because we can easily identify which action and
     observation vector came from. */
  proj_node->obs = z;
  
  /* Indicate which vector it originated from. */
  proj_node->prev_source = node;

  return ( proj_node );

}  /* projectVector */
/**********************************************************************/
AlphaList 
projectList(  AlphaList list, int a, int z ) 
{
  /*
    Compute the back projection of the list sent in for a particular 
    action and observation.  It also takes the discounting into account.
    and part of the immediate reward.  It distributes the immediate
    reward (which is independent of the observation) evenly among all
    the projection sets, so that the addition of the vectors will
    incorporate the proper immediate reward.
  */
  AlphaList projection, temp;
  double *alpha;
  int cur_state;

  projection = newAlphaList();

  /* We put the action and observation in the list header so we can
     easily identify which projection a particular list is. */
  projection->action = a;
  projection->obs = z;

  if ( list == NULL )
    return ( projection );

  /* It is possible that a particular observation is impossible to
     observe for this given action and the possible resulting states.
     If this happens we will get a vector of all zeroes.  This vector
     of all zeroes is a bit different than what we want.  We want to
     say that there is no value function, not that the value is zero
     everywhere.  When an observation is impossible, we represent this
     with a single vector of 1/|Z| weighted immediate rewards with
     prev_source pointer set to NULL. */
  if ( ! gObservationPossible[a][z] ) 
  {
    
    alpha = newAlpha();
    for ( cur_state = 0; cur_state < gNumStates; cur_state++)
      alpha[cur_state] 
        = getAdjustedReward( a, cur_state ) 
        / ((double) gNumObservations);
    
    /* Make sure the projections will be added in the proper order. */
    temp = appendAlphaList( projection, alpha, a );
    temp->prev_source = NULL;
    temp->action = a;
    temp->obs = z;

    return ( projection );
  } /* if impossible observation */
  
  list = list->head;
  while ( list != NULL ) 
  {

    temp = projectVector( list, a, z );

    appendNodeToAlphaList( projection, temp );

    list = list->next;
  }  /* while list */
  
  return ( projection );
}  /* projectList */
/**********************************************************************/
void 
setAllProjections( AlphaList **projection,
		   AlphaList prev_alpha_list ) 
{
  /*
    Makes all the projected alpha vector lists, which amounts to a
    projected list for each action-observation pair. Stores this as a
    two dimensional array of lists where the first index is the action
    and the other is the observation.
    
    The 'impossible_obs_epsilon' specifies the tolerance to use when
    trying to determine whether or not a particulat observation is at
    all feabile.
  */
  int a, z;

  for ( a = 0; a < gNumActions; a++ ) {
    for ( z = 0; z < gNumObservations; z++ ) {
    
      projection[a][z] = projectList( prev_alpha_list, a, z );
    
    } /* for z */
  } /* for a */

}  /* setAllProjections */
/**********************************************************************/
AlphaList **
makeAllProjections( AlphaList prev_alpha_list ) 
{
  /*
    Makes all the projected alpha vector lists, which amounts to a
    projected list for each action-observation pair. Stores this as a
    two dimensional array of lists where the first index is the action
    and the other is the observation.  This allocates the space for the
    projections first.

    The 'impossible_obs_epsilon' specifies the tolerance to use when
    trying to determine whether or not a particulat observation is at
    all feabile.
  */
 
  AlphaList **projection;
  
  /* We cannot project nothing. */
  if ( prev_alpha_list == NULL )
    return ( NULL );

  projection = allocateAllProjections();
  
  setAllProjections( projection, prev_alpha_list );

  /* Uncomment this if you want to see all the projections in files. */
  /*   dumpProjections( projection ); */

  return ( projection );
}  /* *makeAllProjections */
/**********************************************************************/
void 
dumpProjections( AlphaList **projection ) 
{
  /*
    Dump all projections to a file stream.
  */
  int a, z;
  char filename[255];

  for ( a = 0; a < gNumActions; a++ )
    for ( z = 0; z < gNumObservations; z++ ) {

      sprintf( filename, "projection-a=%d-z=%d.alpha", a, z );
      saveAlphaList( projection[a][z], filename );

    } /* for z */

}  /* displayProjections */
/**********************************************************************/
void 
displayProjections( FILE *file, AlphaList **projection ) 
{
  /*
    Displays all projections to file stream.
  */
  int a, z;

  for ( a = 0; a < gNumActions; a++ )
    for ( z = 0; z < gNumObservations; z++ ) {

      fprintf( file, "Projection[a=%d][z=%d] ", a, z );
      displayAlphaList( file, projection[a][z] );

    } /* for z */

}  /* displayProjections */
/**********************************************************************/
void 
showProjections( AlphaList **projection ) 
{
  /*
    Displays all projections to stdout.
  */

  displayProjections( stdout, projection );

}  /* showProjections */
/**********************************************************************/




