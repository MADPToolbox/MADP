
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    neighbor.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: neighbor.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/neighbor.c,v $
 *    $Revision: 1.2 $
 *    $Date: 2004/10/10 03:44:54 $
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
 *   Routines for processing "neighbors" used in the and two-pass witness
 *   algorithms.
 * 
 *   A "neighbor" is a vector that differs from another by a single
 *   obs_source vector.  So if you consider that there are M possible
 *   settings for obs_source[z] (where M is the size of the previous
 *   iterations alpha list), then there are M*Z neighbors, because each
 *   of 'Z' vectors could take on one of 'M' values.
 * 
 *   We will store a neighbor as the alpha vector, a pointer to the
 *   vector is was created from, and information that tells how its
 *   obs_source array differs from this one.  In particular, it specifies
 *   the observation 'z' where the vector differed, and a pointer into
 *   the projection set that is the difference.
 * 
 *   The motivation here is that we do not have to create an entire
 *   obs_source array for each item.  We can easily reconstruct
 *   the obs_source of a neighbor by first copying the previous vector
 *   choice array and applying the deltas about how it differs.  In terms
 *   of the fields discussed below this would be:
 * 
 *   neighbor->obs_source = duplicateObsSource( prev_source->obs_source )
 *   neighbor->obs_source[neighbor->obs] = neighbor->first_source;
 * 
 *   We will use an AlphaList to hold the items, but we will use
 *   it slightly differently from the way it is used.  The only fields we
 *   use will be:
 * 
 *     int obs;
 *     double *alpha;
 *     AlphaList prev_source;
 *     int mark;
 *     AlphaList first_source;
 * 
 *   with the following meanings.
 * 
 *   prev_source - the vector the neighbor was created from.
 * 
 *   obs - the observation where this has a different obs_source than
 *   prev_source. 
 * 
 *   first_source - the projection pointer that is different. 
 * 
 *   mark - used to indicate whether the neighbor has been processed or
 *   not.  It turns out that keeping the old vectors around is useful in
 *   preventing duplication of work. Since a vector is a neighbor of M*Z
 *   vectors, there are potentially M*Z vectors which could cause the
 */


#include <stdio.h>
#include <stdlib.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"
#include "alpha.h"
#include "params.h"
#include "neighbor.h"

/**********************************************************************/
int 
addNeighbor( AlphaList list, 
	     double *neighbor,
	     AlphaList source, int z, 
	     AlphaList proj_z,
	     int domination_check,
	     double epsilon ) 
{
  /*
    Adds a copy of the given neighbor to the list, but only if:
    
    1) The neighbor isn't already in the list.
    2) The neigbhbor isn't dominated by an existing neighbor.
    
    Returns TRUE if copy of neighbor is added and false otherwise.
    
    Other params:
    
    'source' is the vector that generated this neighbor.
    
    'z' is the observation for which this neighbor differs from the
    source.

    'proj_z' is the vector that is different.
    
  */
  AlphaList node;

  /* First see if neighbor exists in list. */
  node = findAlphaVector( list, neighbor, epsilon );

  /* If it exists, bail out. */
  if ( node != NULL )
    return ( FALSE );

  /* If we are using the simple domination checks, we can use it to
     check before the addition of this vector and also to remove
     neighbors already in the list that this one might dominate. */
  if ( domination_check ) {

    /* Now see if this neighbor is dominated by a vector already in the
       list (using the simple domination check.). */
    if ( dominatedAlphaList( neighbor, list ))
      return ( FALSE );

    /* Now we know that we can add this neighbor to the list.  However,
       we can also eliminate any neighbors in the list that are now
       dominated by this neighbor.  Because we keep all neighbors around
       and just mess with their 'mark' field. Note that this routine
       *does not * clear the 'mark' field.  Therefore, if list items
       are already marked because they have been processed, this will
       result in 'mark'ed vector thatv are not necessarily dominated.
       However, this is exactly what we want anyway. Anything that is
       marked we want to keep marked and anythingthat is dominated we
       want to mark. */
    markDominatedAlphaList( neighbor, list );

  } /* if gDominationCheck */

  /* Finally, we can add this neighbor to the list. */
  node = newAlphaNode( neighbor, UNINITIALIZED );
  node->prev_source = source;
  node->obs = z;
  node->first_source = proj_z;

  /* Put at at the beginning so that all 'unmarked' vector are at the
     front. */
  prependNodeToAlphaList( list, node );

  return ( TRUE );
}  /* addNeighbor */
/**********************************************************************/
int 
addAllNeighbors( AlphaList list, 
		 AlphaList node, 
		 AlphaList *projection,
		 int domination_check, 
		 double epsilon ) 
{
  /*
    Adds all neighbors of the vector in the node into the list.
  */
  int i, z;
  AlphaList proj_z;
  int num_added = 0;
  double *neighbor;

  Assert( list != NULL && node != NULL && projection != NULL,
          "Bad (NULL) parameter(s)." );

  /* To find all neighbors we have to loop over all observations and
     projection vectors for that observation. */
  for ( z = 0; z < gNumObservations;  z++ ) {
    
    proj_z = projection[z]->head;
    while ( proj_z != NULL ) {
      
      
      /* Note that by enumerating over all z and proj_z, we will
         actually make the vector itself Z times.  We can easily
         eliminate this one by checking for this case and skipping
         it. */
      if ( node->obs_source[z] == proj_z ) {
        proj_z = proj_z->next;
        continue;
      } /* if just getting the vector itself */
      
      /* First we will construct the vector in temporary storage so
         we can use it to check whether we need to add it or
         not. Note that we just apply the adjustments using the
         projection vector pointers. Alternatively, we could add a
         loop over observations and explicitly calculate it using
         the projection pointer sfor all observations. */
      neighbor = newAlpha();

      for ( i = 0; i < gNumStates; i++ )
        neighbor[i] 
          = node->alpha[i] - node->obs_source[z]->alpha[i]
          + proj_z->alpha[i];
      
      /* Now we have the neighbor vector which we will add to the
         list.  However, we can avoid adding it if it is already
         in the list or if it is dominated by an existing vector. */
      if ( addNeighbor( list, neighbor, 
                        node, z, proj_z, 
                        domination_check, epsilon  ))
        num_added++;
      else
        XFREE ( neighbor );
      
      proj_z = proj_z->next; 
    } /* while proj_z != NULL */
    
  } /* for z */

  return ( num_added );

}  /* addAllNeighbors */
/**********************************************************************/
int 
addAllNeighborsFromList( AlphaList dest_list, 
			 AlphaList list,
			 AlphaList *projection,
			 int domination_check,
			 double epsilon ) 
{
  /*
    Adds all the "neighbors" of all the vectors in the list sent in into
    the dest_list list. Checks to make sure the neighbor isn't already in
    the list.
    
    Returns the number of neighbors added.
  */
  int num_added = 0;

  Assert( dest_list != NULL && list != NULL && projection != NULL,
          "Bad (NULL) parameter(s)." );
  
  list = list->head;
  while ( list != NULL ) {

    num_added += addAllNeighbors( dest_list, list, projection,
                                  domination_check, epsilon  );

    list = list->next;
  } /* while list != NULL */

  return ( num_added );
}  /* addAllNeighborsFromList */
/**********************************************************************/

