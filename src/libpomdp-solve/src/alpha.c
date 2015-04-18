
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    alpha.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: alpha.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/alpha.c,v $
 *    $Revision: 1.6 $
 *    $Date: 2005/10/30 23:21:17 $
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
 *   Routines for alpha vectors and alpha vector lists.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"
#include "alpha.h"

/**********************************************************************/
/******************   Alpha Vector Routines      **********************/
/**********************************************************************/
double *
newAlpha( ) 
{
  /* 
     Allocate memory for an alpha vector, whose size is determined by
     the number of states.
  */
  return (  (double *) XMALLOC( sizeof( double ) * gNumStates ));
}  /* newAlpha */
/**********************************************************************/
double *
duplicateAlpha( double *alpha ) 
{
  /*
    Makes a copy of the alpha vector also allocating the memory for it. 
  */
  double *temp;
  int i;
  
  if ( alpha == NULL)
    return NULL;
  temp = newAlpha( );
  for ( i = 0; i < gNumStates; i++)
    temp[i] = alpha[i];
  
  return ( temp );
}  /* duplicateAlpha */
/**********************************************************************/
void 
copyAlpha( double *dest, double *src ) 
{
  /*
    Assumes the memory has been allocated and simply copies the values
    from the src to the dest argument.
  */
  int i;
  
  if (( src == NULL) || ( dest == NULL ))
    return;
  
  for( i = 0; i < gNumStates; i++)
    dest[i] = src[i];
  
}  /* copyAlpha */
/**********************************************************************/
void 
destroyAlpha( double *alpha ) 
{
  /*
    Free the memory for an alpha vector. 
  */

  if ( alpha != NULL )
    XFREE( alpha );
}  /* destroyAlpha */
/**********************************************************************/
int 
sameAlpha( double *alpha1,
	   double *alpha2, double epsilon ) 
{
  /* 
     Compares two alpha vectors and determines if they are the identical
     vector.  The tricky part here is that there is floating point
     comparisons that we need to deal with and that can have a
     substantial effect on the algorithm. 
  */
  int i;

  if (( alpha1 == NULL) && (alpha2 == NULL))
    return TRUE;
  
  if (( alpha1 == NULL ) || (alpha2 == NULL))
    return FALSE;
  
  for (i = 0; i < gNumStates; i++)
    if ( ! Equal( alpha1[i], alpha2[i], epsilon ))
      return (FALSE);
  
  return (TRUE);
}  /* sameAlpha */
/**********************************************************************/
int 
isZeroAlpha( double *alpha, double epsilon ) 
{
  /* 
     Just checks to see if all components are zero.  Will return
     TRUE if it is zero of if NULL is sent in and FALSE otherwise. 
  */
  int i;

  if ( alpha == NULL)
    return ( TRUE );

  for (i = 0; i < gNumStates; i++)
    if ( ! Equal( alpha[i], 0.0, epsilon ))
	 return ( FALSE );

  return ( TRUE );

} /*  isZeroAlpha  */
/**********************************************************************/
void 
displayAlpha( FILE *file, double *alpha ) 
{
  /*
    Display the vector to the file stream putting no extra whitespace
    before or after the vector.
  */
   int i;

   if ( alpha == NULL) {
      fprintf( file, "<NULL>");
      return;
   }
   fprintf( file, "[%.*lf", NUM_DECIMAL_DISPLAY, alpha[0] );
   for (i = 1; i < gNumStates; i++) {
      fprintf(file, " ");
      fprintf( file, "%.*lf", NUM_DECIMAL_DISPLAY, alpha[i] );
   }  /* for i */
      fprintf(file, "]");
}  /* displayAlpha */
/**********************************************************************/
void 
showAlpha( double *alpha ) 
{
  /*
    Displays vector to stdout.
  */
  displayAlpha( stdout, alpha );
  fprintf( stdout, "\n" );
}  /* showAlpha */
/**********************************************************************/
int 
isLexicographicallyBetterAlpha( double *first_alpha,
						  double *second_alpha,
						  double epsilon ) 
{
  /* 
     Does a lexicographic check on two vectors, given the two vectors.
     Return TRUE if first_alpha is lexicographically better than
     second_alpha. 
  */
  int i;

  /* This loops iterates until it finds two components that are not
     exactly the same.  */
  for ( i = 0; i < gNumStates; i++ ) {

    if ( Equal( first_alpha[i], 
			 second_alpha[i], epsilon )) 
      continue;
    
    if ( GreaterThan( first_alpha[i], second_alpha[i], epsilon )) 
      return ( TRUE );

    else
      return ( FALSE );

  }  /* for i */

  /* If we get to here then they really are equal, so the first one is
     not better than the second. */
  return ( FALSE );

}  /* isLexicographicallyBetterAlpha */
/**********************************************************************/
int 
isLexicographicallyBetter( AlphaList first_alpha,
					  AlphaList second_alpha,
					  double epsilon ) 
{
  /* Does a lexicographic check on two vectors pointed two by
     the two list nodes.
  */
  return ( isLexicographicallyBetterAlpha( first_alpha->alpha,
                                           second_alpha->alpha,
                                           epsilon ));

}  /* isLexicographicallyBetter */
/**********************************************************************/
int 
isDominatedVector( double *alpha1, double *alpha2 ) 
{
  /* 
     Returns true if alpha2 is component-wise dominated by alpha1. The
     assumption here is that with two identical vectors neither would be
     considered dominating toward the other.  
  */
  int i;
 
  Assert( alpha1 != NULL && alpha2 != NULL,
          "Vector(s) is NULL." );

  /* We really can get away with an epsilon of 0.0 here; I can prove
     it!  */
  
  for (i = 0; i < gNumStates; i++) 
    if ( alpha1[i] <= alpha2[i] )
      return ( FALSE );

  return ( TRUE );
} /* isDominatedVector */
/**********************************************************************/

/**********************************************************************/
/******************  Obs_Source Routines         **********************/
/**********************************************************************/
AlphaList *
newObsSourceArray(  ) 
{
  /*
    Just a convenient function for getting a pointer to an array of
    AlphaList pointers for the obs_source field of the AlphaList
    nodes. Initializes the array to have all NULL vectors.
  */
  AlphaList node;

  return ( (AlphaList *) XCALLOC( gNumObservations,
				  sizeof( *node ) ));
}  /* *newObsSourceArray */
/**********************************************************************/
AlphaList *
duplicateObsSourceArray( AlphaList *orig_obs_source ) 
{
  /*
    Allocates memory for and copies the obs_source array and returns a
    pointer to the new memory.
  */
  AlphaList *new_obs_source;
  int z;

  new_obs_source = newObsSourceArray();

  for( z = 0; z < gNumObservations; z++ )
    new_obs_source[z] = orig_obs_source[z];

  return ( new_obs_source );
}  /* *duplicateObsSourceArray */
/**********************************************************************/

/**********************************************************************/
/******************  Alpha List Node Routines    **********************/
/**********************************************************************/
AlphaList 
newAlphaNode( double *alpha, int action ) 
{
  /*
    Allocates the memory for and sets initial values for an alpha list
    node. 
  */
  AlphaList temp;

  temp = (AlphaList) XMALLOC( sizeof( *temp ));

  temp->alpha = alpha;
  temp->action = action;

  /* These fields should start out with some default values. */
  temp->id = UNINITIALIZED;
  temp->obs = UNINITIALIZED;
  temp->prev_source = NULL;
  temp->next = NULL;
  temp->obs_source = NULL;
  temp->first_source = NULL;
  temp->second_source = NULL;
  temp->witness = NULL;
  temp->mark = FALSE;
  temp->hook = NULL;

  /* Make all the header-specific fields useless values. */
  temp->length = UNINITIALIZED;
  temp->head = NULL;
  temp->tail = NULL;

  return ( temp );
}  /* newAlphaNode */
/**********************************************************************/
AlphaList 
newAlphaNodeObsSource( double *alpha, 
		       int action ) 
{
  AlphaList node;

  node = newAlphaNode( alpha, action );

  node->obs_source = newObsSourceArray();

  return( node );
} /* newAlphaNodeObsSource */
/**********************************************************************/
void 
destroyAlphaNode( AlphaList temp ) 
{
  /*
    Frees the memory for an alpha list node. Also free some supplemental
    memory that might be hanging off this. 
  */

  Assert( temp != NULL, "Cannot destroy NULL node." );

  destroyAlpha( temp->alpha );
  
  /* We assume that memory was allocated for this for the sole purpose
     of this node, so we should free it now. Note that it does not
     free anything that these point to since they are actually part of
     a separate list. */
  if ( temp->obs_source != NULL ) 
    XFREE( temp->obs_source );

  /* We also assume that if a witness point was set here, that we need
     to free this memory also. */
  if ( temp->witness != NULL ) 
    XFREE( temp->witness );
  
  /* Note that we *do not* clear out memory of 'source' and other
     fields because these are just pointers into other data structures
     whose memory might already have been freed or which is still in
     use. */

  XFREE( temp );
}  /* destroyAlphaNode */
/**********************************************************************/
void 
appendNodeToAlphaList( AlphaList list, AlphaList node ) 
{
  /*
    Adds the node to the end of the list.
  */
  Assert( list != NULL && node != NULL, 
          "Bad (NULL) parameter(s)." );
  
  if ( list->length == 0 ) {
    node->id = 0;
    list->head = node;
  }
  else {
    node->id = list->tail->id + 1;
    list->tail->next = node;
  }
  
  list->tail = node;
  (list->length)++;
  
}  /* appendNodeToAlphaList */
/**********************************************************************/
void 
prependNodeToAlphaList( AlphaList list, AlphaList node ) 
{
  /*
    Adds the node to the beginning of the list. 
  */
  Assert( list != NULL && node != NULL, 
          "Bad (NULL) parameter(s)." );

  if ( list->length == 0 ) {
    node->id = 0;
    list->tail = node;
  }
  else
    node->id = list->head->id - 1;
  
  node->next = list->head;
  list->head = node;
  (list->length)++;
  
}  /* prependNodeToAlphaList */
/**********************************************************************/
AlphaList 
dequeueAlphaNode( AlphaList list ) 
{
  /*
    Removes the first item in the list and returns it.
  */
   AlphaList item;
   
   if ( list->length < 1 )
     return ( NULL );
   
   if ( list->length == 1 )
     list->tail = NULL;
   
   item = list->head;
   list->head = list->head->next;
   item->next = NULL;
   (list->length)--;
   
   return ( item );
}  /* dequeueAlphaList */
/**********************************************************************/
void 
enqueueAlphaNode( AlphaList list, AlphaList node ) 
{
  /*
    Puts an alpha list node at the end of the list.
  */
  appendNodeToAlphaList( list, node );
  
}  /* enqueueAlphaNode */
/**********************************************************************/
AlphaList 
duplicateAlphaNode( AlphaList node ) 
{
  /*
    Allocates the memory and copies an AlphaList node. Copies pointers
    if it has any, but not objects they point to. The slight exception
    is the obs_source array.  It makes new space for this duplicate
    node's obs_source and then copies the pointers.
  */
  AlphaList new_node;
  int z;

  Assert( node != NULL, "Cannot duplicate NULL node." );

  new_node = newAlphaNode( duplicateAlpha( node->alpha ),
                              node->action );

  /* We will copy most everything. */
  new_node->prev_source = node->prev_source;

  new_node->id = node->id;
  new_node->obs = node->obs;

  new_node->first_source = node->first_source;
  new_node->second_source = node->second_source;
  new_node->witness = node->witness;

  /* If there is an obs_choice array, we will copy the pointers, but
     must make a new array to hold them. */
  if ( node->obs_source != NULL ) {
    
    new_node->obs_source = newObsSourceArray();
    for ( z = 0; z < gNumObservations; z++ )
      new_node->obs_source[z] = node->obs_source[z];
  }  /* if node->obs_source != NULL */

  else
    new_node->obs_source = NULL;

    
  return ( new_node );
}  /* duplicateAlphaNode */
/**********************************************************************/
AlphaList 
appendDuplicateNodeToAlphaList( AlphaList list, 
				AlphaList orig_node ) 
{
  /*
    Make a copy of the node and appends it to the list.  Returns a
    pointer to the newly created node.
  */
  AlphaList node;

  node = duplicateAlphaNode( orig_node );
  appendNodeToAlphaList( list, node );

  return ( node );
}  /* appendDuplicateNodeToAlphaList */
/**********************************************************************/
void 
addWitnessToAlphaNode( AlphaList node, double *witness ) 
{
  /*
    Adds a witness point to the alpha list.  This has to be more than
    simply setting the pointer, since typically the witness point comes
    from an LP which re-uses the memory for the solution point.  Thus we
    need to allocate the memory for the witness point and then copy it.
  */
  int i;

  Assert( node != NULL, "Cannot add witness to NULL node." );
  Assert( witness != NULL, "Attempted to add NULL witness to node." );

  if (( node == NULL )
      || ( witness == NULL ))
    return;
  
  /* If there is an existing witness point, first free this memory. At
     the moment I cannot think of why this might happen, but it
     certainly doesn't hurt to avoid a potential memory leak. */
  if ( node->witness != NULL )
    XFREE ( node->witness );

  node->witness = (double *) XMALLOC( gNumStates * sizeof( double ));

  for ( i = 0; i < gNumStates; i++ )
    node->witness[i] = witness[i];

}  /* addWitnessToAlphaNode */
/**********************************************************************/

/**********************************************************************/
/******************  Alpha List Routines         **********************/
/**********************************************************************/

/**********************************************************************/
void 
initAlphaList( AlphaList list ) 
{
  /*
    Sets the initial values form the node representing the header of an
    AlphaList. 
  */
  Assert( list != NULL, "List is NULL." );

  list->head = NULL;
  list->tail = NULL;
  list->length = 0;
  
  /* Just set these to useless values since they are not relevant to
     the header of the list. */
  list->alpha = NULL;
  list->action = UNINITIALIZED;
  list->obs = UNINITIALIZED;
  list->id = UNINITIALIZED;
  list->next = NULL;
  list->obs_source = NULL;
  list->prev_source = NULL;
  list->first_source = NULL;
  list->second_source = NULL;
  list->witness = NULL;
  list->mark = FALSE;
  list->hook = NULL;
  
}  /* inittAlphaList */
/**********************************************************************/
AlphaList 
newAlphaList( ) 
{
  /*
    Allocates the memory for the header node of a new alpha list. 
  */
  AlphaList list;
  
  list = (AlphaList) XMALLOC( sizeof( *list ));
  initAlphaList( list );
  
  return ( list );
}  /* newAlphaList */
/**********************************************************************/
void 
renumberAlphaList( AlphaList list ) 
{
  /*
    Renumbers the alpha list so vectors are numbered sequentially.
  */
  int list_num = 0;
  
  Assert( list != NULL, "List is NULL." );
   
  list = list->head;
  while( list != NULL ) {
    list->id = list_num++;
    list = list->next;
  }  /* while */
  
}  /* renumberAlphaList */
/**********************************************************************/
AlphaList 
prependAlphaList( AlphaList list,
		  double *alpha,
		  int action ) 
{
  /*
    Puts an alpha node at the beginning of the list and retruns a
    pointer to the node added.
  */
  AlphaList temp;
  
  Assert( list != NULL, "List is NULL." );
  
  temp = newAlphaNode( alpha, action );
  
  prependNodeToAlphaList( list, temp );

  return ( temp );
}  /* prependAlphaList */
/**********************************************************************/
AlphaList 
appendAlphaList( AlphaList list,
		 double *alpha,
		 int action ) 
{
  /*
    Puts an alpha node at the end of the list and retruns a
    pointer to the node added.
  */
  AlphaList temp;

  Assert( list != NULL, "List is NULL." );

  temp = newAlphaNode( alpha, action );

  appendNodeToAlphaList( list, temp );

  return ( temp );
}  /* appendAlphaList */
/**********************************************************************/
void 
clearAlphaList( AlphaList orig_list ) 
{
  /*
    Frees the memory for each node in the list and resets the header
    node to reflect an empty list.
  */
  AlphaList list, temp;
  
  Assert( orig_list != NULL, "List is NULL." );

  list = orig_list->head;
  while( list != NULL ) {
    temp = list;
    list = list->next;
    
    destroyAlphaNode( temp );
  }  /* while */

  initAlphaList( orig_list );

}  /* clearAlphaList */
/**********************************************************************/
void 
destroyAlphaList( AlphaList list ) 
{
  /*
    Comletely frees up the memory for the entire list, including all
    nodes and the header node.
  */

  Assert( list != NULL, "List is NULL." );
  
  clearAlphaList( list );
  XFREE( list );

}  /* destroyAlphaList */
/**********************************************************************/
double 
bestVectorValuePrimed( AlphaList list, 
		       double *belief_state,
		       AlphaList *best_ptr,
		       double initial_value,
		       double epsilon ) 
{
  /* 
     Takes a list of alpha vectors and a belief state and returns the
     value and vector in the list that gives the maximal value.  If there
     are ties, then we must invoke the tie breaking scheme using the
     lexicographic comparison of the vectors.  The function returns the
     value, and the best_ptr returns the vector, the initial value
     serves as the initial value to use.  If no vectors are better than
     the initial value, then NULL is returned.
     
     We arbitrarily define that a vector that is equal to the initial
     value is automatically *not* better (since lexicographic check
     cannot be done with no vector.)
  */

  double cur_best_value;
  double cur_value;
  int i;
  
  Assert( list != NULL && belief_state != NULL,
          "List or belief state is NULL." );
  
  /* The worst possible value depends upon whether the state-action
     values of the model are costs or rewards. */
  cur_best_value = initial_value;
  *best_ptr = NULL;
  
  list = list->head;
  while( list != NULL ) {
    
    /* Get dot product value */
    cur_value = 0.0;
    for ( i = 0; i < gNumStates; i++)
      cur_value += belief_state[i] * list->alpha[i];
    
    /* We must break ties in the values by using the lexicographic
       maximum criteria. Because we have an initial value for whch no
       vector might be beter than, we have to make sure that best_ptr
       is not NULL and handle it properly. Because we cannot know
       whether a vector is lexicographically better than an arbitrary
       value, we define that it is not. */
    if ( Equal( cur_value, cur_best_value, epsilon )
         && ( *best_ptr != NULL )
         && isLexicographicallyBetter( list, *best_ptr, epsilon )) {
        
      /* This is the currently controversial line.  Should this value
         be reset to the current value or should we leave it? 
         
         cur_best_value = cur_value;
      */
      
      *best_ptr = list;
      
    } /* if values and vector is lexicographically better. */
    
    else if ( GreaterThan( cur_value, cur_best_value, epsilon )) {
      cur_best_value = cur_value;
      *best_ptr = list;
    }
    
    list = list->next;
  } /* while */
  
  return ( cur_best_value );   

}  /* bestVectorValuePrimed */
/**********************************************************************/
double 
bestVectorValue( AlphaList list, 
		 double *belief_state,
		 AlphaList *best_ptr,
		 double epsilon ) 
{
  /*
    Just calls bestVectorValuePrimed with the worst possible value to
    ensure some vector will be the best. 
  */

  return ( bestVectorValuePrimed( list, belief_state, 
                                  best_ptr, worstPossibleValue(),
                                  epsilon ) );
  
}  /* bestVectorValue */
/**********************************************************************/
AlphaList 
bestVectorPrimed( AlphaList list, 
		  double *belief_state, 
		  double *best_value,
		  double initial_value,
		  double epsilon ) 
{
  /*
    Takes a list of alpha vectors and a belief state and returns the
    vector in the list that gives the maximal value.  If there are ties,
    then we must invoke the tie breaking scheme using the lexicographic
    comparison of the vectors.
  */
  
  AlphaList best_ptr;
  
  Assert( list != NULL && belief_state != NULL,
          "List or belief state is NULL." );

  *best_value = bestVectorValuePrimed( list, belief_state, 
                                       &best_ptr, initial_value,
                                       epsilon );

  return ( best_ptr );   

}  /* bestVectorPrimed */
/**********************************************************************/
AlphaList 
bestVector( AlphaList list, 
	    double *belief_state,
	    double *best_value,
	    double epsilon ) 
{
  /*
    Takes a list of alpha vectors and a belief state and returns the
    vector in the list that gives the maximal value.  If there are ties,
    then we must invoke the tie breaking scheme using the lexicographic
    comparison of the vectors.
  */
  
  AlphaList best_ptr;
  
  Assert( list != NULL && belief_state != NULL,
          "List or belief state is NULL." );

  *best_value = bestVectorValue( list, belief_state, 
                                 &best_ptr, epsilon );

  return ( best_ptr );   

}  /* bestVector */
/**********************************************************************/
AlphaList 
findAlphaVector( AlphaList list, 
		 double *alpha,
		 double epsilon ) 
{
  /*
    This routine returns a pointer to the list node that contains
    the vector 'alpha' of interest if it is found.  It returns NULL
    if the vector is not in the list.
  */
  
  Assert( list != NULL, "List is NULL." );

  list = list->head;
  
  while( list != NULL ) {
    
    if( sameAlpha( list->alpha, alpha, epsilon ) == TRUE )
      return( list );;
    
    list = list->next;
  }  /* while */
  
  return( NULL );
}  /* findAlphaVector */
/**********************************************************************/
int 
queryAlphaList( AlphaList list, 
		double *alpha,
		double epsilon ) 
{
  /*
    Returns TRUE if the alpha vector parameter is in the list. 
  */

  return ( findAlphaVector( list, alpha, epsilon ) != NULL );

}  /* queryAlphaList */
/**********************************************************************/
int 
sizeAlphaList( AlphaList list ) 
{
  /*
    Just get the number of alpha vectors in the list by accessing the
    variable in the header.
  */
  Assert( list != NULL, "List is NULL." );

  return ( list->length );
}  /* sizeAlphaList */
/**********************************************************************/
void 
copyAlphaList( AlphaList dest_list,
	       AlphaList src_list ) 
{
  /*
    Doesn't copy the obs_source or witness point fields, leaves them
    blank.
  */
  AlphaList list, temp;
  double *alpha;

  Assert( dest_list != NULL, "Destination list is NULL." );
  Assert( src_list != NULL, "Source list is NULL." );

  /* It is wrong to copy these fields, since they will want to point
     to whole new fragments of memory where the copy resides. */
  dest_list->head = NULL;
  dest_list->tail = NULL;
  dest_list->length = 0;
  
  /* Just copy from the source list (even if they make no sense in
     the header. */
  dest_list->alpha = src_list->alpha;
  dest_list->action = src_list->action;
  dest_list->obs = src_list->obs;
  dest_list->prev_source = src_list->prev_source;
  dest_list->id = src_list->id;
  dest_list->next = src_list->next;
  dest_list->first_source = src_list->first_source;
  dest_list->second_source = src_list->second_source;
  dest_list->mark = src_list->mark;
  dest_list->hook = src_list->hook;
   
  /* Don't copy these for now */
  dest_list->obs_source = NULL;
  dest_list->witness = NULL;
  
  list = src_list->head;
  while( list != NULL ) {

    alpha = duplicateAlpha( list->alpha );
    temp = appendAlphaList( dest_list, alpha, list->action );

    /* Copy all the fields, though the burden on the being sane values
       lies in the orginal list. */
    temp->id = src_list->id;
    temp->obs = src_list->obs;
    temp->prev_source = src_list->prev_source;
    temp->next = src_list->next;
    temp->first_source = src_list->first_source;
    temp->second_source = src_list->second_source;
    temp->mark = src_list->mark;
    temp->hook = src_list->hook;
    temp->length = src_list->length;
    temp->head = src_list->head;
    temp->tail = src_list->tail;
    
    temp->obs_source = NULL;
    temp->witness = NULL;

    list = list->next;
  } /* while */
  
}  /* copyAlphaList */
/**********************************************************************/
AlphaList 
duplicateAlphaList( AlphaList src_list ) 
{
  /* 
     Allocates a new list and copies the src_list into it.
  */
  AlphaList dest_list;

  Assert( src_list != NULL, "Source list is NULL." );

  dest_list = newAlphaList( );
  
  copyAlphaList( dest_list, src_list );

  return ( dest_list );

}  /* duplicateAlphaList */
/**********************************************************************/
AlphaList 
duplicateAlphaListWithWitnesses( AlphaList list ) 
{
  /*
    Duplicates the alpha list just like the duplicateAlphaList() method,
    but also copies the witness fields.
  */
  AlphaList new_list, node, new_node;

  new_list = duplicateAlphaList( list );

  for ( node = list->head, new_node = new_list->head;
        node != NULL;
        node = node->next, new_node = new_node->next )
    
    addWitnessToAlphaNode( new_node, node->witness );
  
  return ( new_list );
}  /* duplicateAlphaListWithWitnesses */
/**********************************************************************/
int 
sameAlphaList( AlphaList l1, AlphaList l2, double epsilon ) 
{
  /* 
     Just checks if the two lists contain the same alpha_vectors in
     exactly the same order.  
  */
  AlphaList list1, list2;
  
  Assert( l1 != NULL && l2 != NULL, "List(s) is NULL." );

  if ( l1->length != l2->length )
    return ( FALSE );
  
  list1 = l1->head;
  list2 = l2->head;
  while( list1 != NULL ) {
    
    if ( sameAlpha( list1->alpha, list2->alpha, epsilon ) == FALSE )
      return ( FALSE );
    
    list1 = list1->next;
    list2 = list2->next;
  } /* while */
  
  return ( TRUE );
}  /* sameAlphaList*/
/**********************************************************************/
int 
similarAlphaList( AlphaList list1, 
		  AlphaList list2,
		  double epsilon )
{
  /*
    Returns true if the two alpha lists contains the same alpha vectors,
    though the order is not important.  
  */
  AlphaList list;

  Assert( list1 != NULL && list2 != NULL, 
          "Bad (NULL) parameter(s)." );

  if ( list1->length != list2->length )
    return ( FALSE );

  /* We do not want to make any assumptions about the lists containing
     unique vectors, so we must go through both lists to ensure there
     is a vector in the other list. */

  list = list1->head;
  while( list != NULL ) {

    if ( ! queryAlphaList( list2, list->alpha, epsilon ))
      return ( FALSE );
    
    list = list->next;
  } /* while */
  
  list = list2->head;
  while( list != NULL ) {

    if ( ! queryAlphaList( list1, list->alpha, epsilon ))
      return ( FALSE );
    
    list = list->next;
  } /* while */
  
  return ( TRUE );
}  /* similarAlphaList */
/**********************************************************************/
void 
roundAlphaList( AlphaList list, double precision ) 
{
  int i;

  Assert( list != NULL, "Alpha list is NULL." );
  Assert( precision != 0.0, "Precision is zero." );
  
  list = list->head;
  while( list != NULL ) {
    
    for ( i = 0; i < gNumStates; i++ )
	 list->alpha[i] = round( list->alpha[i] / precision ) * precision;
    
    list = list->next;
  }  /* while */

} /* writeAlphaList */
/**********************************************************************/
void 
displayAlphaList( FILE *file, AlphaList list ) 
{
  /*
    Printout a textual version of the list.
  */
  Assert( file != NULL, "File handle is NULL." );
  Assert( list != NULL, "List is NULL." );
  
  fprintf(file, "Alpha List: Length=%d\n", list->length );

  list = list->head;
  while(list != NULL ) {
    
    fprintf(file, "<id=%d:", list->id );
    fprintf(file, " a=%d", list->action );
    if ( list->obs >= 0 )
	 fprintf(file, " z=%d", list->obs );

    if ( list->mark )
      fprintf( file, " m" );
    
    if ( list->witness != NULL )
      fprintf( file, " w" );

    if ( list->obs_source != NULL )
      fprintf( file, " s" );
    fprintf ( file, "> " );

    displayAlpha(file, list->alpha );
    fprintf ( file, "\n" );

    list = list->next;
  }  /* while */

}  /* displayAlphaList */
/**********************************************************************/
void 
showAlphaList( AlphaList list ) 
{
  /*
    Printout to stdout, Especially useful in debugger.
  */
  displayAlphaList( stdout, list );
}  /* showAlphaList */
/**********************************************************************/
AlphaList 
readAlphaList( char *filename, int max_alphas ) 
{
  /*
    Reads a list of alpha vectors from a file.  The format of the file
    is very specific and does not allow comments.  It simply reads a
    sequence of numbers which are assumed to be in the correct order.
    This should only be used to read in things written out by the code.
    Also, there is no checking to make sure the file is consistent with
    the problem. i.e., if the probelm has 3 states and you read a file
    of 4 component vectors, this will not notice and might result in
    strange things.  It does a simple check of this by seeing if it is
    in the middle of a vector when the file ends.  However, this does
    not guarantee anything.
    
    It can also read only a subset of the file of vectors.
    Set max_alphas to <= 0 if you want the whole file read, otherwise
    it will only read the first max_alphas in the file.
  */
   FILE *file;
   int a, i;
   double *alpha;
   AlphaList list = NULL;
   
   if ((file = fopen(filename , "r")) == NULL) {
     fprintf( gStdErrFile, 
              "** Error: The alpha vector file: %s does not exist.\n",
             filename);
     return ( NULL );
   }
   
   list = newAlphaList();
   
   /* We want specifying zero to be like specifying a negative number,
      only we can't start out at zero, or else the loop will never be
      entered.  Just decrement max_alphas and everything will be
      hunky-dorry. */
   if ( max_alphas == 0 )
     max_alphas--;
   
   while( max_alphas != 0 ) {
     max_alphas--;
     if ( fscanf( file, "%d", &a ) == EOF )
       break;
     alpha = newAlpha();
     
     for ( i = 0; i < gNumStates; i++ )
       if ( fscanf( file, "%lf", &( alpha[i] )) == EOF ) {
         fprintf(gStdErrFile, 
                 "** Error: Alpha vector file format incorrect.\n");
         return ( NULL );
       }
     
     appendAlphaList( list, alpha, a );
   }  /* while */
   
   fclose( file );
   
   return ( list );
}  /* readAlphaList */
/**********************************************************************/
void 
writeAlphaList( FILE *file, AlphaList list ) 
{
  int i;

  Assert( file != NULL, "File handle is NULL." );
  Assert( list != NULL, "Alpha list is NULL." );
  
  list = list->head;
  while( list != NULL ) {
    fprintf( file, "%d\n", list->action );
    
    for ( i = 0; i < gNumStates; i++ )
	 fprintf( file, "%.*lf ", ALPHA_FILE_DECIMAL_PRECISION,
			list->alpha[i] );
    fprintf( file, "\n\n");
    
    list = list->next;
  }  /* while */

} /* writeAlphaList */
/**********************************************************************/
void 
saveAlphaList( AlphaList list, char *filename ) 
{
  /*
    Write the alpha list out to a file in the format that
    readAlphaList() will read them in.  It is not a very pretty format,
    but makes reading it in trivial.
  */
   FILE *file;
   
   if ((file = fopen(filename , "w")) == NULL) {
     fprintf(gStdErrFile, 
             "** Error: The alpha vector file: %s cannot be opened.\n",
             filename);
     return;
   }
   
   writeAlphaList( file, list );
   
   fclose( file );
}  /* saveAlphaList */
/**********************************************************************/
void 
unionTwoAlphaLists( AlphaList list, AlphaList other_list ) 
{
  /*
    Takes the union of the two lists sent in and returns the union in
    the 'list' argument.  It is a destructive union, since effectively
    all nodes in the other_list are moved to this list.
  */
  if (( list == NULL ) 
      || ( other_list == NULL )
      || ( other_list->length == 0 ))
    return;

  if ( list->length < 1 ) {
    list->head = other_list->head;
    list->length = other_list->length;
    list->tail = other_list->tail;
  }

  else {
    list->tail->next = other_list->head;
    list->tail = other_list->tail;
    list->length += other_list->length;
  }

  /* We only want to free the header node memory, since it still
     contains pointers to the nodes in the list which now exist in the
     first list. i.e., it is a major mistake to call the
     destroyAlphaList() routine on other_list. */
  XFREE( other_list );

} /* unionTwoAlphaLists */
/**********************************************************************/
void 
clearObsSourceAlphaList( AlphaList list ) 
{
  /*
    Clears any memory for the 'choice' list for the nodes in the list.
  */
  if ( list == NULL )
    return;
  
  list = list->head;
  while( list != NULL ) {
    
    if ( list->obs_source != NULL )
      XFREE( list->obs_source );
    
    list->obs_source = NULL;
    list = list->next;
  }

}  /* clearObsSourceAlphaList */
/**********************************************************************/
AlphaList 
appendUniqueAlphaList( AlphaList list,
		       double *alpha,
		       int action, 
		       double epsilon )
{
  /*
    Appends a new node to the alpha list for a vector but only if this
    vector does not already exist in the list.  Returns a pointer to the
    new node created. 
  */
   AlphaList temp;

   if ( queryAlphaList( list, alpha, epsilon ) == TRUE )
      return ( NULL );

   temp = appendAlphaList( list, alpha, action );

   return ( temp );

}  /* appendUniqueAlphaList */
/**********************************************************************/
int 
dominatedAlphaList( double *alpha, AlphaList list ) 
{
  /*
    Returns TRUE if any of the alphas in the list dominate
    (component-wise) the first argument alpha vector.  
  */
  Assert( alpha != NULL && list != NULL,
          "Vector and/or list is NULL." );

   list = list->head;
   while( list != NULL ) {

      if ( isDominatedVector( list->alpha, alpha ))
        return ( TRUE );

      list = list->next;
   }  /* while */

   return ( FALSE );
}  /* dominatedAlphaList */
/**********************************************************************/
void
clearMarkAlphaList( AlphaList list ) 
{
  /*
    Sets all the nodes in the list to have their 'mark' field FALSE.
  */
  if ( list == NULL )
    return;
  
  list = list->head;
  while( list != NULL ) {
    
    list->mark = FALSE;

    list = list->next;
  }  /* while */

}  /* clearMarkAlphaList */
/**********************************************************************/
void 
markAllAlphaList( AlphaList list ) 
{
  /*
    Sets all the nodes in the list to have their 'mark' field TRUE.
  */
  if ( list == NULL )
    return;
  
  list = list->head;
  while( list != NULL ) {
    
    list->mark = TRUE;

    list = list->next;
  }  /* while */

}  /* markAllAlphaList */
/**********************************************************************/
int 
sizeUnmarkedAlphaList( AlphaList list ) 
{
  /*
    Returns the number of nodes whose 'mark' field is FALSE.
  */
  int count = 0;

  Assert( list != NULL, "List is NULL" );
  
  list = list->head;
  while ( list != NULL ) {
    
    if ( list->mark == FALSE )
      count++;
    
    list = list->next;
  } /* while */

  return ( count );
}  /* sizeUnmarkedAlphaList */
/**********************************************************************/
int 
allMarkedAlphaList( AlphaList list ) 
{
  /*
    Retruns true if all the nodes in the list have their marked fields
    set. 
  */
  if ( list == NULL )
    return ( TRUE );
  
  list = list->head;
  while( list != NULL ) {
    
    if ( list->mark == FALSE )
      return ( FALSE );

    list = list->next;
  }  /* while */

  return ( TRUE );

}  /* clearMarkAlphaList */
/**********************************************************************/
AlphaList 
findUnmarkedVector(  AlphaList list ) 
{
  /* 
     Returns a pointer to the first vector in the list where the 'mark'
     field is FALSE. If none exist, or the list is empty, it returns
     NULL.  
  */
  Assert( list != NULL, "List is NULL." );
  
  list = list->head;
  while ( list != NULL ) {
    
    if ( list->mark == FALSE )
      return ( list );
    
    list = list->next;
  }  /* while list != NULL */
  
  return ( NULL );

}  /* findUnmarkedVector */
/**********************************************************************/
AlphaList 
extractUnmarkedVector( AlphaList list ) 
{
  /*
    Finds a vector node with the 'mark' field set to FALSE and extracts
    that node from the list returning a pointer to the node.
  */

  /* This will return NULL if no node is found. */
  return ( extractAlphaNode( list, 
                             findUnmarkedVector( list ) ));

}  /* extractUnmarkedVector */
/**********************************************************************/
int 
markDominatedAlphaList( double *alpha, AlphaList list ) 
{
  /*
    Checks and marks those vectors in 'list' which are dominated by the
    vector 'alpha' sent it.  Does not delete them yet, just sets their
    'mark' field to TRUE.  Assumes that the 'mark' field has already
    been cleared and it will not set any 'mark' fields to FALSE.  Any
    existing TRUE 'mark' fields will remain that way regardless of
    whether they are dominated or not.
  */
  int num_marked = 0;

  Assert( list != NULL, "List is NULL." );

  list = list->head;
  while( list != NULL ) {
    
    /* We might as well only check those nodes whose 'mark' field is
       not yet set, since the only result, even if it is dominated,
       would be to set the 'mark' field anyway. */
    if ( list->mark != TRUE )
      if ( isDominatedVector( alpha, list->alpha )) {
        list->mark = TRUE;
        num_marked++;
      }
    
    list = list->next;
  }  /* while */

  return ( num_marked );
}  /* markDominatedAlphaList */
/**********************************************************************/
AlphaList 
extractMarkedAlphaList( AlphaList list ) 
{
  /*
    Removes all the nodes which have their 'mark' fields set to TRUE
    and puts them in a separate list which is returned.
  */
  AlphaList extracted_list;
  AlphaList walk, temp, trail;

  Assert( list != NULL, "List is NULL." );

  /* This is where we will put all the extracted nodes. */
  extracted_list = newAlphaList();

  trail = walk = list->head;
  while( walk != NULL ) {
    
    /***** Case 1: Item is not marked. */
    if ( walk->mark == FALSE ) {
      trail = walk;
      walk = walk->next;
      continue;
    }  /* if not marked */
    
    temp = walk;

    /***** Case 2: Only one item in list. */
    if ( list->length == 1 ) {
      list->head = list->tail = NULL;
      walk = NULL;
    }  /* if only 1 in list */
    
    /***** Case 3: Item is last element of list. */
    else if ( list->tail == walk ) {
      list->tail = trail;
      trail->next = NULL;
      walk = NULL;
    }  /* if last element in list */
      
    /***** Case 4: Item is first element in list. */
    else if ( list->head == walk ) {
      list->head = walk->next;
      trail = walk = list->head;
    }  /* if first element of list */
      
    /***** Case 5: Item not first or last in list. */
    else {  /* not first in list */
      trail->next = walk->next;
      walk = walk->next;    }  /* else not first in list */

    /* We'll clear its mark'ed flag and add it to the list. */
    temp->mark = FALSE;
    temp->next = NULL;
    appendNodeToAlphaList( extracted_list, temp );

    (list->length)--;
    
  }  /* while */
  
  return ( extracted_list );
}  /* extractMarkedAlphaList */
/**********************************************************************/
int 
removeMarkedAlphaList( AlphaList list ) 
{
  /*
    Removes all the nodes which have their 'mark' fields set to TRUE.
    Returns the number of nodes removed.
  */
  AlphaList removed_list;
  int num_removed = 0;

  Assert( list != NULL, "List is NULL." );

  /* First extract the marked nodes. */
  removed_list = extractMarkedAlphaList( list );
  
  /* See how many there are and save it. */
  num_removed = removed_list->length;

  /* Free up the memory for removed nodes. */
  destroyAlphaList( removed_list );

  return ( num_removed );
}  /* removeMarkedAlphaList */
/**********************************************************************/
int 
removeDominatedAlphaList( double *alpha, AlphaList list ) 
{
  /*
    Removes all vectors in the list that are component-wise dominated 
    by the first argument alpha vector.
  */

  /* CLear the 'mark' flags. */
  clearMarkAlphaList( list );
  
  /* First mark all the dominated vectors. */
  markDominatedAlphaList( alpha, list );
  
  /* Then removed all those that were marked. */
  return ( removeMarkedAlphaList( list ));
  
}  /* removeDominatedAlphaList */
/**********************************************************************/
AlphaList 
extractAlphaNode( AlphaList list, AlphaList extract_ptr ) 
{
  /*
    Take a pointer to one of the elements in the list and removes that
    node from the list.  It returns a pointer to the removed node 
    (memory is not freed) or NULL if something goes wrong.
  */
   AlphaList walk_ptr, trail_ptr;

   Assert( list != NULL, "List is NULL." );

   if (( extract_ptr == NULL )
       || ( list->length == 0 ))
     return ( NULL );

   /* see if it is the only one in list */
   if ( list->length == 1 ) {
      if ( list->head == extract_ptr ) {
         list->head = list->tail = NULL;
         list->length = 0;
         return ( extract_ptr );
      }

      else /* there's only one element, but it isn't the one we want */
         return ( NULL );
   }  /* if only one element in list */

   /* see if first element of list */
   if ( extract_ptr == list->head ) {
      list->head = extract_ptr->next;
      (list->length)--;
      extract_ptr->next = NULL;
      return ( extract_ptr );
   }
            
   /* At this point we know that there are 2 or more elements
      in the list and the one we want is not the first one */

   trail_ptr = list->head;
   walk_ptr = list->head->next;

   while( walk_ptr != NULL ) {
      if ( walk_ptr == extract_ptr ) {
         if ( extract_ptr == list->tail )
            /* See if it is the last one in list */
            list->tail = trail_ptr;

         trail_ptr->next = extract_ptr->next;
         (list->length)--;
         extract_ptr->next = NULL;
         return ( extract_ptr );
      }  /* if found */

      trail_ptr = walk_ptr;
      walk_ptr = walk_ptr->next;
   } /* while */

   return ( NULL );
}  /* extractAlphaNode */
/**********************************************************************/
AlphaList 
removebestVectorNode( AlphaList list, 
		      double *b, 
		      double epsilon ) 
{
  /*
    Finds the vector with the highest dot-product value with 'b' and
    removes that node from the list.  It doesn't deallocate any 
    memory and returns a pointer to the removed node, or NULL if
    something goes wrong.
  */
   AlphaList best_ptr;
   double best_value;

   best_ptr = bestVector( list, b, &best_value, epsilon  );
   return ( extractAlphaNode( list, best_ptr ));

}  /* removebestVectorNode */
/**********************************************************************/
AlphaList 
extractFromAlphaList( AlphaList list, 
		      double *alpha, 
		      double epsilon ) 
{
  /*
    If the vector sent in already exists in the list, then it is removed
    from the list and the pointer to the node is returned. 
  */
  return ( extractAlphaNode( list, 
                             findAlphaVector( list, alpha, 
                                              epsilon )));

}  /* extractFromAlphaList */
/**********************************************************************/
int 
removeFromAlphaList( AlphaList list, 
		     double *alpha,
		     double epsilon ) 
{
  /*
    If the alpha vector sent in is already in the list, then the node in
    the list is removed, and deallocated.  This routine returns TRUE if
    a node was removed and FALSE if the vector is not in the list. 
  */
  AlphaList node;

  node = extractFromAlphaList( list, alpha, epsilon );

  if ( node == NULL )
    return ( FALSE );

  destroyAlphaNode( node );
  
  return ( TRUE );
}  /* removeFromAlphaList */
/**********************************************************************/
/**********************************************************************/
/*************** Sorting Alpha Lists              *********************/
/**********************************************************************/

/**********************************************************************/
void 
swapPointersAlphaList( AlphaList *x, AlphaList *y ) 
{
  AlphaList temp;

  temp = *x;
  *x = *y;
  *y = temp;

}  /* swapPointersAlphaList */
/**********************************************************************/
void 
quicksortAlphaList( AlphaList *array, int left, int right ) 
{
  /* 
     Implementation of Quicksort algorithm for an array of AlphaList
     pointers.  Adapted from Kernighan and Ritchie, p.120 (second
     edition) 
  */
  int i, last;

  if( left >= right )
    return;

  swapPointersAlphaList( &(array[left]), &(array[(left + right)/2]) );
  last = left;

  /* Just sorting the list means we are not to picky about the epsilon
     value we use.  */
  for( i = left+1; i <= right; i++ )
    if( isLexicographicallyBetterAlpha( array[left]->alpha,
                                        array[i]->alpha,
                                        1e-15 ))
      swapPointersAlphaList( &(array[(++last)]), &(array[i]) );

  swapPointersAlphaList( &(array[left]), &(array[last]) );

  quicksortAlphaList( array, left, last-1 );
  quicksortAlphaList( array, last+1, right );
    
}  /* quicksortAlphaList */
/**********************************************************************/
void 
sortAlphaList( AlphaList list ) 
{
  /*
    Sorts the list lexicographically.
  */
  AlphaList *array;
  int i, num_vectors;

  /* We set up the list in an array, sort it, then put it back into a
     list. */

  array = (AlphaList *) XMALLOC( list->length * sizeof( *array ));

  num_vectors = list->length;

  for ( i = 0; i < num_vectors; i++ )
    array[i] = dequeueAlphaNode( list );
  
  Assert( list->length == 0,
          "List length not what it should be." );
  
  quicksortAlphaList( array, 0, num_vectors-1 );
  
  for ( i = 0; i < num_vectors; i++ )
    enqueueAlphaNode( list, array[i] );

  XFREE ( array );

}  /* sortAlphaList */
/**********************************************************************/
AlphaList 
makeScaledAlphaList( AlphaList list, int num_updates ) 
{
  /*
    This routine will return a duplicate of the list sent in, but with
    its rewards converted.
  */
  AlphaList scaled_list, node;
  double scale_value;
  int i;

  scale_value = getValueScaleFactor( num_updates );

  scaled_list = duplicateAlphaList( list );

  for ( node = scaled_list->head;
        node != NULL;
        node = node->next )
    for ( i = 0; i < gNumStates; i++ )
      node->alpha[i] += scale_value;
  
  return ( scaled_list );
  
}  /* makeScaledAlphaList */
/**********************************************************************/

/**********************************************************************/
/*************** Arrays of Alpha List Routines    *********************/
/**********************************************************************/

/**********************************************************************/
int 
maxSizeAlphaLists( AlphaList *list, int num_lists ) 
{
  /*
    Takes an array of alpha lists and returns the maximum size over all
    lists. 
  */
  int max_size = 0;
  int i;

  if (( list == NULL )
      || ( num_lists < 0 ))
    return ( 0 );
  
  for( i = 0; i < num_lists; i++ )
    if ( sizeAlphaList( list[i] ) > max_size )
      max_size = sizeAlphaList( list[i] );

  return ( max_size );

}  /* maxSizeAlphaLists */
/**********************************************************************/
