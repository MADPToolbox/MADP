
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    belief.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    April, 2003
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: belief.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/belief.c,v $
 *    $Revision: 1.3 $
 *    $Date: 2004/10/10 03:44:53 $
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

/*
 *   For solving a POMDP using only a finite set of belief points.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "mdp/mdp.h"

#include "global.h"
#include "belief.h"

/**********************************************************************/
/******************   Belief State Routines      **********************/
/**********************************************************************/

/**********************************************************************/
double *
newBelief( ) 
{
  /* 
     Allocate memory for a belief state, whose size is determined by
     the number of states.  Initialize it to zeroes
  */

   return (  (double *) XCALLOC( gNumStates, sizeof( double ) ));
}  /* newBelief */
/**********************************************************************/
double *
duplicateBelief( double *b ) 
{
  /*
    Makes a copy of the alpha vector also allocating the memory for it. 
  */
  double *temp;
  int i;
  
  if ( b == NULL)
    return NULL;
  temp = newBelief( );
  for ( i = 0; i < gNumStates; i++)
    temp[i] = b[i];
  
  return ( temp );
}  /* duplicateBelief */
/**********************************************************************/
void 
copyBelief( double *dest, double *src ) 
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
  
}  /* copyBelief */
/**********************************************************************/
void 
destroyBelief( double *b ) 
{
  /*
    Free the memory for a belief state. 
  */

  if ( b != NULL )
    XFREE( b );
}  /* destroyAlpha */
/**********************************************************************/
int 
sameBelief( double *b1, double *b2, double epsilon ) 
{
  /* 
     Compares two belief states and determines if they are the identical
     vector.  The tricky part here is that there is floating point
     comparisons that we need to deal with and that can have a
     substantial effect on the algorithm. 
  */
  int i;

  if (( b1 == NULL) && (b2 == NULL))
    return TRUE;
  
  if (( b1 == NULL ) || (b2 == NULL))
    return FALSE;
  
  for (i = 0; i < gNumStates; i++)
    if ( ! Equal( b1[i], b2[i], epsilon ))
      return (FALSE);
  
  return (TRUE);
}  /* sameBelief */
/**********************************************************************/
/* Printout a textual version of the finite grid.  
 */
void 
writeBeliefList( FILE *file, BeliefList belief_list ) 
{
  int i;

  Assert( file != NULL, "File handle is NULL." );
  Assert( belief_list != NULL, "Belief list is NULL." );
  
  while( belief_list != NULL ) {

    Assert( belief_list->b != NULL, "Belief state is NULL." );

    for ( i = 0; i < gNumStates; i++ ) 
	 fprintf( file, "%.*lf ", BELIEF_FILE_DECIMAL_PRECISION,
			belief_list->b[i] );

    fprintf ( file, "\n" );
    belief_list = belief_list->next;
  }
   
}  /* writeBeliefList */
/**********************************************************************/
void 
saveBeliefList( BeliefList list, char *filename ) 
{
  /*
    Write the belief list out to a file in the format that
    readBeliefList() will read them in.  It is not a very pretty format,
    but makes reading it in trivial.
  */
   FILE *file;
   
   if ((file = fopen(filename , "w")) == NULL) {
     fprintf(gStdErrFile, 
             "** Error: The belief list file: %s cannot be opened.\n",
             filename);
     return;
   }
   
   writeBeliefList( file, list );
   
   fclose( file );
}  /* saveBeliefList */
/**********************************************************************/
void 
showBeliefList( BeliefList list ) 
{
  /*
    Printout to stdout, Especially useful in debugger.
  */
  writeBeliefList( stdout, list );
}  /* showBeliefList */
/**********************************************************************/
BeliefList 
readBeliefList( char *filename, int max ) 
{
  /*
    Reads a list of belief states from a file.  The format of the file
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
    Set max to <= 0 if you want the whole file read, otherwise
    it will only read the first max_beliefs in the file.
  */
   FILE *file;
   int i;
   double *b;
   BeliefList list = NULL;
   
   if ((file = fopen(filename , "r")) == NULL) {
     fprintf( gStdErrFile, 
              "** Error: The belief list file: %s does not exist.\n",
		    filename);
     return ( NULL );
   }
   
   /* We want specifying zero to be like specifying a negative number,
      only we can't start out at zero, or else the loop will never be
      entered.  Just decrement max and everything will be
      hunky-dorry. */
   if ( max == 0 )
     max--;
   
   while( max != 0 ) {
     max--;

     b = newBelief();
     
     for ( i = 0; i < gNumStates; i++ )
       if ( fscanf( file, "%lf", &( b[i] )) == EOF ) {

	    /* We should run out of numbers to read when i == 0, since we
		  need a number for each POMDP state in a belief.
	    */
	    if ( i > 0 ) {
		 fprintf(gStdErrFile, 
			    "** Error: Belief list file format incorrect.\n");
		 return ( NULL );
	    }
	    else 
		 goto RBL_DONE;
		 
       }
     
     list = appendBeliefList( list, b );
   }  /* while */
   
 RBL_DONE:

   fclose( file );
   
   return ( list );
}  /* readBeliefList */
/**********************************************************************/

/**********************************************************************/
/******************  Belief List Node Routines    *********************/
/**********************************************************************/

/**********************************************************************/
BeliefList 
newBeliefNode( double *b ) 
{
  /*
    Allocates the memory for and sets initial values for a belief node
    and set the belief state to the one sent in (by making a copy).
  */
  BeliefList temp;

  temp = (BeliefList) XMALLOC( sizeof( *temp ));

  temp->b = duplicateBelief( b );
  temp->mark = 0;
  temp->next = NULL;

  return ( temp );
}  /* newBeliefNode */
/**********************************************************************/
/*
  This will free the memory for the node including the belief state it
  contains. 
*/
void 
destroyBeliefNode( BeliefList temp ) 
{

  Assert( temp != NULL, "Cannot destroy NULL belief node." );

  destroyBelief( temp->b );
  
  XFREE( temp );
}  /* destroyBeliefNode */
/**********************************************************************/
BeliefList 
prependBeliefList( BeliefList list,
		   double *b ) 
{
  /*
    Puts a belief state at the beginning of the list and returns a
    pointer to the node added (new beginning of list).  Make a copy of
    the belief state sent in.
  */
  BeliefList temp;
  
  temp = newBeliefNode( b );
  
  temp->next = list;
 
  return ( temp );

}  /* prependBeliefList */
/**********************************************************************/
BeliefList 
appendBeliefList( BeliefList list,
			   double *b ) 
{
  /*
    Puts a belief state at the end of the list and returns a
    pointer to the node added (new beginning of list).  Make a copy of
    the belief state sent in.
  */
  BeliefList temp;
  BeliefList walk_ptr;
  
  temp = newBeliefNode( b );
  
  if ( list == NULL ) {
    return temp;
  }

  walk_ptr = list;
  while ( walk_ptr->next != NULL )
    walk_ptr = walk_ptr->next;

  walk_ptr->next = temp;
 
  return ( list );

}  /* appendBeliefList */
/**********************************************************************/
void 
destroyBeliefList( BeliefList list )
{
  BeliefList temp;

  while( list != NULL ) {

    temp = list;
    list = list->next;

    destroyBeliefNode( temp );
  }

} /* destroyBeliefList */
/**********************************************************************/
int 
sizeBeliefList( BeliefList list ) 
{
  /*
    Just get the number of belief states in the list.
  */
  int count = 0;

  Assert( list != NULL, "List is NULL." );

  while( list != NULL ) {
    count++;
    list = list->next;
  }

  return ( count );
}  /* sizeBeliefList */
/**********************************************************************/
BeliefList 
findBeliefState( BeliefList list, 
		 double *b,
		 double epsilon ) 
{
  /*
    This routine returns a pointer to the list node that contains
    the belief state of interest if it is found.  It returns NULL
    if the vector is not in the list.
  */
  
  while( list != NULL ) {
    
    if( sameBelief( list->b, b, epsilon ) == TRUE )
      return( list );;
    
    list = list->next;
  }  /* while */
  
  return( NULL );
}  /* findBeliefState */
/**********************************************************************/


