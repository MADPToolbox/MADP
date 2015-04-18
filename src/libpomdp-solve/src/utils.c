
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    utils.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: utils.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/utils.c,v $
 *    $Revision: 1.13 $
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

/*
 * Utility routines that require pomdp-solve code, but do not really
 * have anything directly to do with solving a POMDP. e.g., debugging,
 * solution undertsanding. 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "mdp/mdp.h"

#include "global.h"
#include "params.h"
#include "pomdp.h"
#include "alpha.h"
#include "pg.h"
#include "belief.h"
#include "utils.h"

double gUtilEpsilon = 1.0e-9;

/**********************************************************************/
/* Simple helper routine for dealing with a list of non-negative integers
   implemented as a fixed size array with a -1 sentinel terminator.
   This returns the number of elements in the list.
*/
int
UTIL_sizeIndexList( int *index_list )
{
  int count, i;

  count = 0;
  
  for ( i = 0; i < MAX_BEST_COUNT; i++ )
    {
	 if ( index_list[i] < 0 )
	   return count;

	 count++;
    }
  
  fprintf( stderr, "%s %s",
		 "Non-terminated index list.",
		 "This seems to be some internal coding error.\n" );
  exit( 1 );

}

/**********************************************************************/
/* Simple helper routine for dealing with a list of non-negative integers
   implemented as a fixed size array with a -1 sentinel terminator.
   This appends the 'index' parameter to the end of the list.

   Assumes the input list index_list is already terminated with a -1
   and that the size of this array is MAX_BEST_COUNT.

   If the list is already full, this will print an error message and
   halt the program, since this indicates and internal coding error.
*/
void
UTIL_appendIndexList( int *index_list, int index )
{
  int i;

  /* Find end of list to insert. */
  i = 0;
  while ( index_list[i] >= 0 )
    {
	 if ( i >= (MAX_BEST_COUNT-2))
	   {
		fprintf( stderr, "%s %s",
			    "Ran off end of index list.",
			    "This seems to be some internal coding error.\n" );
		exit( 1 );
	   }

	 i++;
    }

  index_list[i] = index;
  index_list[i+1] = -1;

} /* UTIL_appendIndexList */

/**********************************************************************/
void
UTIL_printIndexList( FILE *file, int *index_list )
{
  int i;

  for ( i = 0; i < MAX_BEST_COUNT; i++ )
    {
	 if ( index_list[i] < 0 )
	   return;
	 
	 fprintf( file, " %d", index_list[i] );
    }
  
} /* UTIL_printIndexList */

/**********************************************************************/
/*
  This routine takes in an alpha list and a belief point list and will
  set the input array data structures to reflect the values obtained
  from the dot-product of each belief point with the alpha list.  

  The all-values, best_values and best_alphas arrays are what this
  routine sets, and it is assumed that these are allocated and freed
  elsewhere.  The all_values is a 2D array of size NxM where N is the
  size of the belief list and M the size of the alpha list.  The best
  values is a 1D array of length N.  The best_alphas is a 2D array of
  size NxP where P is a hard-coded limit on the list size arbitarily
  set. It contains a list of pointers to the alphas in the list that
  reach the maximal value (one list for each belief point).  This list
  is NULL terminated.
*/
void
UTIL_computeAlphaBeliefValues( AlphaList alpha_list,
						 BeliefList belief_list,
						 double **value,
						 double *best_value,
						 AlphaList **best_alphas,
						 double epsilon ) 
{
  BeliefList b_ptr;
  AlphaList a_ptr;
  int a_num, i, b_num, best_count;

  b_num = 0;
  b_ptr = belief_list;
  while( b_ptr != NULL ) {
    
    best_value[b_num] = worstPossibleValue();
    best_alphas[b_num][0] = NULL;  /* set list to empty */
    best_count = 0;
    a_num = 0;
    a_ptr = alpha_list->head;
    while( a_ptr != NULL ) {
	 
	 /* Get dot product value */
	 value[b_num][a_num] = 0.0;
	 for ( i = 0; i < gNumStates; i++)
	   value[b_num][a_num] += b_ptr->b[i] * a_ptr->alpha[i];

     if ( GreaterThan( value[b_num][a_num],
				   best_value[b_num], 
				   epsilon )) 
	  {
	    best_value[b_num] = value[b_num][a_num];
	    best_alphas[b_num][0] = a_ptr;
	    best_alphas[b_num][1] = NULL;
	    best_count = 1;
	  }
	else if ( Equal( value[b_num][a_num], 
				  best_value[b_num], 
				  epsilon )) 
	  {

	    if ( best_count >= (MAX_BEST_COUNT - 1) )
		 {
		   fprintf( stderr, "%s %s",
				  "Too many best vectors.",
				  "Increase the MAX_BEST_COUNT variable in utils.h.\n" );
		   exit( 1 );
		 }
	    
	    best_alphas[b_num][best_count] = a_ptr;
	    best_alphas[b_num][best_count+1] = NULL;
	    best_count++;

	  }

	 a_num++;
	 a_ptr = a_ptr->next;
    } /* while b_ptr != NULL */

    /* Sanity check as everything else wants to assume this list is at
	  least of length 1 */
    if ( best_alphas[b_num][0] == NULL )
	 {
	   fprintf( stderr, "%s %s",
			  "No best vectors in UTIL_computeAlphaBeliefValues().",
			  "This is very odd and needs further exploration.\n" );
	   exit( 1 );
	 }


    b_num++;
    b_ptr = b_ptr->next;
  } /* while b_ptr != NULL */

}  /* UTIL_computeAlphaBeliefValues */

/**********************************************************************/
/* alpha_best should be a list of lists (as a fixed sized arrays). A
   list for each bel;ief point of pointers to AlphaList objects with a
   NULL terminating the list.
   
   'unique' will be a list (as a fixed sized array) of indices
   into the alpha list with a -1 terminating the list. This indicate
   which belief points is this vector the sole dominating one from the
   list. 
 */
void
UTIL_setAlphaMaximalPoints( BeliefList belief_list,
					   AlphaList alpha_list,
					   AlphaList **alpha_best, 
					   int **unique,
					   int **non_unique )
{
  BeliefList b_ptr;
  int i, b_num, id;

  /* Initialize unique lists to empty to start */
  for ( i = 0; i < sizeAlphaList( alpha_list ); i++ )
    {
	 unique[i][0] = -1;
	 non_unique[i][0] = -1;
    } /* for i */

  /* Go through all belief points and accumulate which alpha vectors
	are best (uniquely or non-uniquely). */
  b_num = 0;
  b_ptr = belief_list;
  while ( b_ptr != NULL )
    {

	 /* See whether the point has a unique best vector */
	 if ( alpha_best[b_num][1] == NULL )
	   {
		id = alpha_best[b_num][0]->id;

		UTIL_appendIndexList( unique[id], b_num );

	   } /* If belief has a sole unique maximal vector */

	 /* Otherwise, there is more than one vector that achieves the
	    maximal value at this belief point, so go through them all
	    and add this belief index to the "non-unique" listing.
	 */
	 else
	   {
		for ( i = 0; alpha_best[b_num][i] != NULL; i++ )
		  {
		    
		    id = alpha_best[b_num][i]->id;
		    
		    UTIL_appendIndexList( non_unique[id], b_num );

		  } /* for i */

	   }

	 b_num++;
	 b_ptr = b_ptr->next;
    } /* while b_ptr != NULLL */
  
} /* UTIL_setAlphaMaximalPoints */

/**********************************************************************/
void 
UTIL_reportBeliefMapping( FILE *file,
					 BeliefList belief_list,
					 AlphaList alpha_list,
					 double **alpha_values,
					 double *best_values,
					 AlphaList **alpha_best,
					 int **alpha_uniquely_best,
					 int **alpha_non_uniquely_best )
{
  int b_num, i;
  BeliefList b_ptr;
  AlphaList a_ptr;


  fprintf( file, "==============\n" );
  fprintf( file, "Belief Mapping\n" );
  fprintf( file, "==============\n\n" );

  fprintf( file, "Belief List: %d points\n", 
		 sizeBeliefList( belief_list ));
  fprintf( file, "Alpha List %d vectors\n\n", 
		 sizeAlphaList( alpha_list ));

  
  fprintf( file, "--- Belief Section --\n\n" );

  b_num = 0; 
  b_ptr = belief_list;
  while( b_ptr != NULL ) {

    fprintf( file, "Belief[%d]:\n", b_num );
    
    fprintf( file, "  Best Alpha Indices: " );
    for ( i = 0; i < MAX_BEST_COUNT; i++ ) {
	 
	 if ( alpha_best[b_num][i] == NULL )
	   break;
	 
	 fprintf( file, "%d ", alpha_best[b_num][i]->id ) ;
    }
    if ( alpha_best[b_num][1] != NULL )
	 fprintf( file, " (non-unique)" );
	 
    fprintf( file, "\n" );

    fprintf( file, "Best Alpha Value: %.*lf\n",
		   BELIEF_FILE_DECIMAL_PRECISION, best_values[b_num] );

    /* FIXME: Sort this list of all values and print along with the
	  index. */
    /*
	 fprintf( file, "All Alpha Values: " );
	 UTIL_printSortedValues( alpha_values[b_num] );
	 fprintf( file, "\n\n" );

	 (old stuff to print them in non-sorted order)
	 for ( i = 0; i < sizeAlphaList( alpha_list ); i++ ) 
	 fprintf( file, "%.*lf ", BELIEF_FILE_DECIMAL_PRECISION,
	           alpha_values[b_num][i] );
	 fprintf( file, "\n\n" );

    */

    fprintf( file, "\n" );

    b_num++;
    b_ptr = b_ptr->next;
  } /* while b_ptr != NULL */

  fprintf( file, "--- Alpha Section --\n\n" );

  a_ptr = alpha_list->head;
  while( a_ptr != NULL )
    {
	
	 fprintf( file, "Alpha[%d]:\n", a_ptr->id );

	 fprintf( file, "  Uniquely best:" );
	 if ( alpha_uniquely_best[a_ptr->id][0] < 0 )
	   {
		fprintf( file, "  NONE (non-useful vector?)\n" );
	   }
	 else
	   {
		UTIL_printIndexList( file, alpha_uniquely_best[a_ptr->id] );
		fprintf( file, "\n" );
	   }

	 fprintf( file, "  Non-uniquely best:" );
	 if ( alpha_non_uniquely_best[a_ptr->id][0] < 0 )
	   {
		fprintf( file, "  NONE\n" );
	   }
	 else
	   {
		UTIL_printIndexList( file, alpha_non_uniquely_best[a_ptr->id] );
		fprintf( file, "\n" );
	   }
	 fprintf( file, "\n" );
	 
	 a_ptr = a_ptr->next;

    } /* while a_ptr != NULL */

} /* UTIL_reportBeliefMapping */

/**********************************************************************/
/*
  Shows each belief state and how the two alpha lists compare with
  regards to the belief points.
*/
void
UTIL_reportBeliefValueComparisons( FILE *file,
							BeliefList belief_list,
							double *alpha1_best_values,
							double *alpha2_best_values,
							AlphaList **alpha1_best,
							AlphaList **alpha2_best,
							int **alpha1_uniquely_best,
							int **alpha2_uniquely_best,
							int **alpha1_non_uniquely_best,
							int **alpha2_non_uniquely_best,
							double epsilon )
{
  double diff, abs_diff, max_diff;
  int b_num, i;
  BeliefList b_ptr;
  int *diff_idx_list;
  int *maximal_idx_list;

  max_diff = -1.0 * HUGE_VAL;

  /* For convenience in reporting, we want to keep a list of the
	belief points where the two value function differ.  Because we use a
	-1 sentinel list terminator, we need this list to have one more
	element than the number of belief points.
  */
  diff_idx_list = (int *) XMALLOC( sizeBeliefList( belief_list )+1
							* sizeof( int ));

  /* The index list is assumed to contain non-negative integers, with
	a -1 sentinel value ending the list. 
  */

  diff_idx_list[0] = -1;

  /* Also want to keep track of those belief points where the maximal
	differences occur (for convenience). */
  maximal_idx_list = (int *) XMALLOC( sizeBeliefList( belief_list )+1
							   * sizeof( int ));

  maximal_idx_list[0] = -1;

  b_num = 0;
  b_ptr = belief_list;
  while ( b_ptr != NULL )
    {
	 
	 fprintf( file, "Belief[%d]:\n", b_num );

	 fprintf( file, "  [%d] Best Alpha1 Indices: ", b_num );
	 for ( i = 0; i < MAX_BEST_COUNT; i++ ) {
	   
	   if ( alpha1_best[b_num][i] == NULL )
		break;
	   
	   fprintf( file, "%d ", alpha1_best[b_num][i]->id ) ;
	 }
	 fprintf( file, "\n" );

	 fprintf( file, "  [%d] Best Alpha1 Value: %.*lf\n", b_num,
			BELIEF_FILE_DECIMAL_PRECISION, alpha1_best_values[b_num] );

	 fprintf( file, "  [%d] Best Alpha2 Indices: ", b_num );
	 for ( i = 0; i < MAX_BEST_COUNT; i++ ) {
	   
	   if ( alpha2_best[b_num][i] == NULL )
		break;
	   
	   fprintf( file, "%d ", alpha2_best[b_num][i]->id ) ;
	 }
	 fprintf( file, "\n" );

	 fprintf( file, "  [%d] Best Alpha2 Value: %.*lf\n", b_num,
			BELIEF_FILE_DECIMAL_PRECISION, alpha2_best_values[b_num] );

	 diff = alpha2_best_values[b_num] - alpha1_best_values[b_num];
	 abs_diff = fabs( diff );

	 fprintf( file, "  [%d] Difference: %.*lf", b_num,
			BELIEF_FILE_DECIMAL_PRECISION, diff );

	 if ( ! Equal( diff, 0.0, epsilon ))
	   {
		UTIL_appendIndexList( diff_idx_list, b_num );
		
		if ( GreaterThan( abs_diff, max_diff, epsilon ))
		  {
		    max_diff = abs_diff;

		    /* Every time we find a new max, we need to reset the
			  list of maximal indices. */
		    maximal_idx_list[0] = b_num;
		    maximal_idx_list[1] = -1;
		  }

		/* Keep track of the belief points where the maximal value
		   difference is achieved. */
		else if ( Equal( abs_diff, max_diff, epsilon ) )
		    UTIL_appendIndexList( maximal_idx_list, b_num );


		fprintf( file, " (non-zero)" );

	   }

	 fprintf( file, "\n" );
	 
	 b_num++;
	 b_ptr = b_ptr->next;
	 
    } /* while b_ptr != NULL */
  
  fprintf( file, "\n-- Summary --\n\n" );

  
  fprintf( file, "Number of beliefs with value difference: %d\n",
		 UTIL_sizeIndexList( diff_idx_list ));
  
  fprintf( file, "Belief indices with diffs:" );
  UTIL_printIndexList( file, diff_idx_list );
  fprintf( file, "\n" );

  fprintf( file, "Maximal difference: %*f (%d points)\n", 
		 BELIEF_FILE_DECIMAL_PRECISION,
		 max_diff,
		 UTIL_sizeIndexList( maximal_idx_list ) );
  fprintf( file, "Belief indices with maximal diffs:" );
  UTIL_printIndexList( file, maximal_idx_list );
  fprintf( file, "\n" );


  XFREE( diff_idx_list );
  XFREE( maximal_idx_list );

} /* UTIL_reportBeliefValueComparisons */

/**********************************************************************/
#ifdef UMMAGUMMA
void 
UTIL_doAlphaMappping( AlphaList base_list,
				  AlphaList target_list )
{

  fprintf( file, "\nAlpha Mapping - Belief Point Based: alpha1 -> alpha2\n" );

  base_ptr = base_list->head;
  while( base_ptr != NULL )
    { 
	 id = base_ptr->id;

	 fprintf( file, "Alpha1[%d]: ", id );
	 
	 /* First we check to see if there is a belief point for which
	    this base alpha vector is uniquely optimal.  If there is
	    none, then it seems to suggest that this is a non-useful
	    vector, since some combination of the other vectors can yield
	    just as good a value.  
	 */
	 if ( base_uniquely_best[id][0] < 0 )
	   {
		fprintf( file, "Is not uniquely optimal for any belief.\n" );
		continue;
	   }
	 
	 /* b_num is the (first) belief point index for which this vector is
	    uniquely optimal. */
	 b_num = base_uniquely_best[id][0];

	 /* Now we find those optimal zzz .... */
	 base_ptr = base_ptr->next;

    } /* while base_ptr */


} /* UTIL_doAlphaMappping */
#endif

/**********************************************************************/
/*
  This routine will zzzz
 */
void 
UTIL_compareAlphaFilesUsingBeliefs( char *alpha1_filename,
							 char *alpha2_filename,
							 char *belief_filename,
							 double epsilon,
							 char *out_filename  )
{
  int i;

  FILE *file;
  AlphaList alpha1_list;
  AlphaList alpha2_list;
  BeliefList belief_list;

  double **alpha1_values;
  double **alpha2_values;
  double *alpha1_best_values;
  double *alpha2_best_values;
  AlphaList **alpha1_best;
  AlphaList **alpha2_best;

  int **alpha1_uniquely_best;
  int **alpha2_uniquely_best;
  int **alpha1_non_uniquely_best;
  int **alpha2_non_uniquely_best;

  /*********************
   Reading files
  */

  alpha1_list = readAlphaList( alpha1_filename, 0 );
  if ( alpha1_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha1 file: %s cannot be read.\n",
		  alpha1_filename);
    return;
  }

  alpha2_list = readAlphaList( alpha2_filename, 0 );
  if ( alpha2_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha2 file: %s cannot be read.\n",
		  alpha2_filename);
    return;
  }

  belief_list = readBeliefList( belief_filename, 0 );
  if ( belief_list == NULL ) {
    fclose( file );
    return;
  }

  if ((file = fopen( out_filename , "w")) == NULL) {
    fprintf(gStdErrFile, 
		  "** Error: The output file: %s cannot be opened.\n",
		  out_filename);
    return;
  }


  /*********************
   Allocating memory
  */

  alpha1_values =  (double **) XMALLOC( sizeBeliefList( belief_list )
							  * sizeof( *alpha1_values ) );
  alpha2_values =  (double **) XMALLOC( sizeBeliefList( belief_list )
							  * sizeof( *alpha2_values ) );
  
  for ( i = 0; i < sizeBeliefList( belief_list ); i++) {
    alpha1_values[i] = (double *) XMALLOC( sizeAlphaList( alpha1_list )
								   * sizeof( double ) );
    alpha2_values[i] = (double *) XMALLOC( sizeAlphaList( alpha2_list )
								   * sizeof( double ) );
  }

  alpha1_best_values =  (double *) XMALLOC( sizeBeliefList( belief_list )
								    * sizeof( double ) );
  alpha2_best_values =  (double *) XMALLOC( sizeBeliefList( belief_list )
								    * sizeof( double ) );
  

  alpha1_best = (AlphaList **) XMALLOC( sizeBeliefList( belief_list )
								* sizeof( *alpha1_best ));
  alpha2_best = (AlphaList **) XMALLOC( sizeBeliefList( belief_list )
								* sizeof( *alpha2_best ));

  for ( i = 0; i < sizeBeliefList( belief_list ); i++) {
    alpha1_best[i] = (AlphaList *) XMALLOC( MAX_BEST_COUNT
								    * sizeof( AlphaList ) );
    alpha2_best[i] = (AlphaList *) XMALLOC( MAX_BEST_COUNT
								    * sizeof( AlphaList ) );
  }

  alpha1_uniquely_best = (int **) XMALLOC( sizeAlphaList( alpha1_list )
								   * sizeof( *alpha1_uniquely_best));
  alpha1_non_uniquely_best = (int **) XMALLOC( sizeAlphaList( alpha1_list )
							   * sizeof( *alpha1_non_uniquely_best));

  for ( i = 0; i < sizeAlphaList( alpha1_list ); i++) {
    alpha1_uniquely_best[i] = (int *) XMALLOC( MAX_BEST_COUNT
									  * sizeof( int ) );
    alpha1_non_uniquely_best[i] = (int *) XMALLOC( MAX_BEST_COUNT
										 * sizeof( int ) );
  }

  alpha2_uniquely_best = (int **) XMALLOC( sizeAlphaList( alpha2_list )
								   * sizeof( *alpha2_uniquely_best));
  alpha2_non_uniquely_best = (int **) XMALLOC( sizeAlphaList( alpha2_list )
							   * sizeof( *alpha2_non_uniquely_best));

  for ( i = 0; i < sizeAlphaList( alpha2_list ); i++) {
    alpha2_uniquely_best[i] = (int *) XMALLOC( MAX_BEST_COUNT
									  * sizeof( int ) );
    alpha2_non_uniquely_best[i] = (int *) XMALLOC( MAX_BEST_COUNT
										 * sizeof( int ) );
  }

  /*********************
   Doing the work
  */

  UTIL_computeAlphaBeliefValues( alpha1_list, belief_list,
						   alpha1_values, alpha1_best_values,
						   alpha1_best, epsilon );

  UTIL_computeAlphaBeliefValues( alpha2_list, belief_list,
						   alpha2_values, alpha2_best_values,
						   alpha2_best, epsilon );

  UTIL_setAlphaMaximalPoints( belief_list, 
						alpha1_list,
						alpha1_best,
						alpha1_uniquely_best,
						alpha1_non_uniquely_best );

  UTIL_setAlphaMaximalPoints( belief_list,
						alpha2_list,
						alpha2_best, 
						alpha2_uniquely_best,
						alpha2_non_uniquely_best );

  UTIL_reportBeliefMapping( file, 
					   belief_list, 
					   alpha1_list,
					   alpha1_values,
					   alpha1_best_values,
					   alpha1_best,
					   alpha1_uniquely_best,
					   alpha1_non_uniquely_best );

  UTIL_reportBeliefMapping( file,
					   belief_list, 
					   alpha2_list,
					   alpha2_values,
					   alpha2_best_values,
					   alpha2_best,
					   alpha2_uniquely_best,
					   alpha2_non_uniquely_best );

  UTIL_reportBeliefValueComparisons( file,
							  belief_list,
							  alpha1_best_values,
							  alpha2_best_values,
							  alpha1_best,
							  alpha2_best,
							  alpha1_uniquely_best,
							  alpha2_uniquely_best,
							  alpha1_non_uniquely_best,
							  alpha2_non_uniquely_best,
							  epsilon );

#ifdef UMMAGUMMA
  UTIL_doAlphaMapping( alpha1_, alpha2_, fwd_mapping );
  UTIL_doAlphaMapping( alpha2_, alpha1_, rev_mapping );
#endif

  /*********************
   Deallocating the memory
  */

  for ( i = 0; i < sizeBeliefList( belief_list ); i++)
    {
	 XFREE( alpha1_values[i] );
	 XFREE( alpha2_values[i] );
	 XFREE( alpha1_best[i] );
	 XFREE( alpha2_best[i] );
    }

  XFREE( alpha1_values );
  XFREE( alpha2_values );
  XFREE( alpha1_best_values );
  XFREE( alpha2_best_values );
  XFREE( alpha1_best );
  XFREE( alpha2_best );

  for ( i = 0; i < sizeAlphaList( alpha1_list ); i++)
    {
	 XFREE( alpha1_uniquely_best[i] );
	 XFREE( alpha1_non_uniquely_best[i] );
    }

  XFREE( alpha1_uniquely_best );
  XFREE( alpha1_non_uniquely_best );

  for ( i = 0; i < sizeAlphaList( alpha2_list ); i++)
    {
	 XFREE( alpha2_uniquely_best[i] );
	 XFREE( alpha2_non_uniquely_best[i] );
    }

  XFREE( alpha2_uniquely_best );
  XFREE( alpha2_non_uniquely_best );

} /* UTIL_fooNeedsAName */


/**********************************************************************/
/*
  This routine will read in a belief list, and alpha list and then
  output another file containing the belief state, the best alpha
  vector for this state, the alpha vector number (order from alpha
  file, and the value that is attained.  It will also output the value
  for all of the other vectors.
*/
void
UTIL_mapBeliefList( char *belief_filename,
				char *alpha_filename,
				double epsilon,
				char *map_filename )
{
  BeliefList belief_list, b_ptr;
  AlphaList alpha_list, a_ptr;
  AlphaList **alpha_best;
  FILE *file;
  int a_num, b_num, i, id;
  double** alpha_values;
  double* alpha_best_values;

  int **alpha_uniquely_best;
  int **alpha_non_uniquely_best;

  if ((file = fopen( map_filename , "w")) == NULL) {
    fprintf(gStdErrFile, 
             "** Error: The mapping file: %s cannot be opened.\n",
		  map_filename);
    return;
  }
  
  belief_list = readBeliefList( belief_filename, 0 );
  if ( belief_list == NULL ) {
    fclose( file );
    return;
  }

  alpha_list = readAlphaList( alpha_filename, 0 );
  if ( alpha_list == NULL ) {
    destroyBeliefList( belief_list );
    fclose( file );
    return;
  }
  
  /* Print the file preamble */
  fprintf( file, "Belief Mapping File\n\n" );
  fprintf( file, "Belief File: %s (%d points)\n", 
		 belief_filename, sizeBeliefList( belief_list ));
  fprintf( file, "Alpha File: %s (%d vectors)\n\n", 
		 alpha_filename, sizeAlphaList( alpha_list ));
  
  alpha_values =  (double **) XMALLOC( sizeBeliefList( belief_list )
							  * sizeof( *alpha_values ) );
  
  for ( i = 0; i < sizeBeliefList( belief_list ); i++) {
    alpha_values[i] = (double *) XMALLOC( sizeAlphaList( alpha_list )
								* sizeof( double ) );
  }

  alpha_best = (AlphaList **) XMALLOC( sizeBeliefList( belief_list )
								* sizeof( *alpha_best ));

  for ( i = 0; i < sizeBeliefList( belief_list ); i++) {
    alpha_best[i] = (AlphaList *) XMALLOC( MAX_BEST_COUNT
								    * sizeof( AlphaList ) );
  }

  alpha_best_values =  (double *) XMALLOC( sizeBeliefList( belief_list )
							  * sizeof( double ) );

  alpha_uniquely_best = (int **) XMALLOC( sizeAlphaList( alpha_list )
								  * sizeof( *alpha_uniquely_best));
  alpha_non_uniquely_best = (int **) XMALLOC( sizeAlphaList( alpha_list )
							   * sizeof( *alpha_non_uniquely_best));

  /* We are also going to track which of the alpha vectors are
	uniquely and non_uniquely the best for a given belief state.
  */
  for ( i = 0; i < sizeAlphaList( alpha_list ); i++) {
    alpha_uniquely_best[i] = (int *) XMALLOC( MAX_BEST_COUNT
									 * sizeof( int ) );
    alpha_non_uniquely_best[i] = (int *) XMALLOC( MAX_BEST_COUNT
										* sizeof( int ) );
  }

  /* Gather belief points per alpha vector stats. */
  UTIL_computeAlphaBeliefValues( alpha_list, belief_list,
						   alpha_values, alpha_best_values, 
						   alpha_best, epsilon );

  UTIL_setAlphaMaximalPoints( belief_list,
						alpha_list,
						alpha_best, 
						alpha_uniquely_best,
						alpha_non_uniquely_best );

  UTIL_reportBeliefMapping( file,
					   belief_list, 
					   alpha_list,
					   alpha_values,
					   alpha_best_values,
					   alpha_best,
					   alpha_uniquely_best,
					   alpha_non_uniquely_best );


  fprintf( file, "\nAll Belief Points:\n\n" );
  writeBeliefList( file, belief_list );

  fprintf( file, "\nAll Alpha Vectors:\n\n" );
  displayAlphaList( file, alpha_list );

  for ( i = 0; i < sizeBeliefList( belief_list ); i++)
    {
	 XFREE( alpha_values[i] );
	 XFREE( alpha_best[i] );
    }

  XFREE( alpha_values );
  XFREE( alpha_best_values );
  XFREE( alpha_best );

  for ( i = 0; i < sizeAlphaList( alpha_list ); i++)
    {
	 XFREE( alpha_uniquely_best[i] );
	 XFREE( alpha_non_uniquely_best[i] );
    }

  XFREE( alpha_uniquely_best );
  XFREE( alpha_non_uniquely_best );

  destroyBeliefList( belief_list );
  destroyAlphaList( alpha_list );

}  /* UTIL_mapBeliefList */

/**********************************************************************/
/*
  This routine will read in an alpha list, sort it and then write it
  back out.  
*/
void
UTIL_sortAlphaFile( char *in_alpha_filename,
			char *out_alpha_filename ) 
{
  AlphaList in_alpha_list, out_alpha_list;

  in_alpha_list = readAlphaList( in_alpha_filename, 0 );
  if ( in_alpha_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha file: %s cannot be read.\n",
		  in_alpha_filename);
    return;
  }

  sortAlphaList( in_alpha_list );

  saveAlphaList( in_alpha_list, out_alpha_filename );

  destroyAlphaList( in_alpha_list );

}  /* UTIL_sortAlphaFile */

/**********************************************************************/
/*
  This routine will read in an alpha list, sort it and then write it
  back out.  
*/
void
UTIL_purgeAlphaFile( char *in_alpha_filename,
				 char *out_alpha_filename,
				 PomdpSolveParams param ) 
{
  AlphaList in_alpha_list;

  in_alpha_list = readAlphaList( in_alpha_filename, 0 );
  if ( in_alpha_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha file: %s cannot be read.\n",
		  in_alpha_filename);
    return;
  }

  fprintf( param->report_file, 
		 "Input Alpha Count: %d\n",
            sizeAlphaList(in_alpha_list)  );

  purgeAlphaList( in_alpha_list, 
                  param->q_purge_option, 
                  param );

  sortAlphaList( in_alpha_list );

  fprintf( param->report_file, 
		 "Ouput Alpha Count: %d\n",
            sizeAlphaList(in_alpha_list)  );

  saveAlphaList( in_alpha_list, out_alpha_filename );

  destroyAlphaList( in_alpha_list );

}  /* UTIL_purgeAlphaFile */

/**********************************************************************/
/***********   Match Alpha Routines     *******************************/
/**********************************************************************/

/**********************************************************************/
double
UTIL_measureAlphaDiff( double *src_alpha, 
				   double *dest_alpha )
{
  /*
    Compares two alpha vectors and returns a measure of their
    difference. Difference measure is always a positive number, where
    the smaller it is the better.

    Not clear at this time what the best
    comparisons is, but generally, we want to associate the two vectors
    from the two lists that have "the closest slope and values".  I
    believe that wanting both the closest slope and the closest values
    leads to some difficult to define metric.

    Some things to think about:

      Using the maximal component-wise distance (and taking the vector
      that produces the smallest max) [ This is the one that is
      currently implemented below. ]

      Using the total component-wise difference (which is equivalent to
      the average copmponent-wise difference since all vectors have the
      same number of components.)

    I think the best thing is to compute the volume of space between two
    vectors over the area of belief space where one or the other
    dominates.  However, this will require a fair amount of linear
    algebra as we will need to find out the intersection points, and
    then set up a multi-variate integral over some bounded region.
    Because everything is linear, the solution form may have some nice
    properties, but at trhe moment, I have not delved deep enough into
    the math to work this out.
   */
  int s;
  double cur_diff;
  double max_diff = -1.0*HUGE_VAL; /* max component-wise difference */

  for ( s = 0; s < gNumStates; s++ ) {

    cur_diff = fabs( src_alpha[s] - dest_alpha[s] );
    
    if ( cur_diff > max_diff )
	 max_diff = cur_diff;

  }  /* for s */
  
  return max_diff;
  
} /* UTIL_measureAlphaDiff */

/**********************************************************************/
int
UTIL_matchAlpha( double *src_alpha, 
		  AlphaList dest_alpha_list )
{
  /*
    Will take a source (src) alpha vector and will find the best
    matching vector in the destination (dest) list.  It will return
    the index of the vector in dest list that best matches.
    It will also fill in statistically information about the match in
    the MatchAlphaStats structure.
  */
  AlphaList walk_ptr;
  int cur_idx, best_idx;
  double cur_diff;
  double best_diff = HUGE_VAL;

  cur_idx = 0;
  walk_ptr = dest_alpha_list->head;
  while( walk_ptr != NULL ) {

    cur_diff = UTIL_measureAlphaDiff( src_alpha, walk_ptr->alpha );
    
    if ( cur_diff < best_diff )
	 {
	   best_idx = cur_idx;
	   best_diff = cur_diff;
	 }

    walk_ptr = walk_ptr->next;
    cur_idx++;
  }  /* while walk_ptr */

  return best_idx;

} /* matchAlpha */

/**********************************************************************/
int* 
UTIL_matchAlphaLists( AlphaList src_alpha_list, 
				  AlphaList dest_alpha_list )
{
  /*
    Will take a source (src) alpha list and create a mapping (of
    indices) from this list to the best matching vector in the
    destination (dest) list.  It will return a vector whose index is
    that of the index of the src alpha list, and whose value is the
    index of best matching alpha in the dest list.  It will also fill
    in statistically information about the match in the
    MatchAlphaListStats structure.
   */
  AlphaList walk_ptr;
  int from_idx, to_idx;
  int *idx_map;

  /* We will map one vector at a time in alpha1, so these arrays to
	track differences only need to be as large as the alpha2 list.
  */
  idx_map = (int *) XMALLOC( sizeof( int ) * sizeAlphaList(src_alpha_list) );

  from_idx = 0;
  walk_ptr = src_alpha_list->head;
  while( walk_ptr != NULL ) {

    to_idx = UTIL_matchAlpha( walk_ptr->alpha, dest_alpha_list );
    
    idx_map[from_idx] = to_idx;

    walk_ptr = walk_ptr->next;
    from_idx++;
  }  /* while walk_ptr */

  return idx_map;

} /* UTIL_matchAlphaLists */


/**********************************************************************/
void
UTIL_relinkPolicyGraph( char *alpha_filename,
				    char *pg_filename,
				    char *prev_alpha_filename,
				    char *out_pg_filename ) 
{
  /*
    This routine will create a policy graph that is a relinking of the
    input policy graph.  This is used when you want to use a policy
    graph from a finite horizon solution, where you do not have the
    same set of vectors from one iteration to the next.

    Here, you need to have the alpha and pg file for the horizon 'h'
    solution, as well as need to provide the alpha file from the 'h-1'
    iteration. The reason is that the horizon 'h' policy graph has
    edges that points into the 'h-1' policy graph.  These pointers are
    indices of the policy graph nodes in 'h-1'. However, the policy
    graph nodes indices are identical to the indices corresponding to
    the order of the 'h-1' alpha list.

    therefore, we find the closest match from the 'h-1' vectors to the
    'h' vectors, and then reset the 'h' pg pointers to point to the
    best match in the 'h' alpha list rathjer than those in the 'h-1'
    alpha list.
   */
  AlphaList alpha_list, prev_alpha_list;
  int *alpha_map;
  PG pg;

  alpha_list = readAlphaList( alpha_filename, 0 );
  if ( alpha_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha1 file: %s cannot be read.\n",
		  alpha_filename);
    return;
  }

  prev_alpha_list = readAlphaList( prev_alpha_filename, 0 );
  if ( prev_alpha_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha2 file: %s cannot be read.\n",
		  prev_alpha_filename);
    return;
  }

  /* Don't run sanity checks after reading, since here we know we may
	not have a valid PG (the whole purpose of this relinking is to turn
	it into something that is valid).
  */
  pg = PG_read( pg_filename, FALSE );
  if ( pg == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The pg1 file: %s cannot be read.\n",
		  pg_filename);
    return;
  }

  if ( sizeAlphaList( alpha_list ) != pg->num_nodes ) {
    fprintf(gStdErrFile, 
		  "** Error: The pg1 and alpha1 files are not compatible.\n",
		  pg_filename);
    return;
  }

  if ( gNumObservations != pg->num_obs ) {
    fprintf(gStdErrFile, 
		  "** Error: The pg1 file is not compatible with POMDP file.\n",
		  pg_filename);
    return;
  }

  /* Note the reversal of the list order in this call.  Though the
	current pg points into the previous pg, to relink, we want the
	mapping from the previous to the best match in the current, since
	we will be replacing the previous alpha with current ones.
  */
  alpha_map = UTIL_matchAlphaLists( prev_alpha_list, alpha_list );

  PG_relink( pg, alpha_map, sizeAlphaList( prev_alpha_list )-1 );

  PG_write( pg, out_pg_filename );

  /* clean up time */
  destroyAlphaList( alpha_list );
  PG_Destructor( pg );
  destroyAlphaList( prev_alpha_list );
  XFREE( alpha_map );


} /* UTIL_relinkPolicyGraph */

/**********************************************************************/
/***********   Compare Alpha Routines     *******************************/
/**********************************************************************/

/**********************************************************************/
/*
  Here we attempt to match vectors from one alpha list to vectors in
  another alpha list.  Not clear at this time what the best
  comparisons is, but generally, we want to associate the two vectors
  from the two lists that have "the closest slope and values".  I
  believe that wanting both the closest slope and the closest values
  leads to some difficult to define metric.

  Some things to think about:

    Using the maximal component-wise distance (and taking the vector
    that produces the smallest max) [ This is the one that is
    currently implemented below. ]

    Using the total component-wise difference (which is equivalent to
    the average copmponent-wise difference since all vectors have the
    same number of components.)

  I think the best thing is to compute the volume of space between two
  vectors over the area of belief space where one or the other
  dominates.  However, this will require a fair amount of linear
  algebra as we will need to find out the intersection points, and
  then set up a multi-variate integral over some bounded region.
  Because everything is linear, the solution form may have some nice
  properties, but at trhe moment, I have not delved deep enough into
  the math to work this out.

*/
void
UTIL_compareAlphaFiles( char *alpha1_filename,
				    char *alpha2_filename,
				    double epsilon,
				    char *out_filename ) 
{
  FILE *file;
  AlphaList alpha1_list, alpha2_list;
  AlphaList walk_ptr1, walk_ptr2;

  /* Stores the differences between vectors.  We will keep this as a
	sorted list and doa  simple insertion sort as we put things into
	it. */ 
  double *diff;  

  /* Also interested in the actions associated with the vector
	comparison. */  
  int *action;  

  /* As we do the insertion sort of the 'diff' arrray, we also want to
	keep track of the indices of the vectors producing the
	differences.  We'll want to output the mapping from alpha vetcor
	to alpha vector along with the differences.
  */
  int *idx;

  int s, from_idx, to_idx, insert_idx, move_idx;
  double max_diff;

  alpha1_list = readAlphaList( alpha1_filename, 0 );
  if ( alpha1_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha1 file: %s cannot be read.\n",
		  alpha1_filename);
    return;
  }

  alpha2_list = readAlphaList( alpha2_filename, 0 );
  if ( alpha2_list == NULL ) {
    fprintf(gStdErrFile, 
		  "** Error: The alpha2 file: %s cannot be read.\n",
		  alpha2_filename);
    return;
  }

  if ((file = fopen( out_filename , "w")) == NULL) {
    fprintf(gStdErrFile, 
		  "** Error: The output file: %s cannot be opened.\n",
		  out_filename);
    return;
  }

  /* We will map one vector at a time in alpha1, so these arrays to
	track differences only need to be as large as the alpha2 list.
  */
  diff = (double *) XMALLOC( sizeof( double ) * sizeAlphaList(alpha2_list) );
  idx = (int *) XMALLOC( sizeof( int ) * sizeAlphaList(alpha2_list) );
  action = (int *) XMALLOC( sizeof( int ) * sizeAlphaList(alpha2_list) );
  
  fprintf( file, "Alpha List Comparison File\n\n" );
  fprintf( file, "Alpha1 File: %s\n", alpha1_filename );
  fprintf( file, "Alpha2 File: %s\n", alpha2_filename );
  fprintf( file, "Epsilon: %e\n\n", epsilon );

  from_idx = 0;
  walk_ptr1 = alpha1_list->head;
  while( walk_ptr1 != NULL ) {

    to_idx = 0;
    walk_ptr2 = alpha2_list->head;
    while( walk_ptr2 != NULL ) {


	 max_diff = UTIL_measureAlphaDiff( walk_ptr1->alpha,
								walk_ptr2->alpha );
            
	 /* Now insert this into the ordered list. (note that 'to_idx'
	    also happens to be the number of values already added to the
	    list. */

	 /* Find insertion_point */
	 for ( insert_idx = 0; insert_idx < to_idx; insert_idx++ )
	   if ( diff[insert_idx] > max_diff )
		break;

	 /* Move data down in list to accomodate new entry. (note we need
	    to move down both the diff and idx arrays). */
	 for ( move_idx = (to_idx-1); move_idx >= insert_idx; move_idx-- ) {
	   diff[move_idx+1] = diff[move_idx];
	   idx[move_idx+1] = idx[move_idx];
	   action[move_idx+1] = action[move_idx];
	 } /* for move_idx */

	 /* Now we can insert */
	 diff[insert_idx] = max_diff;
	 idx[insert_idx] = to_idx;
	 action[insert_idx] = walk_ptr2->action;

	 walk_ptr2 = walk_ptr2->next;
	 to_idx++;
    }  /* while walk_ptr2 */

    /* Now we output all those whose max_diff values is less than
	  epsilon. Note that in case there is *no* matching vector less
	  than epsilon, we output the lowest value, but flag it so it is
	  explict. */

    fprintf( file, "Index %d [action=%d]", from_idx, walk_ptr1->action );

    if ( diff[0] > epsilon ) {
	 fprintf( file, ": NONE, best is " );
    }

    fprintf( file, ": %d [diff=%e, action=%d] ", idx[0], diff[0], action[0] );
    
    if ( action[0] != walk_ptr1->action )
	 fprintf( file, " (ACTION DIFFERS) " );
    
    for ( s = 1; s < sizeAlphaList(alpha2_list); s++ ) {

	 if ( diff[s] > epsilon ) 
	   break;

	 fprintf( file, ": %d [diff=%e, action=%d] ", idx[s], diff[s], action[s] );
	 
    } /* for s */

    fprintf( file, "\n" );

    walk_ptr1 = walk_ptr1->next;
    from_idx++;
  }  /* while walk_ptr1 */
  
  XFREE( diff );
  XFREE( idx );
  XFREE( action );

  destroyAlphaList( alpha1_list );
  destroyAlphaList( alpha2_list );

}  /* UTIL_compareAlphaFiles */

/**********************************************************************/
/***********   Belief Update Routines   *******************************/
/**********************************************************************/

/**********************************************************************/
int 
UTIL_getLine( FILE *fh, char *string, int max)
{
   register int i;
   register int c;

   max--; 
   i = 0;

   while (( (c = getc(fh)) != EOF )
		&& ( c != '\n'))
	{
	  if ( i < max ) 
	    string[i++] = c;
	}

   string[i] = '\0';
   
   return (( c == EOF ) && (i == 0 ) ? EOF : i);
}

/**********************************************************************/
void 
UTIL_displayBelief( double *b )
{
  int i;

  fprintf( stdout, "[" );
  for ( i = 0; i < gNumStates; i++ ) 
    if ( i == 0 )
	 fprintf( stdout, " %.6lf", b[i] );
    else
	 fprintf( stdout, ", %.6lf", b[i] );
  fprintf( stdout, " ]\n" );

} /* UTIL_displayBelief */

/**********************************************************************/
void 
UTIL_doBeliefUpdates()
{
  /*
   * This function serves as a belief state updating module.  After
   * reading in a POMDP file specification it will continually
   * compute the next belief state given the current action and
   * observation.
   *
   * This program wil receive its input on stdin, and will consist of
   * action and observation.  The current belief state is initialized
   * from the POMDP file specification, or defaults to uniform if not
   * present. This module will track the belief state internally, thus
   * only requiring the action and observation to be input.
   *
   * The input action and observation must be given as an integre
   * oridinal value of the actions andobservations in the POMDP
   * file. This would be the index number corresponding to the
   * positional location of a mnemonic in the list (or simply the
   * 'i'th action/observation if no mnemomic list existed in the POMDP
   * file.)
   *
   * The output belief states will conform to the python default
   * syntax for array output: delimited by square brackets, with comma
   * separators between the elements in the array.
   *
   * All input and output will be line-based text.  Thus, the input
   * should be on a single line, e.g.
   *
   *    action  obs
   *
   * Inputting the line consisting only of the word 'exit' will cause the
   * program to terminate.
   *
   * The output will be the next belief state and will appear on a single
   * line as: 
   *
   *    [ O1, O2, ..., On ]
   *
   * The program will output the resulting belief state to stdout.  If
   * there are input errors, then a message will be displayed on
   * stderr and the return value will be an empty list ('[]'): An
   * input error will function as a no-op, so that the next input will
   * use the last good belief state in its update calculation.
   *
   * The one other possible output is a vector of all zeroes. This may
   * be the result of an input error, or more likely the result of a
   * modeling error (problem in POMDP file).  This vector of all
   * zeroes is generated when the program determines that based on the
   * current belief state and action taken, the observation given is
   * not possible (has zero probability).  This means that something
   * is inconsistent in the sense that something is generating an
   * observation that the model has decalred impossible.
   *
   */ 
   
  long action, obs;
  double *prev_belief, *next_belief;

  char line[1024];
  char *post_action_ptr;
  char *post_obs_ptr;

  char line_init[255];

  prev_belief = newBelief();
  next_belief = newBelief();

  copyBelief( prev_belief, gInitialBelief );

  while ( 1 ) 
    {
	 UTIL_displayBelief( prev_belief );

	 /* UTIL_getline() returns -1 on EOF (and other errors). */
	 if ( UTIL_getLine( stdin, line, 1023 ) == EOF )
	   break;

	 sscanf( line, "%s", line_init );

	 if ( strcmp( line_init, "exit" ) == 0 )
	   break;
	 
	 action = strtol( line, &post_action_ptr, 10 );

	 if ( post_action_ptr == line )   /* nothing parsed */
	   {
		fprintf( stderr, "Bad line input.\n" );
		fprintf( stdout, "[]\n" );
		continue;
	   }
	
	 if (( action < 0 ) || ( action >= gNumActions ))
	 {
		fprintf( stderr, "Bad action index.\n" );
		fprintf( stdout, "[]\n" );
		continue;
	 }

	 obs = strtol( post_action_ptr, &post_obs_ptr, 10 );

	 if ( post_obs_ptr == post_action_ptr )   /* nothing parsed */
	   {
		fprintf( stderr, "Bad line input.\n" );
		fprintf( stdout, "[]\n" );
		continue;
	   }
	
	 if (( obs < 0 ) || ( obs >= gNumObservations ))
	 {
		fprintf( stderr, "Bad observation index.\n" );
		fprintf( stdout, "[]\n" );
		continue;
	 }

	 /* Make sure there is not extraneous stuff around. */
	 while ( post_obs_ptr[0] != '\0' )
	   {
		if (( post_obs_ptr[0] != ' ' )
		    && ( post_obs_ptr[0] != '\t' )
		    && ( post_obs_ptr[0] != '\r' )
		    && ( post_obs_ptr[0] != '\n' ))
		  break;

		post_obs_ptr++;
	   }

	 if ( post_obs_ptr[0] != '\0' )
	   {
		fprintf( stderr, "Bad line input.\n" );
		fprintf( stdout, "[]\n" );
		continue;
	   }
 
	 transformBeliefState( prev_belief, next_belief, action, obs );

	 copyBelief( prev_belief, next_belief );
    }

} /* UTIL_doBeliefUpdates */

/**********************************************************************/
