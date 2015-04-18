
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    alpha.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: alpha.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/alpha.h,v $
 *    $Revision: 1.4 $
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
 *   Stuff for alpha vectors and alpha-vector lists.
 * 
 *   Each list will have a header list with a pointer to the first and
 *   last element in the list.
 * 
 */

#ifndef ALPHA_H
#define ALPHA_H 1

#include <stdio.h>

/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/* When writing alpha vectors out to a file, how many decimal points
   should we use. */
#define ALPHA_FILE_DECIMAL_PRECISION          25

/**********************************************************************/
/********************   DEFAULT VALUES       **************************/
/**********************************************************************/

/* Defines the epsilon to use for the floating point comparison of
   alpha vector components in determining if two vectors are the
   same. */
#define DEFAULT_ALPHA_COMPARE_EPSILON         1.0e-9

/**********************************************************************/
/********************    TYPEDEFS            **************************/
/**********************************************************************/

typedef struct AlphaListType *AlphaList;
struct AlphaListType {

  /* Each vector can have an associated action, or the header of the
     list can have an action for all vectors in the list. */ 
   int action;

  /* When dealing with a set that represents a projection of the
     previous value function, we may also want to keep track of the
     observation used in the projection.  For othert sets this
     variable will have no useful value. This will only have meaning
     in the header of a list. */
  int obs;

  /* A vector of length gNumStates representing the actual vector */
  double *alpha;

  /* A unique identifier for elements, usually the list position */
  int id;

  /* Pointer to next vector i the list. */
  AlphaList next;

  /* Sometimes we will want to save a witness point for a vector. This
     is the point that we used to determine that this vector was indeed
     a useful vector.  This will only exist for vectors in a
     parsimonious representation. */
  double *witness;

  /* Also, when we have a projection set, we will want to maintain a
     pointer into the previous alpha vector set which this vector is
     the projection of. This field is used when the vector is a
     projection of a previous alpha list vector. */
  AlphaList prev_source;

  /* It will also be useful (especially for the generalized cross-sum)
     to  know the two vectors that were immediately used to create a
     vector.  These two fields hold this information when a vector is
     created via a cross sum operation. */
  AlphaList first_source;
  AlphaList second_source;
  
  /* When constructing V_t from V_{t-1} we will use a vector from
     V_{t-1} for each observation in the construction of a vector
     in V_t.  We would like to keep track of which ones we used
     so that we can trace out the policy tree.  This array will be
     pointers, but the confusing part is that at one point they are
     pointers into the projection sets and at another they are
     pointers into the previous alpha list.  They are mostly the
     former, but just before the policy graph stuff is used these are
     set to point directly into the previous alpha list.*/
  AlphaList *obs_source;
  
  /* General purpose pointer for miscellaneous things. */
  void *hook;

  /* A flag to use to mark and unmark vectors.  Right now Sondik uses
     this to determine which vectors have already been looked at. */
  int mark;

  /* For list headers */
  int length;
  AlphaList head;
  AlphaList tail;

};

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/**********************************************************************/
/******************   Alpha Vector Routines      **********************/
/**********************************************************************/

/* Allocate memory for an alpha vector, whose size is determined by
   the number of states.  */
extern double *newAlpha( );


/* Makes a copy of the alpha vector also allocating the memory for it.  */
extern double *duplicateAlpha( double *alpha );

/* Assumes the memory has been allocated and simply copies the values
  from the src to the dest argument.  */
extern void copyAlpha( double *dest, 
                       double *src );

/* Free the memory for an alpha vector. */
extern void destroyAlpha( double *alpha );

/* Compares two alpha vectors and determines if they are the identical
   vector.  The tricky part here is that there is floating point
   comparisons that we need to deal with and that can have a
   substantial effect on the algorithm. */
extern int sameAlpha( double *alpha1,
                      double *alpha2,
                      double epsilon);

/* Just checks to see if all components are zero.  Will return TRUE if
   it is zero of if NULL is sent in and FALSE otherwise.  */
extern int isZeroAlpha( double *alpha,
                        double epsilon );

/* Display the vector to the file stream putting no extra whitespace
  before or after the vector.  */
extern void displayAlpha( FILE *file, 
                          double *alpha );

/* Displays vector to stdout.  */
extern void showAlpha( double *alpha );

/* Does a lexicographic check on two vectors, given the two vectors.
   Return TRUE if first_alpha is lexicographically better than
   second_alpha.  */
extern int isLexicographicallyBetterAlpha( double *first_alpha,
                                           double *second_alpha,
                                           double epsilon );

  /* Does a lexicographic check on two vectors pointed two by the two
     list nodes.  */
extern int isLexicographicallyBetter( AlphaList first_alpha,
                                      AlphaList second_alpha,
                                      double epsilon );

/* Returns true if alpha2 is component-wise dominated by alpha1. The
   assumption here is that with two identical vectors neither would be
   considered dominating toward the other.  */
extern int isDominatedVector( double *alpha1, 
                              double *alpha2 );

/**********************************************************************/
/******************  Obs_Source Routines         **********************/
/**********************************************************************/

/* Just a convenient function for getting a pointer to an array of
  AlphaList pointers for the obs_source field of the AlphaList
  nodes. Initializes the array to have all NULL vectors.  */
extern AlphaList *newObsSourceArray(  );

/* Allocates memory for and copies the obs_source array and returns a
  pointer to the new memory.  */
extern AlphaList *duplicateObsSourceArray( AlphaList *orig_obs_source );
  
/**********************************************************************/
/******************  Alpha List Node Routines    **********************/
/**********************************************************************/

/* Allocates the memory for and sets initial values for an alpha list
  node.  */
extern AlphaList newAlphaNode( double *alpha, 
                               int action );

/* Allocates the memory for and sets initial values for an alpha list
   node.  Also allocates the mmeory for the obs_source array. */
extern AlphaList newAlphaNodeObsSource( double *alpha, 
                                        int action );

/* Frees the memory for an alpha list node. Also free some
  supplemental memory that might be hanging off this.  */
extern void destroyAlphaNode( AlphaList temp );

/* Adds the node to the end of the list.  */
extern void appendNodeToAlphaList( AlphaList list, 
                                   AlphaList node );

/* Adds the node to the beginning of the list.  */
extern void prependNodeToAlphaList( AlphaList list, 
                                    AlphaList node );

/* Removes the first item in the list and returns it.  */
extern AlphaList dequeueAlphaNode( AlphaList list );

/* Puts an alpha list node at the end of the list.  */
extern void enqueueAlphaNode( AlphaList list, 
                              AlphaList node );

/* Allocates the memory and copies an AlphaList node. Copies pointers
  if it has any, but not objects they point to. The slight exception
  is the obs_source array.  It makes new space for this duplicate
  node's obs_source and then copies the pointers.  */
extern AlphaList duplicateAlphaNode( AlphaList node );

/* Make a copy of the node and appends it to the list.  Returns a
  pointer to the newly created node.  */
extern AlphaList appendDuplicateNodeToAlphaList( AlphaList list, 
                                                 AlphaList orig_node );

/* Adds a witness point to the alpha list.  This has to be more than
  simply setting the pointer, since typically the witness point comes
  from an LP which re-uses the memory for the solution point.  Thus we
  need to allocate the memory for the witness point and then copy it.  */
extern void addWitnessToAlphaNode( AlphaList node, 
                                   double *witness );


/**********************************************************************/
/******************  Alpha List Routines         **********************/
/**********************************************************************/

/* Sets the initial values form the node representing the header of an
  AlphaList.  */
extern void initAlphaList( AlphaList list );

/* Allocates the memory for the header node of a new alpha list.  */
extern AlphaList newAlphaList(  );

/* Renumbers the alpha list so vectors are numbered sequentially.  */
extern void renumberAlphaList( AlphaList list );

/* Puts an alpha node at the beginning of the list and retruns a
  pointer to the node added.  */
extern AlphaList prependAlphaList( AlphaList list,
                                   double *alpha,
                                   int action );

/* Puts an alpha node at the end of the list and retruns a pointer to
  the node added.  */
extern AlphaList appendAlphaList( AlphaList list,
                                  double *alpha,
                                  int action );

/* Frees the memory for each node in the list and resets the header
  node to reflect an empty list.  */
extern void clearAlphaList( AlphaList orig_list );

/* Comletely frees up the memory for the entire list, including all
  nodes and the header node.  */
extern void destroyAlphaList( AlphaList list );

/* Takes a list of alpha vectors and a belief state and returns the
   value and vector in the list that gives the maximal value.  If
   there are ties, then we must invoke the tie breaking scheme using
   the lexicographic comparison of the vectors.  The function returns
   the value, and the best_ptr returns the vector, the initial value
   serves as the initial value to use.  If no vectors are better than
   the initial value, then NULL is returned.

   We arbitrarily define that a vector that is equal to the initial
   value is automatically *not* better (since lexicographic check
   cannot be done with no vector.)  */
extern double bestVectorValuePrimed( AlphaList list, 
                                     double *belief_state,
                                     AlphaList *best_ptr,
                                     double initial_value,
                                     double epsilon );

/* Just calls bestVectorValuePrimed with the worst possible value to
  ensure some vector will be the best.  */
extern double bestVectorValue( AlphaList list, 
                               double *belief_state,
                               AlphaList *best_ptr,
                               double epsilon );

/* Takes a list of alpha vectors and a belief state and returns the
  vector in the list that gives the maximal value.  If there are ties,
  then we must invoke the tie breaking scheme using the lexicographic
  comparison of the vectors.  */
extern AlphaList bestVectorPrimed( AlphaList list, 
                                   double *belief_state, 
                                   double *best_value,
                                   double initial_value,
                                   double epsilon );

  /* Takes a list of alpha vectors and a belief state and returns the
  vector in the list that gives the maximal value.  If there are ties,
  then we must invoke the tie breaking scheme using the lexicographic
  comparison of the vectors.  */
extern AlphaList bestVector( AlphaList list, 
                             double *belief_state, 
                             double *best_value,
                             double epsilon );

/* This routine is basically the same as the queryAlphaList() routine
   in alpha.c except it returns a pointer to the list node that
   contains the vector 'alpha' of interest if it is found.  It returns
   NULL if the vector is not in the list.  */
extern AlphaList findAlphaVector( AlphaList list,
                                  double *alpha,
                                  double epsilon );

/* Returns TRUE if the alpha vector parameter is in the list.  */
extern int queryAlphaList( AlphaList list, 
                           double *alpha,
                           double epsilon );

/* Just get the number of alpha vectors in the list by accessing the
  variable in the header.  */
extern int sizeAlphaList( AlphaList list );

/* Doesn't copy the obs_source or witness point fields, leaves them
  blank.  */
extern void copyAlphaList( AlphaList dest_list,
                           AlphaList src_list );

/* Allocates a new list and copies the src_list into it.  */
extern AlphaList duplicateAlphaList( AlphaList src_list );

/* Duplicates the alpha list just like the duplicateAlphaList()
  method, but also copies the witness fields.  */
extern AlphaList duplicateAlphaListWithWitnesses( AlphaList list );

/* Just checks if the two lists contain the same alpha_vectors in
  exactly the same order.  */
extern int sameAlphaList( AlphaList l1, 
                          AlphaList l2,
                          double epsilon );

/* Returns true if the two alpha lists contains the same alpha
  vectors, though the order is not important.  */
extern int similarAlphaList( AlphaList list1,
                             AlphaList list2,
                             double epsilon );

/* Force roundin tor epsilon precision */
extern void roundAlphaList( AlphaList list, double precision ); 
 
/* Printout a textual version of the list.  */
extern void displayAlphaList( FILE *file, 
                              AlphaList list );

/* Printout to stdout, Especially useful in debugger.  */
extern void showAlphaList( AlphaList list );

/* Reads a list of alpha vectors from a file.  The format of the file
  is ver specific and does not allow comments.  It simply reads a
  sequence of numbers which are assumed to be in the correct order.
  This should only be used to read in things written out by the code.
  Alos, there is no checking to make sure the file is consistent with
  the problem. i.e., if the probelm has 3 states and you read a file
  of 4 component vectors, this will not notice and might result in
  strange things.  It does a simple check of this by seeing if it is
  in the middle of a vector when the file ends.  However, this does
  not guarantee anything.

  It can also read only a subset of the file of vectors.
  Set max_alphas to <= 0 if you want the whole file read, otherwise
  it will only read the first max_alphas in the file.  */
extern AlphaList readAlphaList( char *filename, 
                                int max_alphas );

/* Writes to a file handle. It is not a very pretty format,
   but makes reading it in trivial.  */
extern void writeAlphaList( FILE *file, AlphaList list );

/* Write the alpha list out to a file in the format that
  readAlphaList() will read them in.   */
extern void saveAlphaList( AlphaList list, 
                            char *filename );

/* Takes the union of the two lists sent in and returns the union in
  the 'list' argument.  It is a destructive union, since effectively
  all nodes in the other_list are moved to this list.  */
extern void unionTwoAlphaLists( AlphaList list, 
                                AlphaList other_list
);

/* Clears any memory for the 'choice' list for the nodes in the list.  */
extern void clearObsSourceAlphaList( AlphaList list );

/* Appends a new node to the alpha list for a vector but only if this
  vector does not already exist in the list.  Returns a point to the
  new node created.  */
extern AlphaList appendUniqueAlphaList( AlphaList list,
                                        double *alpha,
                                        int action,
                                        double epsilon );

/* Returns TRUE if any of the alphas in the list dominate
  (component-wise) the first argument alpha vector.  */
extern int dominatedAlphaList( double *alpha, 
                               AlphaList list );

/* Sets all the nodes in the list to have their 'mark' field FALSE.  */
extern void clearMarkAlphaList( AlphaList list );

/* Sets all the nodes in the list to have their 'mark' field TRUE.  */
extern void markAllAlphaList( AlphaList list );

/* Returns the number of nodes whose 'mark' field is FALSE.  */
extern int sizeUnmarkedAlphaList( AlphaList list );

/* Returns true if all the node in the list have their marked fields
  set.  */
extern int allMarkedAlphaList( AlphaList list );

/* Returns a pointer to the first vector in the list where the 'mark'
   field is FALSE. If none exist, or the list is empty, it returns
   NULL.  (Doesn't remove the node fom the list.) */
extern AlphaList findUnmarkedVector(  AlphaList list );

/* Finds a vector node with the 'mark' field set to FALSE and extracts
   that node from the list returning a pointer to the node.  */
extern AlphaList extractUnmarkedVector( AlphaList list );

/* Checks and marks those vectors in 'list' which are dominated by the
  vector 'alpha' sent it.  Does not delete them yet, just sets their
  'mark' field to TRUE.  Assumes that the 'mark' field has already
  been cleared and it will not set any 'mark' fields to FALSE.  Any
  existing TRUE 'mark' fields will remain that way regardless of
  whether they are dominated or not.  */
extern int markDominatedAlphaList( double *alpha, 
                                   AlphaList list );

/* Removes all the nodes which have their 'mark' fields set to TRUE
  and puts them in a separate list which is returned.  */
extern AlphaList extractMarkedAlphaList( AlphaList list );

/* Removes all the nodes which have their 'mark' fields set to TRUE.
  Returns the number of nodes removed.  */
extern int removeMarkedAlphaList( AlphaList list );

/* Removes all vectors in the list that are component-wise dominated
  by the first argument alpha vector.  */
extern int removeDominatedAlphaList( double *alpha, 
                                     AlphaList list );

/* Take a pointer to one of the elements in the list and removes that
   node from the list.  It returns a pointer to the removed node
   (memory is not freed) or NULL if something goes wrong.  */
extern AlphaList extractAlphaNode( AlphaList list, 
                                   AlphaList extract_ptr );

/* Finds the vector with the highest dot-product value with 'b' and
   removes that node from the list.  It doesn't deallocate any memory
   and returns a pointer to the removed node, or NULL if something
   goes wrong.  */
extern AlphaList removebestVectorNode( AlphaList list, 
                                       double *b,
                                       double epsilon );
                                                      
/* If the vector sent in already exists in the list, then it is
  removed from the list and the pointer to the node is returned.  */
extern AlphaList extractFromAlphaList( AlphaList list, 
                                       double *alpha, 
                                       double epsilon );


/* If the alpha vector sent in is already in the list, then the node
  in the list is removed, and deallocated.  This routine returns TRUE
  if a node was removed and FALSE if the vector is not in the list.  */
extern int removeFromAlphaList( AlphaList list, 
                                double *alpha,
                                double episilon );

/* Sorts the list lexicographically.  */
extern void sortAlphaList( AlphaList list );

/* This routine will return a duplicate of the list sent in, but with
   its rewards converted.  */
extern AlphaList makeScaledAlphaList( AlphaList list, 
                                      int num_updates );

/* Takes an array of alpha lists and returns the maximum size over all
  lists.  */
extern int maxSizeAlphaLists( AlphaList *list, int num_lists );
       
#endif

