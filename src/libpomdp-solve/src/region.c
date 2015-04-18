
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    region.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    August, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: region.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/region.c,v $
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
 *   All the routines that help to create a parsimonious representation
 *   and for finding region points.
 */


#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"
#include "alpha.h"
#include "stats.h"
#include "lp-interface.h"
#include "region.h"

/**********************************************************************/
int 
countNonZeroesAlpha( double *alpha, double epsilon ) 
{
  /*
    Just goes through each component of the vector and
    compares the component to zero.  Returns the number of non-zero
    components.
  */
  int i;
  int count = 0;

  Assert( alpha != NULL, "Vector is NULL." );

  for ( i = 0; i < gNumStates; i++ )
    if ( ! Equal( alpha[i], 0.0, epsilon ))
      count++;
  
  return ( count );
}  /* countNonZeroesAlpha */
/**********************************************************************/
int 
countNonZeroesList( AlphaList list, double epsilon ) 
{
  /*
    Just goes through each component of each vector in the list and
    compares the component to zero.  Returns the number of non-zero
    components.
  */
  int i;
  int count = 0;

  Assert( list != NULL, "List is NULL." );

  list = list->head;
  while ( list != NULL ) {
    
    for ( i = 0; i < gNumStates; i++ )
      if ( ! Equal( list->alpha[i], 0.0, epsilon ))
        count++;
    
    list = list->next;
  } /* while */
  
  return ( count );
}  /* countNonZeroesList */
/**********************************************************************/
int 
countNonZeroesDiff( double *alpha, AlphaList list, double epsilon ) 
{
  /* 
     Just goes through each component of each vector in the list and
     compares the component to the alpha[i] component to see whether the
     difference between the two would yield a zero value.  Returns the
     number of non-zero differences.  
  */
  int i;
  int count = 0;
  double diff;

  Assert( list != NULL && alpha != NULL, 
          "Bad (NULL) parameter(s)." );

  list = list->head;
  while ( list != NULL ) {
    
    for ( i = 0; i < gNumStates; i++ )
      if ( ! Equal( alpha[i], list->alpha[i], epsilon ))
        count++;
    
    list = list->next;
  } /* while */
  
  return ( count );
}  /* countNonZeroesDiff */
/**********************************************************************/
void 
setUpObjectiveFunction( LP lp ) 
{
  /*
    Sets up the objective function coefficients and sense.

    Note that we do not keep the objective function in a sparse
    structure, so this is the same regardless of whether we are using
    sparse LPs or not.
  */
  int i;

  Assert( lp != NULL, "LP is NULL." );

  /* We will maximize the objective function. */
  lp->objsen = MAXIMIZE;

  /* The objective function is also specific to the LP we will solve
     during solution of POMDP. Set the objective function. */
  for( i = 0; i < lp->cols; i++ )
    lp->obj[i] = 0.0;
  
  /* ...then set the extra variable (delta) coefficient to '1'. Note
     that regardless of whether USE_NEW_LP_FORMULATION is set or not,
     the delta variable appears in the same position.  The new
     variable added by USE_NE_LP_FORMU:ATION is the last variable
     whose position would be right after the delat variable. */
  lp->obj[gNumStates] = 1.0;  /* maximize delta */

  /* For all the belief state variables in the LP, we know that must
     be >=0 and <=1, so we set their bounds here. */
  for( i = 0; i < gNumStates; i++ ) {
    lp->lowbnd[i] = 0.0;
    lp->upbnd[i] = 1.0;
  } /* for i */

  /* For the rest of the columns, we just make the lowerbound 0.0 and
     the upper bound infinite.  */
  for( i = gNumStates; i < lp->cols; i++ ) {
    lp->lowbnd[i] = 0.0;
    lp->upbnd[i] = INFBOUND;
  } /* for i */

 }  /* setUpObjectiveFunction */
/**********************************************************************/
void 
setUpRegionConstraintsOld( LP lp, double *alpha, 
			   AlphaList orig_list ) 
{
  /*
    Sets up all the constraints for this LP. Sets the coefficients in
    lp->matval, the sense, the RHS and the bookkeeping arrays
    lp->matbeg, lp->matcnt, lp->matind defining the start and length of
    columns and the row numbers for each lp->matval entry.
    
    This routine will work with or without a sparse LP formulation.
    
    This routine uses the old region formulation. 
    The LP looks like:

    max: delta
    s.t.
       x * 1 = 1
       x * ( item - listitem ) >= delta, for all listitem in list
       
       where x is a vector of variables and item and iotem list are
       linear hyperplane coefficients (i.e., alpha vectors).  Delta isan
       LP variable and the region has measurable volume if delta > 0. 
  */
  int index, row, col;
  AlphaList list;

  /* This routine should only be called if this constant is *not* set,
     otherwise the LPs will not be allocated with too much space. */
#ifndef USE_OLD_LP_FORMULATION
  Abort( "Can only use this function when USE_OLD_LP_FORMULATION is set." );
#endif

  Assert( lp != NULL && alpha != NULL && orig_list != NULL,
          "Bad (NULL) parameter(s)." );

  /* Iterate over the constraint coefficient matrix column by
     column. 'index' will be the current index into lp->matval and we
     will increment it everytime we set it. */
  index = 0;

  /****************************************************/
  /* Columns corresponding to belief state variables. */
  /****************************************************/

  for ( col = 0; col < gNumStates; col++ ) {

    /* Need to set the bookkeeping array for where this column
       starts. */
    lp->matbeg[col] = index;

    /* Since the first constraint is the simplex constraint, the first
       thing we need to do for this belief state variable (each column
       is a belief state variable) is set the coefficient to '1'. */
    lp->matval[index] = 1.0;
    lp->matind[index++] = 0;

    /* Looping over this list is now looping over the rows, but
       starting at row = 1, since The simplex constraint was row
       '0'. */ 
    row = 1;
    list = orig_list->head;
    while ( list != NULL ) {

#ifndef USE_DENSE_LPS
    if ( ! Equal( list->alpha[col], alpha[col], lp->sparse_epsilon ))
#endif
      {
        lp->matval[index] = list->alpha[col] - alpha[col];
        lp->matind[index++] = row;
      }

      list = list->next;
      row++;
    } /* while list != NULL */

    /* We can compute the number of entries we entered by looking at
       the current index position and where this column started. */
    lp->matcnt[col] = index - lp->matbeg[col];

  } /* for col */

  /*******************************************/
  /* Column corresponding to delta variable. */
  /*******************************************/

  /* Now we add the column corresponding to the delta varible. This
     will be '1.0' for every constraint row of the vector list and
     zero for the simplex constraints (which we only add if we are
     using dense matrices.) */

    lp->matbeg[gNumStates] = index;

#ifdef USE_DENSE_LPS
    lp->matval[index] = 0.0;
    lp->matind[index++] = 0;
#endif
  for ( row = 1; row < (orig_list->length + 1); row++ ) {
    lp->matval[index] = 1.0;
    lp->matind[index++] = row;
  } /* for row */
    
  lp->matcnt[gNumStates] = index - lp->matbeg[gNumStates];

  /* After all is said and done, we should have entered the same
     number of entries as we computed we would need when we allocated
     the LP. */
  Assert( index == lp->matspace, 
          "Computed non-zeroes didn't match actual non-zeroes." );

  /***********************************************/
  /* Constraint RHS and sense.                   */
  /***********************************************/

  /* Simplex constraint. */
  lp->sense[0] = 'E';
  lp->rhs[0] = 1.0;
  
  /* Remainder of constraints. */
  for ( row = 1; row < (orig_list->length + 1); row++ ) {
    lp->sense[row] = 'L';
    lp->rhs[row] = 0.0;
  } /* for row */

}  /* setUpRegionConstraintsOld */
/**********************************************************************/
void 
addExtraVarColumn( LP lp, int col, int *index, double sign ) 
{
  /*
    With the new LP region formulation we need to add two extra LP
    variables.  The two extra variables added will represent the value
    of the current vector being tested.  Because the LPs only deal with
    positive variable values, we need to represent the value as the
    difference between to positive variables.  Adding the columns for
    these is identical, except for the sign change, which is done with
    the parameter sent in.
  */
  int row;
  
  lp->matbeg[col] = *index;

  /* If we using dense LPs, we need to explicitly put a zeroe in for
     these variables in the simplex constraint (constraint row 0). */
#ifdef USE_DENSE_LPS
  lp->matval[*index] = 0.0;
  lp->matind[(*index)++] = 0;
#endif
  /* The extra constraint coef value in the first constraint... */
  lp->matval[*index] = sign;
  lp->matind[(*index)++] = 1;

  /* ...and now the value for each list vector constraint. */
  for ( row = 2; row < lp->rows; row++ ) {
    
    /* There are some sign changes that need to be done depending upon
       whether the POMDP immediate values represent rewards or
       costs. */ 
    lp->matval[*index] = sign;
  
    lp->matind[(*index)++] = row;
  } /* for row */

  lp->matcnt[col] = *index - lp->matbeg[col];

}  /* addExtraVarColumn */
/**********************************************************************/
void 
setUpRegionConstraintsNew( LP lp, double *alpha, 
			   AlphaList orig_list ) 
{
  /*
    Sets up all the constraints for this LP. Sets the coefficients in
    lp->matval, the sense, the RHS and the bookkeeping arrays
    lp->matbeg, lp->matcnt, lp->matind defining the start and length of
    columns and the row numbers for each lp->matval entry.

    This routine will work with or without a sparse LP formulation.

    This particular set-up was proposed by Bob Givan and has
    the advantages:
   
   1) Does not require a substraction computation for each state and
      vector. 
   2) Allows a new vector to be compared to the same list by changing
      just one constraint.  

  The minor disadvantages are that it requires one more variable and
  one more constraint that the original method.  I reality, we
  actually have to add two more variables to the LP, since the extra
  variable that is added is unbounded in range.  The standard LP
  strick is to replace an unbounded varkable as the difference of two
  positive variables.  We employ this trick here because lp_solve only
  deals with variables >= 0.  

  This routine will only work if the LP coefficient matrix is dense!

  The LP looks like this for immediate rewards:

     max: delta
     s.t.
 	  x * 1 = 1
 	  x * alpha - v1 + v2 = 0
       x * alphatilde + delta - v1 + v2 <= 0, for all alphatilde in list

  and  looks like this for immediate cost POMDPs:

    max: delta
    s.t.
 	 x * 1 = 1
 	 x * alpha - v1 + v2 = 0
      v1 - v2 - x * alphatilde + delta <= 0, for all alphatilde

   where x is a vector of variables and alpha and alphatilde are
   linear hyperplane coefficients (i.e., alpha vectors).  Delta and
   'v1' and 'v2' are LP variables and the region has measurable volume
   if delta > 0.

*/
  int index, row, col;
  AlphaList list;

  /* This routine should only be called if this constant is set,
     otherwise the LPs will not be allocated with too little space. */
#ifdef USE_OLD_LP_FORMULATION
  Abort( "Cannot use this function unless USE_OLD_LP_FORMULATION is set." );
#endif

  Assert( lp != NULL && alpha != NULL && orig_list != NULL,
          "Bad (NULL) parameter(s)." );

  /* Iterate over the constraint coefficient matrix column by
     column. 'index' will be the current index into lp->matval and we
     will increment it everytime we set it. */
  index = 0;

  /****************************************************/
  /* Columns corresponding to belief state variables. */
  /****************************************************/

  for ( col = 0; col < gNumStates; col++ ) {
    
    /* Need to set the bookkeeping array for where this column
       starts. */
    lp->matbeg[col] = index;
    
    /* Since the first constraint is the simplex constraint, the first
       thing we need to do for this belief state variable (each column
       is a belief state variable) is set the coefficient to '1'. */
    lp->matval[index] = 1.0;
    lp->matind[index++] = 0;

    /* For the new LP formulation, the second constraint is the extra
       constraint which effectively assigns the vectors value to an
       extra variable. Set this coefficient now. */
#ifndef USE_DENSE_LPS
    if ( ! Equal( alpha[col], 0.0, lp->sparse_epsilon ))
#endif
      {
        lp->matval[index] = alpha[col];
        lp->matind[index++] = 1;
      }

    /* Looping over this list is now looping over the rows, but
       starting at row = 2, since The simplex constraint was row '0'
       and the extra constraint was row '1'. */
    row = 2;
    list = orig_list->head;
    while ( list != NULL ) {

#ifndef USE_DENSE_LPS
    if ( ! Equal( list->alpha[col], 0.0, lp->sparse_epsilon ))
#endif
      {
        lp->matval[index] = list->alpha[col];
        lp->matind[index++] = row;
      }

      list = list->next;
      row++;
    } /* while list != NULL */

    /* We can compute the number of entries we entered by looking at
       the current index position and where this column started. */
    lp->matcnt[col] = index - lp->matbeg[col];

  } /* for col */

  /*******************************************/
  /* Column corresponding to delta variable. */
  /*******************************************/

  lp->matbeg[gNumStates] = index;

  /* Now we add the column corresponding to the delta varible. This
     will be '1.0' for every constraint row of the vector list and
     zero for the first two constraints (simplex and extra one),
     (which we only add if we are using dense matrices.) */
#ifdef USE_DENSE_LPS
    lp->matval[index] = 0.0;
    lp->matind[index++] = 0;
    lp->matval[index] = 0.0;
    lp->matind[index++] = 1;
#endif
  for ( row = 2; row < (orig_list->length + 2); row++ ) {
    lp->matval[index] = 1.0;
    lp->matind[index++] = row;
  } /* for row */
    
  lp->matcnt[gNumStates] = index - lp->matbeg[gNumStates];

  /************************************************/
  /* Column corresponding to the extra variables. */
  /************************************************/

  /* We now add the two columns that corresponds to the extra
     varibles.  These will appear in every constraint but the simplex
     constraint, though its sign may be different in rows 2 upward due
     to the reward/cost immediate reward issue. We only add the zero
     for the simplex consraint if we are using dense matrices. */

  addExtraVarColumn( lp, lp->cols-2, &index, -1.0 );
  addExtraVarColumn( lp, lp->cols-1, &index, 1.0 );

  /* After all is said and done, we should have entered the same
     number of entries as we computed we would need when we allocated
     the LP. */
  Assert( index == lp->matspace, 
          "Computed non-zeroes didn't match actual non-zeroes." );

  /***********************************************/
  /* Constraint RHS and sense.                   */
  /***********************************************/

  /* Simplex constraint. */
  lp->sense[0] = 'E';
  lp->rhs[0] = 1.0;
  
  /* Extra consraint. */
  lp->sense[1] = 'E';
  lp->rhs[1] = 0.0;
  
  /* Remainder of constraints. */
  for ( row = 2; row < (orig_list->length + 2); row++ ) {
    lp->sense[row] = 'L';
    lp->rhs[row] = 0.0;
  } /* for row */

}  /* setUpRegionConstraintsNew */
/**********************************************************************/
LP 
setUpRegionLP( double *alpha, AlphaList list, double sparse_epsilon ) 
{
  int num_constraints, num_variables, num_non_zeroes;
  LP lp;

  /* Calculate the LP size for the new LP formulation. */
#ifdef USE_OLD_LP_FORMULATION
  num_constraints = list->length + 1;
  num_variables = gNumStates + 1;
  
#ifdef USE_DENSE_LPS
  /* Assuming all are non-zeroes, every variable has an entry for
     every constraint except the delta variable in the simplex
     constraint. */
  num_non_zeroes = num_constraints * num_variables;
#else
  /* If we are using sparse LPs, we need to count the non-zero
     entries.  This is actually a combination of counting and
     calculating based upon knowledge of the problem. 

     Here we have a non-zero for each belief state variable in the
     simple constraint, then will have one for each component
     comparison for each vetcor in the list.  Finally, each comparison
     of this sort will have a non-zero coef for the delta coef. */
  num_non_zeroes 
    = gNumStates
    + countNonZeroesDiff( alpha, list, sparse_epsilon )
    + list->length;
#endif

#else  /* Using the NEW region formulation. */
  num_constraints = list->length + 2;
  num_variables = gNumStates + 3;

#ifdef USE_DENSE_LPS
  /* Assuming all are non-zeroes, every variable has an entry for
     every constraint. */
  num_non_zeroes = num_constraints * num_variables;
#else
  /* If we are using sparse LPs, we need to count the non-zero
     entries.  This is actually a combination of counting and
     calculating based upon knowledge of the problem. 

     Here we have a definite '1' coef. for the belief state variables
     in the simplex constraint.  Then there is the extra constraint
     which is just the number of non-zeroes in the vector being
     compared, plus the extra variable we introduced to force this
     vector's value to be equal to this variable. For each vector in
     the list, we will have at least two non-zero coefs correponding
     to the delta and extra variable, with more entries for each
     non-zero component of the vector.  */

  /* zzz Not sure that these are the proper epsilons to use, but just
     put them in so it would compile. */
  num_non_zeroes 
    = gNumStates
    + countNonZeroesAlpha( alpha, sparse_epsilon ) + 2
    + countNonZeroesList( list, sparse_epsilon ) 
    + 3 * list->length;
#endif

#endif

  lp = LP_newLP( num_constraints, num_variables, num_non_zeroes );
  lp->sparse_epsilon = sparse_epsilon;

  setUpObjectiveFunction( lp );

#ifdef USE_OLD_LP_FORMULATION
  setUpRegionConstraintsOld( lp, alpha, list );
#else
  setUpRegionConstraintsNew( lp, alpha, list );
#endif

  return ( lp );
}  /* setUpRegionLP */
/**********************************************************************/
int 
findRegionPoint( double *alpha, AlphaList list, 
		 double *witness_point, double *diff,
		 PomdpSolveParams param ) 
{
  /*
    Checks to see if the alpha vector 'alpha' has a non-empty region
    (measurable area) where it is better than all the other vectors in
    the 'list'. If the region is non-empty the routine returns TRUE with
    the witness_point set to a point in that region.  If there is no
    point where alpha is better, then FALSE is returned.
  */
  LP lp;
  int i;

  Assert( alpha != NULL && list != NULL && param != NULL,
          "Vector or list is NULL." );

  /* If the list is initially empty, then we know that any simplex
     point is a witness to the vector 'alpha' being bettwr than the
     list.  For this case we just return a simplex corner and forego
     the LPs.  Note that letting this empty list case pass through
     will not work, since the setUpRegionLp() doesn't not set up the
     proper LP for this case. */ 
  if ( list->length == 0 ) {

    if ( witness_point != NULL ) {
      witness_point[0] = 1.0;
      for( i = 1; i < gNumStates; i++ )
        witness_point[i] = 0.0;
    } /* if need to return a witness point. */

    if ( diff != NULL )
      *diff = HUGE_VAL;

    return ( TRUE );
  }

  /* Set up constraints and rest of memory. */
  lp = setUpRegionLP( alpha, list, param->sparse_epsilon );

  /* See if we get a feasible solution to the LP, but if not just
     return FALSE. */
  switch ( LP_solveLP( lp, param->stat )) {
  case LP_OPTIMAL:
    /* Ok, fall through to code below. */
    break;

  case LP_INFEASIBLE:
    LP_freeLP( lp );
    return ( FALSE );

  case LP_UNBOUNDED:
    /* We will assume that an unbounded LP has resulted from numerical
       precision/instability issues and just report this as having no
       region. */
    Warning( "LP return status is unbounded. Assuming infeasible." );
    LP_freeLP( lp );
    return ( FALSE );

  default:
    Abort( "LP return status is unknown." );
  }

  /* If the 'diff' argument is not NULL then we return the objective
     value in it. */
  if ( diff != NULL )
    *diff = lp->objval;

  /* If the LP is feasible, then we need to make sure the objective
     value is > 0.  We want to objective value to be greater than
     zero, but use an epsilon factor for two reasons: first, numerical
     stability requires this and second, it provides the place where
     optimization can be achieved by boosting this up. Since the
     objective function consists of a single variable with a
     coefficient of '1', the objective value and the solution value of
     that variable should be identical. Alas, this does not always
     seem to be the case because of precision issues.  Thus we only
     claim the result is larger than zero if *both* the objective
     value and the variable value are larger than the epilon value we
     are interested in. */
  if( ( lp->objval < param->epsilon )
      || ( lp->x[gNumStates] < param->epsilon ))   {
    LP_freeLP( lp );
    return ( FALSE );
  }

  /* Just copy solution, if it was wanted. */
  if ( witness_point != NULL )
    for( i = 0; i < gNumStates; i++ )
      witness_point[i] = lp->x[i];

  LP_freeLP( lp );
  return ( TRUE );

}  /* *findRegionPoint */
/**********************************************************************/
