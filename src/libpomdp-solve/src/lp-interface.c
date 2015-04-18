/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    lp-interface.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: lp-interface.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/lp-interface.c,v $
 *    $Revision: 1.4 $
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
 *   Module that defines the LP interface functions needed by the
 *   pomdp-solve program. It will either call the appropriate CPLEX or
 *   lp_solve library function depending upon the compilation
 *   configuration. 
 * 
 *   Actually using an LP after it is allocated is a matter of first
 *   creating and allocating the LP data structure, then setting all the
 *   constraints and objective rows, solving or optimizing the problem
 *   and then finally, retrieving the solution results.
 * 
 *   Thus the general scheme for an LP for a POMDP with 'm' states and an
 *   LP with 'm' constraints goes something like:
 * 
 *    LP lp;                       # Pointer to LP data structure. 
 *    lp = LP_newLP( m, n );       # Only need to call this once.
 *    result = LP_solveLP( lp );   # Solve LP and get solution.
 *    LP_freeLP( lp )              # Only need to call this at very end.
 * 
 *   Note that internall the LP_optimizeLP() routine does this general
 *   scheme to solve the LP instance: 
 *  
 *   LP_loadLP( lp );         # Tell solver to set things up.
 *   LP_optimizeLP( lp );     # Let the solver solve the LP.
 *   LP_getSolution( lp );    # Get the solution information.
 *   LP_unloadLP( lp );       # Tell solver you are done with LP.
 * 
 *   The main LP data structure have the variables that would be directly
 *   used by CPLEX.  Even if CPLEX is not being used, we will use the
 *   CPLEX structures to create and LP instance, but only when it comes
 *   time to solve the LP will it be converted into the proper format for
 *   the particular solver.  This concentrates all the LP dependency in a
 *   single spot and just requires writing a translator from the
 *   CPLEX-style LP to the LP solver available.  For non-CPLEX solving
 *   you alos need to write a translator that will extract the solution
 *   information into the CPLEX-like data structure.
 */

#include <stdio.h>
#include <stdlib.h>

#include "mdp/mdp.h"

#include "global.h"
#include "stats.h"
#include "params.h"
#include "lp-interface.h"


/* These are defined in lp_solve/solve.c and are not really used for
   the CPLEX version. */
extern int gLpSolveInstabilityCount;
extern int gShowInstabilityMessages;

/**********************************************************************/
/**********        LP Tolerances/Precision/Epsilons      **************/
/**********************************************************************/

/* We need to transmit the command line LP epsilon to the routine that
   create an LP structure.  This was added a little late in the game,
   so the easiest way to do this was to create a global variable which
   can be set and queried.  */
double gLpEpsilon = -1.0;

/**********************************************************************/
void 
LP_setPrecision( double epsilon ) 
{
/*
  Sets the precision that should be used when solving LPs.
*/

#ifdef HAVE_LIBCPLEX
  double p_too_small, p_too_big;
  
  /* Feasibility tolerance */
  if( seteprhs( epsilon, &p_too_small, &p_too_big ) )
    fprintf( stderr,
             "Feas. Tolerance problem: low=%.10e, high=%.10e\n",
             p_too_small, p_too_big);
  
  /* Markowitz tolerance (not sure what this means.) */
  if( setepmrk( epsilon, &p_too_small, &p_too_big ) )
    fprintf( stderr,
             "Mark. Tolerance problem: low=%.10e, high=%.10e\n",
             p_too_small, p_too_big);
  
  /* Optimality tolerance */
  if( setepopt( epsilon, &p_too_small, &p_too_big ) )
    fprintf( stderr,
             "Opt. Tolerance problem: low=%.10e, high=%.10e\n",
             p_too_small, p_too_big);
  
#endif

  gLpEpsilon = epsilon;

} /* LP_setPrecision */
/**********************************************************************/
double 
LP_getPrecision(  ) 
{
  return( gLpEpsilon );
}  /* LP_getPrecision */
/**********************************************************************/

/**********************************************************************/
/**********        LP_SOLVE Specific Routines            **************/
/**********************************************************************/
/* 
   There are a lot of little pain-in-the-neck indexing problems that
   need to be worked out when converting CPLEX structures to lp_solve.
   Here is the basis of all this math assuming a full dense
   coefficient matrix. Because the LP_loadLpSolveLP() actually copies
   the CPLEX sparsity, the computation is a little easier.  However,
   this is useful reference material.

     --------------
     LP_SOLVE MATH: 
     --------------

     Assuming the '1'-based indexing that lp_solve uses, and the
     storage of the objective row in the coefficient matrix, here is
     the calculation for the index into the 'mat' array for new value
     'coef' at position (row,col):

     mat[lp->rows * (col-1) + row].value = coef;

     If we were given the (row,col) in zero-based numbering, then we
     would have:

     mat[lp->rows * col + row + 1].value = coef;

     Going the other way around, computing the row and col for
     matval[i] is:

     row = i % lp->rows;
     col = (int) (i/(lp->rows) + 1);
     coef[row][col] = mat[i].value;

     -----------
     CPLEX MATH: 
     -----------

     Assuming the zero-based indexing and *no* objective row in the
     coefficint matrix here is the the calculation for the index into
     'matval' for new value 'coef' at position (row,col):

     matval[ lp->rows * col + row] = coef;

     Going the other way around, computing the row and col for
     matval[i] is:

     row = i % lp->rows;
     col = (int) (i/(lp->rows));
     coef[row][col] = matval[i];

     -------------------
     LP CONVERSION MATH:
     -------------------

     We now consider how to map the coeffiecient matrix of CPLEX to
     that of lp_solve.  Assuming we are looping over all indices of
     'matval' with index 'i' being under consideration, we have the
     zero-based:

     zero_row = i % lp->rows;
     zero_col = (int) (i/(lp->rows));

     but lp_solve is '1'-based indexing, so we really need:

     row = i % lp->rows + 1;
     col = (int) (i/(lp->rows)) + 1;
     
     Since row '0' of CPLEX constraints is row '1' of lp_solve. Now we
     know (from above) that to set ('1'-based) (row,col) element in
     the lp_solve matrix we need:

     mat[lp->rows * (col-1) + row],value = coef;
     
     So a strict replacement yields:

     mat[lp->rows * (((int) (i/(lp->rows)) + 1)-1) 
         + (i % lp->rows + 1)],value = coef;

     which happens to be equivalent to:

     mat[ i + i/lp->rows + 1],value = coef;

     which makes sense because each entry is just offset with the
     amount depending upon which column it is in, since this
     defines how many objective coefficients have preceeded it.  

     Since the objective coefficients are stored separately in CPLEX
     and they correspond to row '0' in lp_solve, objective coef 'i'
     (zero-based) goes into the matrix at position:

     mat[ i * (lp->rows+1) ].value = obj[i];
*/

/**********************************************************************/
void 
initLpInterface( PomdpSolveParams param ) 
{
  /*
    Initialize and allocate anything we might need for LPs.  For now, it
    just sets the flag that controls whether we display instability
    messages if using lp_solve and sets the precision tolerance to use.
  */

  /* This is defined in lp_solve/solve.c and is used to decide whetehr
     to show messages about numerically unstable LPs or just keep a
     count of them. */
  gShowInstabilityMessages = SHOW_INSTABILITY_MESSAGES;
  
  LP_setPrecision( param->lp_epsilon );

}  /* initLpInterface */
/**********************************************************************/
void 
cleanUpLpInterface(  ) 
{
  /*
    Free up any temporary memory that was allocated.
  */
  char msg[MAX_MSG_LENGTH];

#ifndef HAVE_LIBCPLEX

  /* For now just report if there were instability problems from
     lp_solve . */

  if ( gLpSolveInstabilityCount ) {
    sprintf( msg, 
             "lp_solve reported %d LPs with numerical instability.",
             gLpSolveInstabilityCount );
    Warning( msg );
  } /* if instability count non-zero */

#endif

}  /* cleanUpLpInterface */
/**********************************************************************/
lprec *
LP_make_lp( int rows, int columns, int non_zeroes ) 
{
  /*
    The lp_solve file lpkit.c contains the routine make_lp() which is
    used to initially allocate an LP structure.  It functions to
    initially assume there are no non-zero matrix elements and expand
    this as subsequent calls are made adding constraint and objective
    coefficients.  Because we do not want to incrementally allocate
    an LP, we use this routine, which is 95% the same as lpkit.c make_lp.
    Where things are different, I aded the :ARC suffix to my comments.
    
    The 'non_zeroes' parameter must include the constraint row *and*
    objective row coefficients.
    
    Note that we can still use lpkit.c's delete_lp() routine
    since there is nothing allocated here that isn't allocated in
    make_lp(). The exact changes from make_lp() are annotated below with
    :ARC suffix.  
  */
  lprec *newlp;
  int i, sum;  

  /* Just changed to require positive number of rows and cols and also
     the way the error message is delivered. :ARC */
  Assert( (rows > 0 && columns > 0),
          " Non-positive number of rows or columns." );

  sum = rows + columns;

  newlp = (lprec *) XCALLOC( (size_t) 1, sizeof(*newlp) );

  /* Might as well give it a meaningful name. :ARC */
  strcpy(newlp->lp_name, "pomdp-solve");

  newlp->verbose = FALSE;
  newlp->print_duals = FALSE;
  newlp->print_sol = FALSE;
  newlp->debug = FALSE;
  newlp->print_at_invert = FALSE;
  newlp->trace = FALSE;

  newlp->rows = rows;
  newlp->columns = columns;
  newlp->sum = sum;
  newlp->rows_alloc = rows;
  newlp->columns_alloc = columns;
  newlp->sum_alloc = sum;
  newlp->names_used = FALSE;

  newlp->obj_bound = DEF_INFINITE;
  newlp->infinite = DEF_INFINITE;

  /* Ideally, we would like to adjust these values to make the LP more
     or less sensitive. However, I get weird results when playing
     around with these numbers (it seems to be more sensitive when I
     make epsilon larger).  Therefore, we just use the defaults that
     lp_solve provided in hopes that they are reasonable.

     newlp->epsilon = LP_getPrecision();
     newlp->epsb = LP_getPrecision();
     newlp->epsd = LP_getPrecision();
     newlp->epsel = LP_getPrecision();
  */
  newlp->epsb = DEF_EPSB; /* for rounding RHS values to 0 determine 
                             infeasibility basis */
  newlp->epsel = DEF_EPSEL; /* for rounding other values (vectors) to 0 */
  newlp->epsd = DEF_EPSD; /* for rounding reduced costs to zero */
  newlp->epsilon = DEF_EPSILON; /* to determine if a float value is integer */

  /* We have now explicitly told this routine how many non-zero
     entries there are. Note that this number *has* to include the
     objective row coefficients as well. :ARC */
  newlp->non_zeros = non_zeroes;
  newlp->mat_alloc = newlp->non_zeros;

  newlp->mat = (matrec *) XCALLOC( (size_t) newlp->mat_alloc,
							sizeof(*(newlp->mat)) );
  newlp->col_no = (int *) XCALLOC( (size_t) newlp->mat_alloc + 1,
							sizeof(*(newlp->col_no)) );
  newlp->col_end = (int *) XCALLOC( (size_t) columns + 1,
							 sizeof(*(newlp->col_end)) );
  newlp->row_end = (int *) XCALLOC( (size_t) rows + 1,
							 sizeof(*(newlp->row_end)) );
  newlp->row_end_valid = FALSE;
  newlp->orig_rh = (REAL *) XCALLOC( (size_t) rows + 1,
							  sizeof(*(newlp->orig_rh)) );
  newlp->rh = (REAL *) XCALLOC( (size_t) rows + 1,
						  sizeof(*(newlp->rh)) );
  newlp->rhs = (REAL *) XCALLOC( (size_t) rows + 1,
						   sizeof(*(newlp->rhs)) );
  newlp->must_be_int = (short *) XCALLOC( (size_t) sum + 1,
								  sizeof(*(newlp->must_be_int)) );
  for(i = 0; i <= sum; i++)
    newlp->must_be_int[i]=FALSE;
  newlp->orig_upbo = (REAL *) XCALLOC( (size_t) sum + 1,
							    sizeof(*(newlp->orig_upbo)) );
  for(i = 0; i <= sum; i++)
    newlp->orig_upbo[i]=newlp->infinite;
  newlp->upbo = (REAL *) XCALLOC( (size_t) sum + 1,
						    sizeof(*(newlp->upbo)) );
  newlp->orig_lowbo = (REAL *) XCALLOC( (size_t) sum + 1,
								sizeof(*(newlp->orig_lowbo)) );
  newlp->lowbo = (REAL *) XCALLOC( (size_t) sum + 1,
							sizeof(*(newlp->lowbo)) );
  
  /* Not sure why they feel the need to set this to TRUE in lpkit.c.
     Every time something is added to the LP it gets changed to
     false.  I'll start it off FALSE and see what happens. :ARC */
  newlp->basis_valid = FALSE;

  newlp->bas = (int *) XCALLOC( (size_t) rows+1,
						  sizeof(*(newlp->bas)) );
  newlp->basis = (short *) XCALLOC( (size_t) sum + 1,
							 sizeof(*(newlp->basis)) );
  newlp->lower = (short *) XCALLOC( (size_t) sum + 1,
							 sizeof(*(newlp->lower)) );

  for(i = 0; i <= rows; i++)
    {
      newlp->bas[i] = i;
      newlp->basis[i] = TRUE;
    }

  for(i = rows + 1; i <= sum; i++)
    newlp->basis[i] = FALSE;


  for(i = 0 ; i <= sum; i++)
    newlp->lower[i] = TRUE;
 
  /* As with lp->basis_valid, not sure why they feel the need to set
     this to TRUE.  Every time something is added to the LP it gets
     changed to false.  I'll start it off FALSE and see what
     happens. :ARC */
  newlp->eta_valid = FALSE;

  newlp->eta_size = 0;
  newlp->eta_alloc = INITIAL_MAT_SIZE;
  newlp->max_num_inv = DEFNUMINV;

  newlp->nr_lagrange = 0;

  newlp->eta_value = (REAL *) XCALLOC( newlp->eta_alloc,
							    sizeof( *(newlp->eta_value)));
  newlp->eta_row_nr = (int *) XCALLOC( newlp->eta_alloc,
							    sizeof( *(newlp->eta_row_nr)));
  /* +1 reported by Christian Rank */
  newlp->eta_col_end = (int *) XCALLOC( newlp->rows_alloc 
								+ newlp->max_num_inv + 1,
								sizeof( *(newlp->eta_col_end)));

  newlp->bb_rule = FIRST_NI;
  newlp->break_at_int = FALSE;
  newlp->break_value = 0;

  newlp->iter = 0;
  newlp->total_iter = 0;

  newlp->solution = (REAL *) XCALLOC( sum + 1,
							   sizeof( *(newlp->solution)));
  newlp->best_solution = (REAL *) XCALLOC( sum + 1,
								   sizeof( *(newlp->best_solution)));
  newlp->duals = (REAL *) XCALLOC( rows + 1,
							sizeof( *(newlp->duals)));

  newlp->maximise = FALSE;
  newlp->floor_first = TRUE;

  newlp->scaling_used = FALSE;
  newlp->columns_scaled = FALSE;

  newlp->ch_sign = (short *) XCALLOC(rows + 1, sizeof( *(newlp->ch_sign)));

  for(i = 0; i <= rows; i++)
    newlp->ch_sign[i] = FALSE;

  newlp->valid = FALSE; 

  /* I don't use names, so why allocate more space than I
     need. Changed the argument to create_hash_table from HASHSIZE to
     '1'. :ARC */
  /* create two hash tables for names */
  newlp->rowname_hashtab = create_hash_table(1);
  newlp->colname_hashtab = create_hash_table(1);

  return(newlp);
}  /* LP_make_lp */
/**********************************************************************/
void 
LP_writeLpSolveLP( lprec *lp, char *filename ) 
{
  /*
    Opens the file and then uses lp_solve's LP writing function.
  */
  char msg[MAX_MSG_LENGTH];
  FILE *file;

  Assert( lp != NULL, "LP is NULL." );

  if ((file = fopen(filename , "w")) == NULL) {
    sprintf( msg, 
             "The LP file '%s' cannot be opened for writing.",
            filename);
    Abort( msg );
  } /* if couldn't open file */
  
  write_LP( lp, file );
  fclose( file );

}  /* LP_writeLpSolveLP */
/**********************************************************************/
void 
LP_writeLpSolveSolution( lprec *lp, char *filename ) 
{
  char msg[MAX_MSG_LENGTH];
  FILE *file;

  Assert( lp != NULL, "LP is NULL." );

  if ((file = fopen(filename , "w")) == NULL) {
    sprintf
      ( msg, "The LP solution file '%s' cannot be opened for writing.",
        filename );
    Abort( msg );
  } /* if couldn't open file */
  
  write_solution( lp, file );
  fclose( file );

}  /* LP_writeLpSolveSolution */
/**********************************************************************/
lprec *
LP_loadLpSolveLP( LP lp ) 
{
  /* 
     This routine takes the LP instance that has been set up for CPLEX
     and converts it into an LP instance for lp_solve. The idea here is
     that no matter what solver you are using, you set it up the same.
     Then, just before solving the LP, you convert it to the appropriate
     data structure. With CPLEX, no conversion is needed and thus, you
     get the most efficient implementations.  This routine returns a
     pointer the the new struct if the conversion is successful and NULL
     if there was a problem.  The result goes into lp->lp.
     
     Note that there is an indexing difference between CPLEX and lp_solve
     which makes this conversion prone to errors. lp_solve is one-based,
     while CPLEX is zero-based.
     
     This setting of the values makes no assumption about whether the
     matrix or sparse or not.  It copies CPLEX sparse matrix format
     into that of lp_solve's sparse format.  
  */
  int i, col, lpi;
  lprec *lp_solve_lp;
  double obj_sign, row_sign;

  Assert( lp != NULL, "LP is NULL." );

  /* First create the storage and default values for the lp_solve data
     structure. Note that because lp_solve stores the objective row in
     the matrix, the number of non-zero entries in lp_solve is the
     number of non-zero entries in CPLEX plus the number of objective
     coefficeints.  Since CPLEX doesn't store the objective function
     sparsely, this is just the number of columns. */
  lp_solve_lp = LP_make_lp( lp->rows, lp->cols, 
                            lp->matspace + lp->cols  );

  /* lp_solve only does MIN. To get MAX it multiplies the objective
     coefs by '-1', sets ch_sgn[0] = TRUE and lp->maximise = TRUE.
     Because we do not want to mess with such things, we will do the
     conversion ourselves. */
  if ( lp->objsen == MAXIMIZE )
    obj_sign = -1.0;
  else 
    obj_sign = 1.0;

  /* This copies both the non-zero matrices and their corresponding
     row numbers. Unfortunately, the big pain here is that lp_solve
     shoves the objective function coefficients into the coefficient
     matrix as row '0', whereas CPLEX uses a separate array for the
     objective function coefficients.  Since the matrix is
     column-centric, the objective coefficient variables are scattered
     throughout the matrix.  The details of the math are worked out in
     the comments preceeding this section, but the following looping
     structure makes the conversion much easier.

     We do a loop over cols and rows, which will essentially move
     through the matval[] array in sequence.  We will keep the 'lpi'
     index variable and increment it as we go.  The complication here
     is that lp_solve stores the objective coefficients in the matrix
     and CPLX does not. Furthermore CPLEX doesn't use a sparse
     representation for the objective coefficients either. We thus
     bump up ;lpi' once for each matval[] entry and once for each
     objective row coefficient. */
  lpi = 0;
  for( col = 0; col < lp->cols; col++ ) {

    /* Assume that each column has an objective row variable and don't
       forget to adjust it to compensate for MIN or MAX. */
    lp_solve_lp->mat[lpi].value = obj_sign * lp->obj[col];
    lp_solve_lp->mat[lpi].row_nr = 0;
    lpi++;

    /* To loop over the rows for this column, we start at at
       lp->matbeg[col] in 'matval' and go for lp->matcnt[col]
       positions. */
    for( i = lp->matbeg[col]; 
         i < lp->matbeg[col] + lp->matcnt[col]; 
         i++ )  {

      /* When the LP is allocated, it sets things up for all <= rows.
         If we have a >= row, we need to multiply the coef by '-1'.
         The lprec struct has ch_sign[] to aid with this, but we
         prefer to do the conversion ourselves. */
      if ( lp->sense[lp->matind[i]] == 'G' )
        row_sign = -1.0;
      else
        row_sign = 1.0;

    /* Note that lp_solve captures in one array of structs the same
       thing CPLEX uses two arrays for.  No big deal other than
       remembering the extra level of indirection needed for lp_solve.  */
      lp_solve_lp->mat[lpi].value = row_sign * lp->matval[i];
      lp_solve_lp->mat[lpi].row_nr = lp->matind[i] + 1;
      lpi++;

    } /* for i */

    /* CPLEX keeps two 'cols' sized arrays to define the columns
       boundaries of the coefficient matrix entries (matval).  It uses
       indices directly into the matval array for the start of each
       column and also has an array which indicates each columns'
       length (i.e., how many rows have non-zero entries for this
       column.)  lp_solve, on the other hand uses a very weird thing
       about keeping a pointer into the matrix where the column ends.
       The weirdest part is that this index is actually to the first
       element of the *next* column. Yuck. Anyway, this is complicated
       even more by the '1'-based indexing of lp_solve.  This leaves
       me completely unclear as to what value col_end[0] should have
       or even if it matters. I've assumed it is zero, but you never
       know. This is set right after thes loops.

       However, right now 'lpi' points to the first entry in the next
       column, which is exactly what lp_solve needs 'col_end' set to
       for this column.  Also, notice the adjustment of the 'col'
       variable to compensate for the '1'-based indexing of
       lp_solve. */
    lp_solve_lp->col_end[col+1] = lpi;

  }  /* for col */

  /* The weirdness I am not sure of that was mention above. */
  lp_solve_lp->col_end[0] = 0;

  /* The first lp-rows upper bounds are for the constraint rows. The
     last lp->cols are the ones for the variables. There' also the
     off-by-one problem to deal with here. */
  for( i = 0; i < lp->cols; i++) {
    lp_solve_lp->orig_upbo[lp->rows + i + 1] = lp->upbnd[i];
    lp_solve_lp->orig_lowbo[lp->rows + i + 1] = lp->lowbnd[i];
  } /* for i */

  /* The right hand sides just have to account for the off-by-one
     indexing problem. Also, since we are doing the multiplication by
     '-1' for >= rows, make sure we do this for the RHS. */
  for ( i = 0; i < lp->rows; i++ )
    if ( lp->sense[i] == 'G' )
      lp_solve_lp->orig_rh[i+1] = -1.0 * lp->rhs[i];
    else
      lp_solve_lp->orig_rh[i+1] = lp->rhs[i];

  /* The sense of the constraint row is fairly explicit in CPLEX, but
     more subtle in lp_solve. lp_solve really only allows <=
     constraints, so if you want >= you must multiply the constraint
     by -1 if you want to use it.  Judging from the set_const_type()
     routine in lpkit.c, it seems that equality constraints are
     obtained by setting the upper bound on the constraint to zero.
     
     Although the set_const_type() routine will actually do what
     we want it to do, assuming that all the coefficients and RHS
     values have been already set, we prefer to adjust >= row to <= by
     ourselves as was done above.  However, we do still need to set
     the upper bound on the equality rows. */
  for ( i = 0; i < lp->rows; i++ )
    if ( lp->sense[i] == 'E' )
      lp_solve_lp->orig_upbo[i+1] = 0.0;
  
  /* As far as I can tell, the lp_solve fields:

     lp_solve_lp->row_end[]
     lp_solve_lp->col_no[]
     lp_solve_lp->lowbo[]
     lp_solve_lp->upbo[]
     lp_solve_lp->rhs[]

     need to be allocated, but do not need to be set with any
     values. I drew this conclusion from looking at the lpkit.c code
     and saw those routines never actually set these.  Thus my
     speculation is that it is used internally during the solution
     process.  Indeed I did see references to these in the solve.c
     file, so I suspectthis assumption is likely to be right.
  */

  return ( lp_solve_lp );
}  /* LP_loadLpSolveLP */ 
/**********************************************************************/
void 
LP_extractLpSolveSolution( LP lp ) 
{
  /*
    Extracts the solution information from lp_solves data structure into
    the CPLEX-like data structures.
  */
  int i;
  
  Assert( ( lp != NULL ),
          "LP is NULL." );
  
  Assert( ( lp->lp != NULL ),
          "No lp_solve LP found." );

  /* The references to 'lp->lp' are bogus when using CPLEX and will
     cause the compiler to complain about dereferencing incomplete
     types.  To avoid this we just conditionally  compile the body of
     this routine. */
#ifndef HAVE_LIBCPLEX

  /* Note tht lp->lpstat was set in the LP_optimizeLP() routine and
     lp_solve just maintains that. */
  lp->objval = (double) lp->lp->best_solution[0];
  
  /* Since we explicitly multiplied the objective row by -1 for MAX
     problems, we have to remember that the objective value needs to
     be converted back. */
  if ( lp->objsen == MAXIMIZE )
    lp->objval *= -1.0;
  
  /* Solution values for LP variables, if space allocated. */
  if ( lp->x != NULL )
    for(i = 0; i < lp->cols; i++)
      lp->x[i] = (double) lp->lp->best_solution[lp->rows+i+1];
  
  /* We do not use the slack or dual variables so we will not copy
     them over.  However, if it becomes useful, this is how it is
     done:
     
     For Slacks:
     for(i = 0; i < lp->rows; i++) 
     lp->slack[i] =  (double) lp->lp->solution[i+1]; 
     
     Not exactly sure which one is duals: 
     for(i = 0; i < lp->rows; i++) 
     lp->pi = (double) lp->duals[i+1];  
     lp->dj = (double) lp->duals[i+1];  
     
  */

#endif
  
}  /* LP_extractLpSolveSolution */
/**********************************************************************/
void 
LP_unloadLpSolveLP( lprec **lp ) 
{

  Assert( (lp != NULL) && ( *lp != NULL ), "LP is NULL." );

  delete_lp( *lp );

  *lp = NULL;

}  /* LP_unloadLpSolveLP */
/**********************************************************************/

/**********************************************************************/
/**********          LP Interfacing Routines             **************/
/**********************************************************************/

/**********************************************************************/
LP 
LP_newLP( int rows, int cols, int non_zeroes ) 
{
  /*
    Creates a new LP instance, allocating all the memory that is needed
    for an LP of the size given. 
  */ 
  LP lp;

  Assert (( rows > 0 )
          && ( cols > 0 )
          && ( non_zeroes > 0 ),
          " Non-positive number of rows, columns and/or non-zeroes." );
  
  lp = (LP) XMALLOC( sizeof( *lp ));

  lp->name = (char *) XCALLOC( strlen( DEFAULT_LP_NAME )+1, 
                              sizeof(char));
  strcpy( lp->name, DEFAULT_LP_NAME );

  /* Start with no LP loaded. */
  lp->lp = NULL;
  lp->pi = NULL;
  lp->slack = NULL;
  lp->dj = NULL;

  /* Default value to use when considering whether to include a
     coefficient in a sparse representation. Note that we don't really
     want to tie this to the precision that is being used to solve the
     problem, because this value can change the problem being solved.
     Thus this should just be fixed for all time at the minimum
     precision. */
  lp->sparse_epsilon = SMALLEST_PRECISION;

  /* All LPs will be trying to maximize objective function. */
  lp->objsen = MAXIMIZE;
  
  lp->cols = lp->colspace = cols;
  lp->rows = lp->rowspace = rows;
  lp->matspace = non_zeroes;
  
  /* Allocate the memory that comprises the main sparse matrix LP data
     structure. */
  lp->lowbnd = (double *) XMALLOC ( sizeof( double ) * lp->cols );
  lp->upbnd = (double *) XMALLOC ( sizeof( double ) * lp->cols );
  lp->matbeg = (int *) XMALLOC ( sizeof( int ) * lp->cols );
  lp->matcnt = (int *) XMALLOC ( sizeof( int ) * lp->cols );
  lp->matind = (int *) XMALLOC ( sizeof( int ) * lp->matspace );
  lp->matval = (double *) XMALLOC ( sizeof( double ) * lp->matspace );
  lp->rhs = (double *) XMALLOC ( sizeof( double ) * lp->rows );
  lp->sense = (char *) XMALLOC ( sizeof( char ) * lp->rows );
  lp->obj = (double *) XMALLOC ( sizeof( double ) * lp->cols );

  /* When getting the solutions we need an arrary to hold the values
     of the variable.  We allocate this space once and reuse it all
     the time. */
  lp->x = (double *)XMALLOC( lp->cols * sizeof(double));
  
  /* CPLEX allows both a normal optimize and a dual-optimize option.
     This sets the default one, though it is irrelevent for lp_solve
     which has only a single solution method. */
  lp->lp_algorithm = DEFAULT_LP_ALGORITHM;

  /* Set this flag so we know that everything has been allocatd. */
  lp->allocated = 1;
  lp->allocated_space = lp->matspace;

  /* Because this allocation does not know how the non-zeroes appear
     in the formaulation, the burden is up to the user of the LP, when
     setting up the LP to make sure that the arrays:

     lp->matbeg[]
     lp->matcnt[]
     lp->matind[]

     are properly set.  */

  return( lp );

}  /* LP_newLP */
/**********************************************************************/
void 
LP_freeLP( LP lp ) 
{
  /*
    Completely frees up all memory associated with an LP.
  */  
  if ( lp == NULL )
    return;
  
  // Matthijs: lp->name was allocated but never freed, fixed it
  XFREE( lp->name );

  XFREE( lp->obj );
  XFREE( lp->x );
  XFREE( lp->lowbnd );
  XFREE( lp->upbnd );
  XFREE( lp->matbeg );
  XFREE( lp->matcnt );
  XFREE( lp->matind );
  XFREE( lp->matval );
  XFREE( lp->rhs );
  XFREE( lp->sense );

  XFREE( lp );
}  /* LP_freeLP */
/**********************************************************************/
void 
LP_writeSolution( LP lp, char *filename ) 
{
  /*
    Writes the current solution to a file.
  */
  
#ifdef HAVE_LIBCPLEX
  txtsolwrite( lp->lp, filename );
#else
  LP_writeLpSolveSolution( lp->lp, filename );
#endif

} /* LP_writeSolution */
/**********************************************************************/
void 
LP_writeLP( LP lp, char *filename ) 
{
  /*
    Writes the current LP to a file.
  */
  char msg[MAX_MSG_LENGTH];

#ifdef HAVE_LIBCPLEX
   lpwrite( lp->lp, filename );
#else
   LP_writeLpSolveLP( lp->lp, filename );
#endif

} /* LP_writeSolution */
/**********************************************************************/
void 
LP_loadLP( LP lp ) 
{
  /*
    Sets up the LP in the LP solver in preparation for optimizing.
  */
  
  Assert ( lp != NULL, "No lp exist." );
  Assert ( lp->lp == NULL, "An LP already appears loaded." );
    
#ifdef HAVE_LIBCPLEX
  /* CPLEX takes the user defined arrays and loads the information
     into an internal format in preparation for solving. */
   lp->lp = loadlp (lp->name, lp->cols, lp->rows, 
                    lp->objsen, lp->obj, lp->rhs, lp->sense, 
                    lp->matbeg, lp->matcnt, lp->matind, lp->matval,
                    lp->lowbnd, lp->upbnd, NULL,
                    lp->colspace, lp->rowspace, lp->matspace);
#else
   /* If we are using lp_solve, then this is the place where we need
      to translate from the CPLEX-style LP that is currently
      formulated in the 'lp' data structure into that of lp_solve. */
   lp->lp = LP_loadLpSolveLP( lp );
#endif

   if ( lp->lp == NULL ) 
#ifdef HAVE_LIBCPLEX
     Abort( "Problem loading LP. CPLEX licensing problem?" );
#else
     Abort( "Problem loading LP. lp_solve problem?" );
#endif

}  /* LP_loadLP */
/**********************************************************************/
int 
LP_optimizeLP( LP lp ) 
{
  /*
    Calls the appropriate routine in the LP solver to do the
    optimization of the LP.  Only solves the LP, does not give any of
    the results. Note that CPLEX has a choice of algorithms, where
    lp_solve will use the same optimization routine regardless of the
    algorithm. 
  */
  int status;

  switch( lp->lp_algorithm ) {
    
  case dual_simplex:
#ifdef HAVE_LIBCPLEX
    status = dualopt( lp->lp );
#else
    status = solve( lp->lp );
#endif
    break;
    
  case primal_simplex:
  default:
#ifdef HAVE_LIBCPLEX
    status = optimize( lp->lp );
#else
    status = solve( lp->lp );
#endif
    break;
  }  /* switch lp->lp_algorithm */

  /* We'll write the status into the LP structure.  Note that CPLEX
     will use a different status (hence, overwrite this) with its
     routine to get the solution. lp_solve only has the notion of
     returning a code on the solving, so we write it here and it will
     stay set. */
  lp->lpstat = status;
  return ( status );

}  /* LP_optimizeLP */
/**********************************************************************/
int 
LP_getSolution( LP lp ) 
{
  /*
    After the LP_solveLP routine, we will then actually access the
    solution using this routine. 
  */

#ifdef HAVE_LIBCPLEX
  /* CPLEX will fill in everything about the solution that we need. */

  /* Note that CPLEX will overwrite the lp->lpstats written when the
     LP was solved.  It has this weird notion of given a return value
     on the optimization and then when  you actually get the
     solution. */
   return ( solution (lp->lp, &(lp->lpstat), &(lp->objval), 
                      lp->x, lp->pi, lp->slack, lp->dj ));
   
   /* Alternative way to get info from CPLEX */
   /*   lp->lpstat = getstat( lp->lp ); */
   
#else
   /* With lp_solve, we have to extract the solution information. */
   LP_extractLpSolveSolution( lp );

   return ( lp->lpstat );
#endif

}  /* LP_getSolution */
/**********************************************************************/
void 
LP_unloadLP( LP lp ) 
{
  /*
    Tell the LP solver that we are done with this LP.
  */

#ifdef HAVE_LIBCPLEX
   unloadprob( &(lp->lp) );
#else
   LP_unloadLpSolveLP( &(lp->lp) );
#endif

}  /* LP_unloadLP */
/**********************************************************************/
int 
LP_solveLP( LP lp, SolutionStats stat ) 
{
  /*
    When the LP has been set up, this is the routine to call.  It deals
    with all the LP solver ugliness and gives one of three returns coded
    depending upon the LP solution status:
    
    LP_OPTIMAL
    LP_INFEASIBLE
    LP_UNBOUNDED
  */
  char msg[MAX_MSG_LENGTH];
  int i, status;
  int return_status;

   /* If we want to accumulate statistical information about the LPs,
      then call the routine to do this. */
   if ( gVerbose[V_LP] == TRUE ) 
     recordLpStats( stat, lp->cols, lp->rows );

   /* Set up the LP in the actual LP package being used. */
   LP_loadLP( lp );

  /* The references to 'lp->lp' are bogus when using CPLEX and will
     cause the compiler to complain about dereferencing incomplete
     types.  To avoid this we just conditionally  compile the body of
     this routine. */
#ifndef HAVE_LIBCPLEX
   if ( gVerbose[V_LP_INTERFACE] )
     lp->lp->verbose = TRUE;
#endif

   /* Afer the LP is loaded, this then does the actual optimization or
      solving of the LP.  */
   status = LP_optimizeLP( lp );

   /* Optimizing does not give you the actual solution.  It simply
      tries to optimize as far as possible.  We then use this to get
      the status of the optimization as well as the solution, if
      available. */
   status = LP_getSolution( lp );

   /* Now we decide the outcome by looking at the results. */
   switch( lp->lpstat ) {

#ifdef HAVE_LIBCPLEX
   case CPX_OPTIMAL:
   case CPX_OPTIMAL_INFEAS:
#else
   case OPTIMAL:
#endif
     return_status = LP_OPTIMAL;
     break;
     
#ifdef HAVE_LIBCPLEX
   case 0:  /* no solution */
   case CPX_INFEASIBLE:  /* infeasible */
   case CPX_ABORT_FEAS:  /* infeasible in phase II */
#else
   case INFEASIBLE:
#endif
     return_status = LP_INFEASIBLE;
     break;

     /* I get this weird result when running linear support and the
        sz_purge True option.  I get this for LPs which definitely
        have solutions.  For now, I will simply bail out. I believe
        it is some weird interaction between the vertex enumeration
        stuff and this LP business. I don't think that this CPLEX
        result value has a corresponding lp_solve case. */
#ifdef HAVE_LIBCPLEX
   case CPX_ABORT_INFEAS:  /* infeasible in phase I */ 
     LP_writeLP( lp, ABORT_SAVE_FILENAME );
     sprintf ( msg, "LP status %d. (see file '%s')", 
               status, ABORT_SAVE_FILENAME );
     Warning( msg );
     return_status = LP_INFEASIBLE;
     break;
#else
     /* Not entirely sure of the semantics of all these lp_solve
        status codes, but I will assume they mean some form of no
        solution. Not sure how they would map to the CPLEX result
        values either. */
   case MILP_FAIL:
   case FAILURE:
   case NO_FEAS_FOUND:
   case BREAK_BB:
     return_status = LP_INFEASIBLE;
     break;
#endif
     

#ifdef HAVE_LIBCPLEX
   case CPX_UNBOUNDED:
     if( lp->lp_algorithm == dual_simplex )
       /* unbounded in the dual means the original problem is infeasible */
       return_status = LP_INFEASIBLE;
     else
       return_status = LP_UNBOUNDED;
     break;
#else
   case UNBOUNDED:

     /* With lp_solve, the instability sometimes leads to unbounded
        LPs.  We catch this here and simply convert it to be an
        infeasible LP. */
     if ( gShowInstabilityMessages )
       Warning( "lp_solve reports an unbounded LP. Instability?." );
     
     gLpSolveInstabilityCount++;
     return_status = LP_INFEASIBLE;
     break;
#endif

   default:
     LP_writeLP( lp, ABORT_SAVE_FILENAME );
     sprintf ( msg, "LP status %d. (see file '%s')", 
               status, ABORT_SAVE_FILENAME );
     Warning( msg );
     return_status = LP_INFEASIBLE;
     break;

   } /* switch lp->lp_stat */
   
   /* Tell the LP package we are done with this particular LP so it
      can clean up after itself. We do this after checking the LP
      solution status in case we want to print information using the
      lp->lp structure. */
   LP_unloadLP( lp );

   return ( return_status );

}  /* LP_solveLP */
/**********************************************************************/


/**********************************************************************/
/******   Special low-level LP manipulation for vertex-enum.[ch] ******/
/**********************************************************************/

/**********************************************************************/
int 
LP_loadbase( LP lp, int *cstat, int *rstat  ) 
{

#ifdef HAVE_LIBCPLEX
  return ( loadbase( lp->lp, cstat, rstat ));
#else
  Abort( "loadbase() for lp_solve is not implemented." );
#endif

}  /* LP_loadbase */
/**********************************************************************/
int 
LP_getgrad( LP lp, int j, int *ix, double *y  ) 
{

#ifdef HAVE_LIBCPLEX
  return( getgrad( lp->lp, j, ix, y ));
#else
  Abort( "getgrad() for lp_solve is not implemented." );
#endif

}  /* LP_getgrad */
/**********************************************************************/
int 
LP_getx( LP lp, double *x, int begin, int end ) 
{

#ifdef HAVE_LIBCPLEX
  return( getx( lp->lp, x, begin, end ));
#else
  Abort( "getx() for lp_solve is not implemented." );
#endif

}  /* LP_getx */
/**********************************************************************/
int 
LP_getslack( LP lp, double *slack, int begin, int end  ) 
{

#ifdef HAVE_LIBCPLEX
  return( getslack( lp->lp, slack, begin, end ));
#else
  Abort( "getslack() for lp_solve is not implemented." );
#endif

}  /* LP_ */
/**********************************************************************/
int 
LP_getdj( LP lp,double *dj, int begin, int end ) 
{

#ifdef HAVE_LIBCPLEX
  return( getdj( lp->lp, dj, begin, end ));
#else
  Abort( "getdj() for lp_solve is not implemented." );
#endif

}  /* LP_getdj */
/**********************************************************************/
int 
LP_getpi( LP lp, double *pi, int begin, int end ) 
{

#ifdef HAVE_LIBCPLEX
  return ( getpi( lp->lp, pi, begin, end ));
#else
  Abort( "getpi() for lp_solve is not implemented." );
#endif

}  /* LP_getpi */
/**********************************************************************/
int 
LP_getbase( LP lp, int *cstat, int *rstat ) 
{

#ifdef HAVE_LIBCPLEX
  return( getbase( lp->lp, cstat, rstat ));
#else
  Abort( "getbase() for lp_solve is not implemented." );
#endif

}  /* LP_ */
/**********************************************************************/
int 
LP_setlpcallbackfunc(  int (*pcallback)(CPXLPptr lpinfo, 
					int wherefrom )  ) 
{

#ifdef HAVE_LIBCPLEX
  return( setlpcallbackfunc( pcallback ));
#else
  Abort( "setlpcallbackfunc() for lp_solve is not implemented." );
#endif

}  /* LP_setlpcallbackfunc */
/**********************************************************************/
int 
LP_getobjval( LP lp, double *pobjval ) 
{

#ifdef HAVE_LIBCPLEX
  return( getobjval( lp->lp, pobjval ));
#else
  Abort( "getobjval() for lp_solve is not implemented." );
#endif

}  /* LP_getobjval */
/**********************************************************************/
int 
LP_binvarow( LP lp, int row, double *coefs ) 
{

#ifdef HAVE_LIBCPLEX
  return( binvarow( lp->lp, row, coefs ));
#else
  Abort( "binvarow() for lp_solve is not implemented." );
#endif

}  /* LP_binvarow */
/**********************************************************************/
int 
LP_binvrow( LP lp, int row, double *coefs  ) 
{

#ifdef HAVE_LIBCPLEX
  return( binvrow( lp->lp, row, coefs ));
#else
  Abort( "binvrow() for lp_solve is not implemented." );
#endif

}  /* LP_binvrow */
/**********************************************************************/

