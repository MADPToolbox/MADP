
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    lp-interface.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: lp-interface.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/lp-interface.h,v $
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
#ifndef LP_INTERFACE_H
#define LP_INTERFACE_H

/* These are for lp_solve, but we will include them regardless of
   whether we are using CPLEX or not.  This will allow the lp_solve
   specific things to compile and reduce the number of conditionally
   compiled code fragments, which just add confusion. */

#include "lp-solve-old/lpkit.h"
#include "lp-solve-old/lpglob.h"
//
//#include <lpsolve/lp_lib.h>


#include "params.h"


//fao 201403005: we are now using 'HAVE_CPLEX', however, the current CPLEX version does 
//not seem to be supported anymore by this code. Therefore, I am renaming the occurences 
//of HAVE_CPLEX in this file to HAVE_CPLEX_OLD.
#ifdef HAVE_CPLEX_OLD
#include "cpxdefs.inc"
#endif

#include "stats.h"

/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/* Constant to use for the objective function sense of the LP. CPLEX
   defines these values and since they are handy, we just define them
   here if we do not have CPLEX. */
#define MAXIMIZE                   -1
#define MINIMIZE                   +1

/* We use these as the universal return values for LPs and will map the LP solver's return status to these three. */
#define LP_OPTIMAL                  0
#define LP_INFEASIBLE               1
#define LP_UNBOUNDED                2

/* CPLEX defines some constants to be passed to library functions.
   Because we need the same concepts, we will define the same
   constants when we do not have CPLEX just to keep the code
   simple. */ 
#ifndef HAVE_CPLEX_OLD

/* Used for variable bounds in the LP. Same as DEF_INFINITE in lpkit.c */
#define INFBOUND                    1e24

/* These are the return status mnemonic values for solving an
   LP. lp_solve uses different constants and values as are defined in
   lpkit.h. Ideally we want to have both solving routines return the
   same stuff, so we can consolidate the functionality. */
#define CPX_NO_SOLUTION             0
#define CPX_OPTIMAL                 1
#define CPX_OPTIMAL_INFEAS          2
#define CPX_INFEASIBLE              3
#define CPX_ABORT_FEAS              4 
#define CPX_ABORT_INFEAS            5 
#define CPX_UNBOUNDED               6 

#endif

#define ABORT_SAVE_FILENAME          "abort.lp"

/**********************************************************************/
/********************   DEFAULT VALUES       **************************/
/**********************************************************************/

/* Set this to the enumeration type from lp_algorithm_type for the
   default value. */
#define DEFAULT_LP_ALGORITHM        dual_simplex

/* This sets the default value for the epsilon value for comparing the
   objective value to zero. */
#define DEFAULT_LP_SOLN_EPSILON       1.0e-9

#define DEFAULT_LP_ZERO_EPSILON         1e-9

/* Just give the LP a default name */
#define DEFAULT_LP_NAME                "PomdpSolveLP"

/* The lp_solve solver prints a couple lines when an LP is
   demonstrating precision problems.  We can control this by setting
   this constant.  We have modified the lp_solve code to count the
   instability messages so we at least know when there is a problem.
   */
#define SHOW_INSTABILITY_MESSAGES     FALSE

/**********************************************************************/
/********************    TYPEDEFS            **************************/
/**********************************************************************/

/* If we do not have CPLEX, just define the CPLEX type to a void
   pointer so we do not get compilation problems. */
#ifndef HAVE_CPLEX_OLD
typedef lprec *CPXLPptr;
#endif

/* There are various ways the LP could be solved in CPLEX.  This
   enumeration provides mnemonics for these methods which can be set
   for the various LPs. */
typedef enum { primal_simplex, dual_simplex} lp_algorithm_type;

typedef struct LpStruct *LP;
struct LpStruct {

  /* Epsilon value to use when comparing coefficient values to zero to
     see if they should be included in a sparse matrix
     formualtion. Still not clear to me if this should be the same as
     soln_epsilon or not. */
  double sparse_epsilon;

  char     *name;                 /* name for lp */
  int      cols;                  /* number of variables */
  int      rows;                  /* number of constraints */
  int      objsen;

  /* Note that the rows and cols only specify what portion of the
     allocated mmeory is being used. Thus we can allocate more space
     than we need for a given number of rows and columns, thus we need
     these variables to define how much space was actually allocated. */
  int      colspace;              /* number of cols */
  int      rowspace;              /* number of rows */
  int      matspace;              /* number of non-zero coefs */

  /* These are needed by CPLEX, but will also be handy to use for
     lp_solve (using it indirectly to set the real lp_solve data
     structures up. */
  double   *obj;                  /* size = num cols */
  double   *rhs;                  /* size = num rows */
  char     *sense;                /* size = num rows */

  int      *matbeg;               /* size = num cols */
  int      *matcnt;               /* size = num cols */
  int      *matind;               /* size = num non-zero coefs */
  double   *matval;               /* size = num non-zero coefs */
  double   *lowbnd;               /* size = num variables (cols) */
  double   *upbnd;                /* size = num variables (cols) */
  
#ifdef HAVE_CPLEX_OLD
  /* Things only needed for CPLEX. */
  CPXLPptr lp;
#else
  /* Things only needed for lp_solve. */
  lprec *lp;  
#endif
  
  int allocated;
  int allocated_space;
  
  lp_algorithm_type lp_algorithm;

  /* All this stuff is used for holding the solution information. */ 
  int      lpstat;                /* Holds the result of the
                                     optimization of the LP. */

  double   objval;                /* The objective value after
                                     optimization. */

  double   *pi;
  double   *slack;
  double   *dj;
  double   *x;             /* Will hold the variable solution
                                     values. */

};

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/* Sets the precision that should be used when solving LPs.  */
extern void LP_setPrecision( double epsilon );

/* Initialize and allocate anything we might need for LPs.  */
extern void initLpInterface( PomdpSolveParams param );

/* Free up any temporary memory that was allocated.  */
extern void cleanUpLpInterface(  );

/**********************************************************************/
/**********          LP Interfacing Routines             **************/
/**********************************************************************/

/* Creates a new LP instance, allocating all the memory that is needed
  for an LP of the size given.  */
extern LP LP_newLP( int rows, int cols, int non_zeroes );

/* Completely frees up all memory associated with an LP.  */
extern void LP_freeLP( LP lp );

/* Writes the current solution to a file.  */
extern void LP_writeSolution( LP lp, char *filename );

/* Writes the current LP to a file.  */
extern void LP_writeLP( LP lp, char *filename );

/* Sets up the LP in the LP solver in preparation for optimizing.  */
extern void LP_loadLP( LP lp );

/* Calls the appropriate routine in the LP solver to do thr
  optimization of the LP.  Only solves the LP, does not give any of
  the results. Note that CPLEX has a choice of algorithms, where
  lp_solve will use the same optimization routine regardless of the
  algorithm.  */
extern int LP_optimizeLP( LP lp );

/* After the LP_solveLP routine, we will then actually access the
  solution using this routine.  */
extern int LP_getSolution( LP lp );

/* Tell the LP solver that we are done with this LP.  */
extern void LP_unloadLP( LP lp );

/* When the LP has been set up, this is the routine to call.  It deals
    with all the LP solver ugliness and gives one of three returns
    coded depending upon the LP solution status: LP_OPTIMAL,
    LP_INFEASIBLE, LP_UNBOUNDED */
extern int LP_solveLP( LP lp, SolutionStats stat );

/**********************************************************************/
/******   Special low-level LP manipulation for vertex-enum.[ch] ******/
/**********************************************************************/

extern int LP_loadbase( LP lp, int *cstat, int *rstat  );
extern int LP_getgrad( LP lp, int j, int *ix, double *y  );
extern int LP_getx( LP lp, double *x, int begin, int end );
extern int LP_getslack( LP lp, double *slack, int begin, int end  );
extern int LP_getdj( LP lp,double *dj, int begin, int end );
extern int LP_getpi( LP lp, double *pi, int begin, int end );
extern int LP_getbase( LP lp, int *cstat, int *rstat );
extern int LP_setlpcallbackfunc(  int (*pcallback)(CPXLPptr lpinfo, 
                                                    int wherefrom )  );
extern int LP_getobjval( LP lp, double *pobjval );
extern int LP_binvarow( LP lp, int row, double *coefs );
extern int LP_binvrow( LP lp, int row, double *coefs  );
                            

#endif
