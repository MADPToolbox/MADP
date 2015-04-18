
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    global.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: global.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/global.h,v $
 *    $Revision: 1.5 $
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
 *   Header file for all globally defined items.
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>

/**********************************************************************/
/******************   Memory Allocation Stuff  ************************/
/**********************************************************************/

#ifdef DMALLOC

#include "dmalloc.h"

#define XCALLOC(num, size) calloc( (num), (size) )
#define XMALLOC(size) malloc( size )
#define XREALLOC(p, size) realloc( (p), (size) )
#define XFREE(stale) free(stale)

#else

#include "xalloc.h"

#endif

/*******************************************************************/
/**************      USEFUL MNENOMIC CONSTANTS      ****************/
/*******************************************************************/

/* Useful mnemonics. */
#ifndef FALSE
#define FALSE                        0
#endif
#ifndef TRUE
#define TRUE                         1
#endif

#define NULL_CHAR                    '\0'

/* Use this mnemonic for integer values which need to be >= 0 (i.e.,
   counts, obs, action, state numbers.  */
#define UNINITIALIZED                -1

/* When using strings as temporary space for holding messages to be
   printed, define the string to be of this length. Shouldn't need it
   larger than this, but if so, feel free to increase this. */
#define MAX_MSG_LENGTH               80

/* For places where we statically allocate filename character arrays,
   use this for the length. */
#define MAX_FILENAME_LENGTH          100

/*******************************************************************/
/**************              MACROS                 ****************/
/*******************************************************************/

#ifndef Max
#define Max(x,y) ( (x) < (y) ? (y) : (x))
#endif
#ifndef Min
#define Min(x,y) ( (x) < (y) ? (x) : (y))
#endif

/* Comparisons using a tolerance. */
#ifndef Equal
#define Equal(x,y,e) ((fabs((x)-(y)) < (e)) ? 1 : 0)
#endif


#ifndef LessThan
#define LessThan(x,y,e) (( ((x) + (e)) <= (y) ) ? 1 : 0)
#endif

#ifndef GreaterThan
#define GreaterThan(x,y,e) (( ((y) + (e)) <= (x) ) ? 1 : 0)
#endif

/* This checks to see if the expression is false, and if so prints out
   the message given and exits the program. Note that this uses the
   __PRETTY_FUNCTION__ variable which I believe is a GNU-specific
   thing.  */
#ifndef NON_GNU
#define Assert(EXPR,MSG) if(!(EXPR)) { \
  fprintf(gStdErrFile, "\n** ABORT ** File: %s function: %s, Line: %d\n\t", \
          __FILE__, __PRETTY_FUNCTION__, __LINE__ ); \
  fprintf(gStdErrFile, MSG ); \
  fprintf(gStdErrFile, "\n" ); \
  exit( -1 ); }
#else
#define Assert(EXPR,MSG) if(!(EXPR)) { \
  fprintf(gStdErrFile, "\n** ABORT ** File: %s function: %s, Line: %d\n\t", \
          __FILE__, "<Unknown>", __LINE__ ); \
  fprintf(gStdErrFile, MSG ); \
  fprintf(gStdErrFile, "\n" ); \
  exit( -1 ); }
#endif

/* This is like ASSERT, except there is no expression and it aborts
   with a message. */
#define Abort(MSG) { \
  fprintf(gStdErrFile, "\n** PROGRAM ABORTED **\n\t" ); \
  fprintf(gStdErrFile, MSG ); \
  fprintf(gStdErrFile, "\n" ); \
  exit( -1 ); }

/* This is like ASSERT, except there is no expression and it aborts
   with a message. */
#define Warning(MSG) { \
  fprintf(gStdErrFile, "\n** Warning **\n\t" ); \
  fprintf(gStdErrFile, MSG ); \
  fprintf(gStdErrFile, "\n" ); }

/*******************************************************************/
/**************             TYPEDEFS                ****************/
/*******************************************************************/

#define BOOLEAN_STRINGS           { \
                                    "false", \
                                    "true" \
                                 } 



/*******************************************************************/
/**************        OVERALL PROGRAM CONSTANTS    ****************/
/*******************************************************************/

/* Do not allow any precision parameters to be smaller than
   this. Change this if you think the code and machine can handle more
   precision or if it cannot handle this precision. */
#define SMALLEST_PRECISION                 1e-12

/* Sometimes it is handy to have a value that makes no sense as a
   time.  This negative number serves that purpose. */
#define INVALID_TIME                         -1.0

/* Sometimes it is handy to have a value that makes no sense as a
   precision factor.  This negative number serves that purpose. */
#define INVALID_PRECISION                    -1.0

/* When displaying floats, how many decimal points do you want 
   to show? */
#define NUM_DECIMAL_DISPLAY   6

/* Main command line arguments independent of solution stuff. */
#define CMD_ARG_HELP_SHORT           "-h"
#define CMD_ARG_HELP_LONG            "-help"
#define CMD_ARG_VERSION_SHORT        "-v"
#define CMD_ARG_VERSION_LONG         "-version"
#define CMD_ARG_VERBOSE              "-verbose"

/*******************************************************************/
/**************         DEFAULT VALUES              ****************/
/*******************************************************************/

/*******************************************************************/
/**************     VERBOSE CMD LINE OPTIONS        ****************/
/*******************************************************************/

/* Each module in the program will be assigned a unique number.  This
   is used in the verboseness level.  We have an array of TRUE/FALSE
   values indicating whether the module should operate in verbose
   mode.  In addition to each module, some other special purpose
   functionality can get its own verboseness level. */

/* There are three things that need to change if you want to add or
   remove a verboseness option. First the set of constants with the V_
   prefix needs to be modified, and the final result must still be a
   sequential list of integeres starting at zero.  Next, you must
   change the NUM_VERBOSE_OPTIONS to reflect the new number. Finally,
   you must add a string for the command line.  The array of strings
   *must* match up to the order in the sequential numbering. */

#define NUM_VERBOSE_MODES                   26

#define V_CONTEXT                           0
#define V_LP                                1
#define V_GLOBAL                            2
#define V_TIMING                            3
#define V_STATS                             4
#define V_CMD_LINE                          5
#define V_POMDP_SOLVE                       6
#define V_ALPHA                             7
#define V_PROJECTION                        8
#define V_CROSS_SUM                         9
#define V_AGENDA                            10
#define V_ENUMERATE                         11
#define V_TWO_PASS                          12
#define V_LIN_SUPPORT                       13
#define V_WITNESS                           14
#define V_INC_PRUNE                         15
#define V_LP_INTERFACE                      16
#define V_VERTEX_ENUM                       17
#define V_MDP                               18
#define V_POMDP                             19
#define V_PARAM                             20
#define V_PARSIMONIOUS                      21
#define V_REGION                            22
#define V_APPROX_MCGS                       23
#define V_ZLZ_SPEEDUP                       24
#define V_FINITE_GRID                       25

#define VERBOSE_MODE_STRINGS  { \
                                 "context", \
                                 "lp", \
                                 "global", \
                                 "timing", \
                                 "stats", \
                                 "cmdline", \
                                 "main", \
                                 "alpha", \
                                 "proj", \
                                 "crosssum", \
                                 "agenda", \
                                 "enum", \
                                 "twopass", \
                                 "linsup", \
                                 "witness", \
                                 "incprune", \
                                 "lpinterface", \
                                 "vertexenum", \
                                 "mdp", \
                                 "pomdp", \
                                 "param", \
                                 "parsimonious", \
                                 "region", \
                                 "approx_mcgs", \
                                 "zlz_speedup", \
                                 "finite_grid" \
}

/*******************************************************************/
/**************       EXTERNAL VARIABLES            ****************/
/*******************************************************************/

extern char *boolean_str[];
extern char *verbose_mode_str[];

extern int gVerbose[];
extern char gExecutableName[];
extern FILE *gStdErrFile;

extern double *gTempValue;
extern double *gTempBelief;
extern double *gTempAlpha;

/*******************************************************************/
/**************       EXTERNAL FUNCTIONS            ****************/
/*******************************************************************/

/* Sets up and allocates variables that are used globally across
  modules in the program. Currently just allocates a bunch of scratch
  memory areas.  */
extern void initGlobal(  );

/* Cleans up after problem is solved to free any resources and reset
  anything that the initGlobal() routine did.  */
extern void cleanUpGlobal(  );
  
/* Just a wrapper to the UN*X getpid() function to isolate it in case
   this gets ported to another platform.  Note that for POSIX, the
   'pid_t' type returned by getpid() is an 'int'.  */
extern int getPid(  );

/* Just a wrapper to the UN*X unlink() function to isolate it in case
   this gets ported to another platform.  */
extern void removeFile( char *filename );

#endif
