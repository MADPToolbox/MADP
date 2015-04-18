
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    stats.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: stats.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/stats.h,v $
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
#ifndef STATS_H
#define STATS_H

/**********************************************************************/
/********************       TYPEDEFS        ***************************/
/**********************************************************************/

/* For a global program context.  Currently only used to help keep
   counts on the number and sizes of LPs and for timing breakdowns.
   You cannot nest contexts at this time. */
#define MAX_NUM_CONTEXT_TYPES         7
typedef enum { Context_None, 
               Context_Projection_build, 
               Context_Projection_purge, 
               Context_Q_a_build, 
               Context_Q_a_merge,
               Context_Zlz_Speedup,
               Context_Stop_Criteria } ContextType;
#define CONTEXT_TYPE_STRINGS        { \
                                     "No-context", \
                                     "Proj-build", \
                                     "Proj-purge", \
                                     "Qa-build", \
                                     "Qa-merge", \
                                     "Zlz-Speedup", \
                                     "Stop-criteria" \
                                     } 
#define CONTEXT_TYPE_STRINGS_BRIEF    { \
                                       "No-context", \
                                       "PjB", \
                                       "PjP", \
                                       "QaB", \
                                       "QaM", \
                                       "Zlz", \
                                       "Stop" \
                                       } 

/* A structure to store statistics for each epoch. */
typedef struct EpochStatStruct *EpochStats;
struct EpochStatStruct { 

  /* Which epoch number these stats are for. */
  int epoch_num;

  /* Timing for each of the contexts during this epoch. */
  double epoch_time[MAX_NUM_CONTEXT_TYPES];

  /* Total times broken down into system and user time. */
  double user_time;
  double system_time;

  /* Number of vectors in the solution for this epoch. */
  int solution_size;

  /* The precision defined at the start of the epoch. */
  double set_epsilon;

  /* (For the epsilon_prune option only) The computed maximal epsilon
     for the resulting set for this epoch. */
  double max_epsilon;

  /* When using the weak or bellman stopping conditions, this will
     contain the actual computed difference between this set and the
     previous set. */
  double actual_stop_delta;

  EpochStats next;
};

/* We just want to encapsulate all the parameters used in the
   pomdp-solve program into a single structure for convenience. These
   do not include any parameters that are specific to a particular
   algorithm. */ 
typedef struct SolutionStatsStruct *SolutionStats;
struct SolutionStatsStruct {

  /* Where to report statistics. */
  FILE *report_file;

  /* Whether to conclude with a summary of stats at end of VI. */
  int stat_summary;

  /* Variables to help keep track of timing information for entire
     solution process. */
  double start_time_user;
  double start_time_system;

  double tot_time_user;
  double tot_time_system;

  int cur_epoch;
  int tot_epochs;

  /* Variables to help keep track of timing information for a single
     epoch of solution process. */
  double epoch_start_time_user;
  double epoch_start_time_system;

  double epoch_tot_time_user;
  double epoch_tot_time_system;

  /* For a global program context.  Currently only used to help keep
     counts on the number and sizes of LPs for the various contexts. */
  ContextType cur_context;
  
  /* We will keep timing stats for all the different contexts
     available. */
  double cur_context_start_time;
  
  double tot_time[MAX_NUM_CONTEXT_TYPES];
  
  /* We will keep LP stats for all the different contexts available. */
  int lp_count[MAX_NUM_CONTEXT_TYPES];
  int constraint_count[MAX_NUM_CONTEXT_TYPES];

  double epoch_start_lps[MAX_NUM_CONTEXT_TYPES];
  double epoch_start_constraints[MAX_NUM_CONTEXT_TYPES];

  /* We will store statistics for each iteration.  For now we store
     the epsilon used, the maximum epsilon (for epsilon prune
     version, the time and the number of vectors in each epoch. A
     linked list of these elements. */
  EpochStats epoch_stats;

};

/**********************************************************************/
/********************     DEFAULT VALUES    ***************************/
/**********************************************************************/

#define DEFAULT_CONTEXT                   Context_None

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/* Will return the EpochStats structure for the given epoch number if
   it exists in the stat->epoch_stats linked list, or else create a new
   structure and add it to the end of the list and return the pointer
   to it.  */
extern EpochStats getEpochStats( SolutionStats stat,
                                 int epoch_num );

/* Stores the solution size for a particular epoch.  */
extern void recordEpochMaxEpsilon( int epoch_num,
                                   double max_epsilon,
                                   SolutionStats stat );

/* Stores the actual computed stopping criteria difference between the
   value functions according to either the 'weak' or 'bellman'
   stopping criteria.  */
extern void recordEpochCurStopDelta( int epoch_num,
                                     double cur_stop_delta,
                                     SolutionStats stat );

/* Allocates memory and sets the default values for all variables
   related to tracking POMDP solving statistics.  */
extern SolutionStats newSolutionStats(FILE *report_file, 
                                      int stat_summary );

/* Called just prior to solving an LP, this routine records the size
  of the LP for the given context that the program is runing in.  */
extern void recordLpStats(  SolutionStats stat,
                            int num_variables, 
                            int num_constraints );

/* Retrieves the stores LP stats for all the different program
  contexts.  Puts them in two arrays corresponding to the total number
  of LPs and the total number of constraints.  */
extern void getLpStats( SolutionStats stat, 
                        int *tot_lps, 
                        int *tot_constraints );

/* Prints out the current LP stats that have accumulated.  */
extern void reportLpStats( SolutionStats stat );

/* As the program executes, it makes its way through various
  conceptual portions of the code.  We have chosen to break up the
  code's execution into a few distinct contexts.  This routine is
  called to indicate that a new context has started.  You cannot do
  nesting of contexts at this time.  */
extern void startContext( SolutionStats stat, 
                        ContextType context );

/* Ends the program execution context indicated.  */
extern void endContext( SolutionStats stat, 
                        ContextType context );

/* Shows both the execution time and LP statistics broken down by the
  various program contexts.  */
extern void reportContextDetails( SolutionStats stat );

/* Called just prior to doing the value iteration new value function
  computation.  Sets up the stats in preparation for this iteration.  */
extern void epochStartStats( SolutionStats stat );

/* Called after an iteration of value iteration.  Will update the
  statistics, report for this epoch.  Need the solution size in the
  reporting.  */
extern void epochEndStats( SolutionStats stat, 
                           int solution_size,
                           double cur_error );
  
#endif
