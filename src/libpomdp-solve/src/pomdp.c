
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    pomdp.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    February, 1999
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: pomdp.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/pomdp.c,v $
 *    $Revision: 1.3 $
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
  This file contains code for reading in a pomdp file and setting
  the global variables for the problem for use in all the other files.
  It also has routines for operations on belief states.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "mdp/mdp.h"

#include "global.h"
#include "pomdp.h"

int **gObservationPossible;
int *gNumPossibleObservations;

/* Some variations will require there to be no negative immediate
   rewards. If this is the case, then this flag should be set. */
int gRequireNonNegativeRewards = FALSE;

/**********************************************************************/
double worstPossibleValue() {
/*
  Often we would like to do some max or min procedure and require
  initialization to the most extreme value.  This routine used to
  detect whether there were rewards or costs for the immediate
  utilities, but because the getImmediateReward() rtouine in global.h
  now enforces everything to be rewards, this is just a simple thing.  
*/

  return( -1.0 * HUGE_VAL );

}  /* worstPossibleValue */
/**********************************************************************/
double bestPossibleValue() {
/*
   Often we would like to do some max or min procedure and require
   initialization to the most extreme value.  This routine used to
  detect whether there were rewards or costs for the immediate
  utilities, but because the getImmediateReward() rtouine in global.h
  now enforces everything to be rewards, this is just a simple thing.  
*/
  return( HUGE_VAL );

}  /* bestPossibleValue */

/**********************************************************************/
void setPossibleObservations( double epsilon ) {
/*
  Sets the global arrays to precomputed values to determine whether or
  not each observation is possible for a given action.  Also stores
  how many observations are possible for each action.
*/
  int a, z, j, cur_state;
  int all_zero_prob_obs;

  for ( a = 0; a < gNumActions; a++ ) {

    for ( z = 0; z < gNumObservations; z++ ) {
      
      /* We want to check for the case where an observation is
         impossible.  */

      all_zero_prob_obs = TRUE;
      for ( cur_state = 0; cur_state < gNumStates; cur_state++)
        for ( j = P[a]->row_start[cur_state]; 
              j < P[a]->row_start[cur_state] 
                + P[a]->row_length[cur_state];
              j++ ) 
          if ( ! Equal( getEntryMatrix( R[a], P[a]->col[j], z ),
                        0.0, epsilon )) {
            all_zero_prob_obs = FALSE;
      
            /* Yeah, it's a 'goto'; just so I can say I used one. */
            goto END_LOOP;
          }
      
    END_LOOP:
      
      if ( all_zero_prob_obs )
        gObservationPossible[a][z] = FALSE;
      
      else  {
        gObservationPossible[a][z] = TRUE;
        gNumPossibleObservations[a]++;
      }  /* if observation is possible */
      
    } /* for z */
  
  } /* for a */

  /* A little sanity check. */
  for ( a = 0; a < gNumActions; a++ )
    Assert( gNumPossibleObservations[a] > 0,
            "Bad POMDP. No observations possible for some action." );

}  /* setPossibleObservations */
/**********************************************************************/
void initializePomdp( char *filename, 
                      double obs_possible_epsilon ) {
/*
  Does the necessary things to read in and set-up a POMDP file.
  Also precomputes which observations are possible and which are not.
*/
  int a;
  char msg[MAX_MSG_LENGTH];
  
  if (( filename == NULL ) || ( filename[0] == NULL_CHAR )) {
    sprintf( msg,
             "No parameter file specified (Use '%s' for options.)",
             CMD_ARG_HELP_SHORT );
    Abort( msg );
  }
  
  if ( ! readMDP( filename )) {
    sprintf( msg, "Could not successfully parse file: %s.\n",
             filename );
    Abort( msg );
  } /* if problem parsing POMDP file. */
  
  if ( gProblemType != POMDP_problem_type ){
    sprintf( msg,
             "Parameter file is not a POMDP specification." );
    Abort( msg );
  }

  /* We'll use this stuff if the setPossibleObservations() routine is
     called. */ 
  gObservationPossible 
    = (int **) XMALLOC( gNumActions 
                       * sizeof( *gObservationPossible ));
  for ( a = 0; a < gNumActions; a++ )
    gObservationPossible[a]
      = (int *) XCALLOC( gNumObservations, 
                        sizeof( **gObservationPossible ));
  
  gNumPossibleObservations
    = (int *) XCALLOC( gNumActions,
                      sizeof( *gNumPossibleObservations ));

  setPossibleObservations( obs_possible_epsilon );

}  /* initializePomdp */
/**********************************************************************/
void cleanUpPomdp(  ) {
/*
  Deallocates the POMDP read in by initializePomdp().
*/
  int a;

  for ( a = 0; a < gNumActions; a++ )
    XFREE( gObservationPossible[a] );
  XFREE( gObservationPossible );
  
  XFREE( gNumPossibleObservations );
  
  deallocateMDP();

}  /* cleanUpPomdp */
/************************************************************************/
double getAdjustedReward( int action, int state ) {
/*
  Although the getEntryMatrix() routine is normally used to extract
  matrix entries, we provide this routine for the immediate reward
  (utilities) matrix for two purposes:  First, the pomdp-solve code
  itself no longer deals with with utilities that are specified in
  terms of costs.  This routine will mask this fact by multiplying all
  immediate rewards by -1.  Second, it is often desirable to have
  non-negative rewards. Any problem can be converted to one of these
  by adding the appropriate offset.  The routine can do this as well.
  Note that the actual value functions will be skewed and require some
  form of rescaling to make sense.

  To force only non-negative rewards, set the global flag
  gRequireNonNegativeRewards before calling this routine. 

  If you want cost utilities and do not mind negative values,
  the you should access the immediate rewards directly with:

     getEntryMatrix( Q, a, state ) 
*/
  double reward;

  reward = getEntryMatrix( Q, action, state );

  if( gValueType == COST_value_type )
    reward *= -1.0;

  if (( gRequireNonNegativeRewards )
      && ( gMinimumImmediateReward < 0.0 ))
    reward -= gMinimumImmediateReward;

  return ( reward );

}  /* getAdjustedReward */
/**********************************************************************/
int valuesRequireScaling(  ) {
/*
  Returns a boolean value as to whether or not the immediate rewards
  have to be scaled (and the value functions that are calculated as
  well.)  This adjustment is either due to a non-negativity
  constraint or a COST utility which was converted to REWARD.
*/
  return ((( gRequireNonNegativeRewards )
           && ( gMinimumImmediateReward < 0.0 ))
          || ( gValueType == COST_value_type ));
  
}  /* valuesRequireScaling */
/**********************************************************************/
double getValueScaleFactor( int num_updates ) {
/*
  Because we use the getAdjustedReward() routine to access the
  rewards, the value function at a given time might not be for the
  underlying POMDP since the values are skewed.  To bring the values
  back to the solution to the POMDP requires undoing the scaling that
  getAdjustedReward() did.  This scaling is based upon the number of
  value function updates, since this is how many times the adjusted
  reward was incorporated into the values.  To scale, you should add
  the value returned.
*/

  double scale_factor = 0.0;

  /* Only need to adjust reward if the immediate rewards were
     adjusted for non-negativity. */
  if (( gRequireNonNegativeRewards )
      && ( gMinimumImmediateReward < 0.0 )) {
    
    /* If no discount was being used, then the amount of extra reward
       accumulated is just the amount of adjustment times the number of
       epochs. */
    if ( gDiscount == 1.0 )
      scale_factor = gMinimumImmediateReward * num_updates;

    else
      scale_factor = gMinimumImmediateReward 
        * (1.0 - pow(gDiscount,num_updates-1)) / (1.0 - gDiscount);

  } /* if need to adjust reward for non-negativity */
  
  if( gValueType == COST_value_type )
    scale_factor *= -1.0;
  
  return ( scale_factor );

}  /* getValueScaleFactor */
/**********************************************************************/
double scaleValue( double value, int num_updates ) {
/*
  As per the getImmediateReward() rotuine, there are two manners in
  which the POMDP immediate rewards could have been adjusted.  First
  they could have been adjusted to convert COSTS to REWARDS.  Next
  they might have been adjusted to be non-negative.  Because of these
  two conversions, when you get a value function, it is not really the
  value function of the underlying POMDP.  However, we can convert it
  to an equivalent value function by re-adjusting the rewards. Because
  there can be discounting involved, and rewards are accumulated, how
  much the values need to be adjusted depend upon how many value
  iteration epochs have passed, which is why you need to specify this.
*/
  
  return ( value + getValueScaleFactor( num_updates ));
}  /* scaleValue */
/**********************************************************************/
