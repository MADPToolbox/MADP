
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    projection.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: projection.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/projection.h,v $
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
#ifndef PROJECTION_H
#define PROJECTION_H

/*******************************************************************/
/**************       EXTERNAL VARIABLES            ****************/
/*******************************************************************/

/*******************************************************************/
/**************       EXTERNAL FUNCTIONS            ****************/
/*******************************************************************/

/* Just creates and returns the storage to hold all the projection
  sets.  They will initially all be NULL.  */
extern AlphaList **allocateAllProjections(  );

/* Discards all the projection lists and memory associated with them.  */
extern void freeAllProjections( AlphaList **projection );
 
/* Makes all the projected alpha vector lists, which amounts to a
   projected list for each action-observation pair. Stores this as a
   two dimensional array of lists where the first index is the action
   and the other is the observation.  This allocates the space for the
   projections first.  The 'impossible_obs_epsilon' specifies the
   tolerance to use when trying to determine whether or not a
   particulat observation is at all feabile.  */
extern AlphaList **makeAllProjections( AlphaList prev_alpha_list );

/* Displays all projections to file stream.  */
extern void displayProjections( FILE *file, AlphaList **projection );

/* Displays all projections to stdout.  */
extern void showProjections( AlphaList **projection );
        
#endif
