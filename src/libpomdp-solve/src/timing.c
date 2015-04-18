
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    timing.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: timing.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/timing.c,v $
 *    $Revision: 1.2 $
 *    $Date: 2004/01/16 21:10:59 $
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
 *   Routines for handling CPU times.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* This has: 'CLK_TCK'  */
#include <time.h>
#include <unistd.h>

/* This has: 'struct tms', times() */
#include <sys/times.h>

#include "timing.h"

/**********************************************************************/
void 
getSecsDetail( double *user_time, double *system_time ) 
{
  /* Get total CPU time in seconds breaking it down by user
     and system time. */
  struct tms time;

  times( &time );
  
  *user_time = (double) time.tms_utime / (double) sysconf(_SC_CLK_TCK);
  *system_time = (double) time.tms_stime / (double) sysconf(_SC_CLK_TCK);

}  /* getSecsDetail */
/**********************************************************************/
double 
getSecs( ) 
{
  /* Get total CPU time in seconds including user and system time. */
  double user_time, system_time;

  getSecsDetail( &user_time, &system_time );

  return ( user_time + system_time );

}  /* getSecs */
/**********************************************************************/
void 
reportTimes( FILE *file, double tot_secs, char *str ) 
{
  /* 
     Report the total secons time in a nicer hr, min sec format with a
     string to label what the time is for. 
  */

   int hrs, mins;
   double secs;

   mins = (int)(tot_secs / 60.0) % 60;
   hrs = (int)(tot_secs / 3600.0) % 60;

   secs = tot_secs - 3600*hrs - 60*mins;

   fprintf( file,
            "%s %d hrs., %d mins, %.2lf secs. (= %.2lf secs)\n", 
            str, hrs, mins, secs, tot_secs );
   
}  /* reportTimes */
/**********************************************************************/
