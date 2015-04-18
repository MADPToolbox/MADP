
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    global.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: global.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/global.c,v $
 *    $Revision: 1.3 $
 *    $Date: 2004/10/10 03:44:53 $
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "mdp/mdp.h"

#include "global.h"

/* Nice to have pretty names for the boolean values 0 and 1. */
char *boolean_str[] = BOOLEAN_STRINGS;

/* Strings for the various stopping criteria */
char *verbose_mode_str[] = VERBOSE_MODE_STRINGS;

/* For some reason I could not assign this to 'stderr' here after
   1998.  I am moving this to be one of the first things that gets
   done in main(). */ 
FILE *gStdErrFile = NULL;

/* The name of the executable of this program. */
char gExecutableName[80];

/* There are various ways to turn verboseness on and off.  Each elemnt
   of the array defines whether one of these is on or off.  The
   mnemonics in the header file show which ones are which. */
int gVerbose[NUM_VERBOSE_MODES];

/**********************************************************************/
/* Temporary variables usefule for scratch work. */
/**********************************************************************/

/* There are times when we need an array of doubles for temporary
   workspace.  These vectors will be gNumStates in length. */
double *gTempValue;
double *gTempBelief;
double *gTempAlpha;

/**********************************************************************/
void 
initGlobal(  ) 
{
  /*
    Sets up and allocates variables that are used globally across
    modules in the program. Currently just allocates a bunch of scratch
    memory areas.
  */
  gTempBelief = (double *) XMALLOC( gNumStates * sizeof( double ));
  gTempAlpha = (double *) XMALLOC( gNumStates * sizeof( double ));
  gTempValue = (double *) XMALLOC( gNumStates * sizeof( double ));
  
}  /* initGlobal */
/**********************************************************************/
void 
cleanUpGlobal(  ) 
{
  /*
    Cleans up after problem is solved to free any resources and reset
    anything that the initGlobal() routine did.
  */

  XFREE( gTempBelief );
  XFREE( gTempAlpha );
  XFREE( gTempValue );

}  /* cleanUpGlobal */
/**********************************************************************/
int 
getPid(  ) 
{
  /* 
     Just a wrapper to the UN*X getpid() function to isolate it in case
     this gets ported to another platform.  Note that for POSIX, the
     'pid_t' type returned by getpid() is an 'int'.
  */
  return( (int) getpid() );
}  /* getPid */
/**********************************************************************/
void 
removeFile( char *filename ) 
{
  /* 
     Just a wrapper to the UN*X unlink() function to isolate it in case
     this gets ported to another platform.  
  */

  unlink( filename );

}  /* removeFile */
/**********************************************************************/
void 
strupr ( char *string )
{
  /* 
	Converts a string to upper case
  */
  while ( *string )
    {
	 *string = toupper ( *string );
	 ++string;
    }

}/**********************************************************************/
