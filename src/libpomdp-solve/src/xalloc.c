
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    xalloc.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    May, 2003
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: xalloc.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/xalloc.c,v $
 *    $Revision: 1.3 $
 *    $Date: 2004/10/10 03:44:54 $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    2003, Anthony R. Cassandra
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
 *   Wrappers for memory allocation routines.
 */

#include <stdlib.h>

#include "common.h"

/**********************************************************************/
void *
XA_malloc (size_t num)
{
  void *new = (void *) malloc (num);
  if (!new)
    Abort("Out of memory in malloc()!");
  return new;
}
/**********************************************************************/
void *
XA_realloc (void *p, size_t num)
{
  void *new;

  if (!p)
    return XA_malloc (num);

  new = (void *) realloc (p, num);
  if (!new)
    Abort("Out of memory in realloc()!");

  return new;
}
/**********************************************************************/
void *
XA_calloc (size_t num, size_t size)
{
  void *new;

#if HAVE_MEMSET
  new = XA_malloc (num * size);
  memset(new, '\0', num * size);
#else

#if HAVE_BZERO
  new = XA_malloc (num * size);
  bzero(new, num * size);
#else
  new = calloc (num, size);
  if (!new)
    Abort("Out of memory in calloc()!");
#endif

#endif

  return new;
}
/**********************************************************************/
