
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    region.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    August, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: region.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/region.h,v $
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
#ifndef REGION_H
#define REGION_H

/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/* There are two ways to set up a region LP.  The original way and the
   way proposed by Bob Givan.  There is a global constant, when is set
   when you want to use the Givan approach. This results in
   conditional compilation of some parts of lp-interface.c. */

/*  #define USE_OLD_LP_FORMULATION  */


/* We can choose between representing things using the sparse LP
   representation or making the thing dense, adding in the zero
   entries. Defining this variable forces the use fo dense LP
   representation. */

/*  #define USE_DENSE_LPS  */

/**********************************************************************/
/********************   DEFAULT VALUES       **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/* Checks to see if the alpha vector 'alpha' has a non-empty region
  (measurable area) where it is better than all the other vectors in
  the 'list'. If the region is non-empty the routine returns TRUE with
  the witness_point set to a point in that region.  If there is no
  point where alpha is better, then FALSE is returned.  */
extern int findRegionPoint( double *alpha, 
                            AlphaList list, 
                            double *witness_point,
                            double *diff,
                            PomdpSolveParams param );

#endif
