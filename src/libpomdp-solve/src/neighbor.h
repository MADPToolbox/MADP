
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    neighbor.h
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    July, 1998
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: neighbor.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/neighbor.h,v $
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
#ifndef NEIGHBOR_H
#define NEIGHBOR_H

/**********************************************************************/
/********************       CONSTANTS       ***************************/
/**********************************************************************/

/**********************************************************************/
/********************   DEFAULT VALUES       **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL VARIABLES   **************************/
/**********************************************************************/

/**********************************************************************/
/********************   EXTERNAL FUNCTIONS    *************************/
/**********************************************************************/

/* Adds a copy of the given neighbor to the list, but only if: 1)
   The neighbor isn't already in the list.  2) The neigbhbor isn't
   dominated by an existing neighbor.  Returns TRUE if copy of
   neighbor is added and false otherwise.  Other params: 'source' is
   the vector that generated this neighbor.  'z' is the observation
   for which this neighbor differs from the source.  'proj_z' is the
   vector that is different.  */
extern int addNeighbor( AlphaList list, 
                        double *neighbor,
                        AlphaList source, 
                        int z, 
                        AlphaList proj_z,
                        int domination_check,
                        double epsilon );

/* Adds all neighbors of the vector in the node into the list.  */
extern int addAllNeighbors( AlphaList list, 
                            AlphaList node, 
                            AlphaList *projection,
                            int domination_check,
                            double epsilon );

/* Adds all the "neighbors" of all the vectors in the list sent in
  into the list. Checks to make sure the neighbor isn't already
  in the list.  Returns the number of neighbors added.  */
extern int addAllNeighborsFromList( AlphaList dest_list, 
                                    AlphaList list,
                                    AlphaList *projection,
                                    int domination_check,
                                    double epsilon );
       
#endif
