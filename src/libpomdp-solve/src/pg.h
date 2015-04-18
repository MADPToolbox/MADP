
/*
 *<SOURCE_HEADER>
 *
 *  <NAME>
 *    pg.c
 *  </NAME>
 *  <AUTHOR>
 *    Anthony R. Cassandra
 *  </AUTHOR>
 *  <CREATE_DATE>
 *    March, 2004
 *  </CREATE_DATE>
 *
 *  <RCS_KEYWORD>
 *    $RCSfile: pg.h,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/pg.h,v $
 *    $Revision: 1.5 $
 *    $Date: 2004/11/01 04:27:34 $
 *  </RCS_KEYWORD>
 *
 *  <COPYRIGHT>
 *
 *    2004, Anthony R. Cassandra
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

#ifndef PG_H
#define PG_H

#include "alpha.h"

/* FIXME: Make this go away and allow larger graphs.  This was done so
   I did not have to parse the file twice when reading. 
*/
#define PG_MAX_SIZE 10000

/* Useful mnemonic for a node ID that  is non-valid.  note that all
   node IDs must be non-negative. */
#define INVALID_NODE -1
#define NO_INFO_NODE -2

#define INVALID_NODE_STR "X"
#define NO_INFO_NODE_STR "-"

/* This linked list data structure is used for arbitarily large policy
   graphs.
   */
typedef struct Linked_PG_Struct *LinkedPG;
struct Linked_PG_Struct {
  
  int id;      /* A unique identifier for the node */
  int action;  /* action associated with the node */

  int *neighbor;   /* An array of id's, one for each observation
		      which tells us which node to go to after an 
		      observation. */

  LinkedPG next;
};

/* This data structure is used for fixed sized policy graphs.
   Static arrays for faster access.
   */
typedef struct PG_Struct *PG;
struct PG_Struct {

  int num_nodes;
  int num_states;
  int num_actions;
  int num_obs;

  int *marked;

  int *id;
  int *action;
  int **next;

};


extern LinkedPG LPG_addPGNode( LinkedPG pg, int id, int action, 
						 int *neighbor );
extern int LPG_size( LinkedPG pg );
extern void LPG_display( LinkedPG pg );
extern LinkedPG LPG_read( char *filename, int verify );
extern void LPG_write( LinkedPG lpg, char *filename );
extern LinkedPG LPG_findNode( LinkedPG pg, int id );


extern PG PG_Constructor( int num_nodes, 
                          int num_states,
                          int num_actions, 
                          int num_obs  );
extern PG PG_ConstructorFromAlphaList( AlphaList );
extern void PG_Destructor( PG pg );
extern PG PG_clone( PG pg );
extern void PG_rand( PG pg );
extern void PG_printf( PG pg, FILE *file  );
extern PG PG_scanf( FILE *file, int verify );
extern void PG_display( PG pg  );
extern void PG_write( PG pg, char *filename  );
extern PG PG_read( char *filename, int verify  );

extern PG PG_convertLPGToPG( LinkedPG old_pg );
extern void PG_relink( PG pg, int* link_map, int max_idx );

extern void APG_displayPolicyGraph( FILE *file, AlphaList list );
extern void APG_writePolicyGraph( AlphaList list, char *filename );
extern void APG_showPolicyGraph( AlphaList list );

#endif
