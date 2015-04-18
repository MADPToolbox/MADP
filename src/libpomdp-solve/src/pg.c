
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
 *    $RCSfile: pg.c,v $
 *    $Source: /u/cvs/proj/pomdp-solve/src/pg.c,v $
 *    $Revision: 1.6 $
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

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "global.h"
#include "mdp/mdp.h"

#include "pomdp.h"
#include "alpha.h"
#include "pg.h"

/**********************************************************************/
/*****************     LinkedPG Routines     **************************/
/**********************************************************************/

/*
  This is for policy graphs stored in a linked list manner.
 */

/**********************************************************************/
LinkedPG 
LPG_newNode( int id, int action, int *neighbor ) {
  LinkedPG pg;
  int z;

  pg = (LinkedPG) XMALLOC( sizeof( *pg ));
  pg->id = id;
  pg->action = action;
  pg->neighbor = (int *) XCALLOC( gNumObservations, sizeof( int ));

  if( neighbor != NULL )
    for( z = 0; z < gNumObservations; z++ )
      pg->neighbor[z] = neighbor[z];

  pg->next = NULL;

  return( pg );
}  /* LPG_newPolicyGraphNode */

/**********************************************************************/
void 
LPG_destroyNode( LinkedPG pg ) {

  if( pg == NULL )
    return;

  XFREE( pg->neighbor );
  XFREE( pg );

}  /* LPG_destroyNode */

/**********************************************************************/
void 
LPG_destroy( LinkedPG pg ) {
  LinkedPG temp;

  while( pg != NULL ) {

    temp = pg;
    pg = pg->next;

    LPG_destroyNode( temp );
  }  /* while */

}  /* LPG_destroy */

/**********************************************************************/
LinkedPG 
LPG_append( LinkedPG pg, LinkedPG node ) {
  LinkedPG temp = pg;

  if( pg == NULL )
    return( node );

  while( temp->next != NULL ) 
    temp = temp->next;

  temp->next = node;

  return( pg );
}  /* LPG_append */

/**********************************************************************/
LinkedPG 
LPG_addNode( LinkedPG pg, int id, int action, int *neighbor ) {
  LinkedPG node;
  
  node = LPG_newNode( id, action, neighbor );

  pg = LPG_append( pg, node );

  return( pg );
}  /* LPG_addNode */

/**********************************************************************/
LinkedPG 
LPG_findNode( LinkedPG pg, int id ) {
/*
   Returns '1' if there is a node in the graph with 'id'.
*/
  while( pg != NULL ) {
    if( pg->id == id )
      return( pg );

    pg = pg->next;
  }

  return( NULL );
}  /* LPG_findNode */

/**********************************************************************/
int 
LPG_verify( LinkedPG orig_pg ) {
/*
   Make sure the policy graph is valid.  Need valid actions and
   all neighbors must be defined.
*/
  LinkedPG pg = orig_pg;
  int z;

  while( pg != NULL ) {

    if(( pg->action < 0 ) || ( pg->action >= gNumActions ))
      return( 0 );

    for( z = 0; z < gNumObservations; z++ )
	 {

	   if( LPG_findNode( orig_pg, pg->neighbor[z] ) == NULL )
		return( 0 );

	 } /* for z */

    pg = pg->next;
  }  /* while */

  return( 1 );
}  /* LPG_verify */

/**********************************************************************/
LinkedPG 
LPG_scanf( FILE *file, int verify ) {

  /*
    Reads a policy graph from a file, and optionally will do some
    sanity checks (if verify is non-zero ).
   */

  LinkedPG lpg = NULL;
  int id, action;
  int *neighbor;
  int z;
  char int_str[MAX_MSG_LENGTH];
  int count = 0;

  neighbor = (int *) XMALLOC( gNumObservations * sizeof( int ));

   while( 1 ) {

     if( fscanf( file, "%d", &id ) == EOF )
       break;

     if( fscanf( file, "%d", &action ) == EOF ) {
	  LPG_destroy( lpg );
	  Warning( "Not enough values in the policy graph file.");
	  return( NULL );
     }

	if (( action < 0 ) 
	    || ( action >= gNumActions ))
	  {
	    LPG_destroy( lpg );
	    Warning( "Bad action number.\n" );
	    return NULL;
	  }
	
	for( z = 0; z < gNumObservations; z++ ) {
	 
	  if ( fscanf( file, "%s", int_str) == EOF )
	    {
		 LPG_destroy( lpg );
		 Warning("Not enough values in the policy graph file." );
		 return NULL;
	    }
	  
	  if ( strcmp( int_str, INVALID_NODE_STR ) == 0 )
	    neighbor[z] = INVALID_NODE;
	  else if ( strcmp( int_str, NO_INFO_NODE_STR ) == 0 )
	    neighbor[z] = NO_INFO_NODE;
	  else
	    neighbor[z] = atoi( int_str );
	}

	lpg = LPG_addNode( lpg, id, action, neighbor );
	count++;
   }  /* while */
  
  XFREE( neighbor );

  Assert( count > 0, "Node pg nodes read." );
  
  /* DO sanity checking, but only if requested. */
  if ( verify
	  && ( !LPG_verify( lpg ))) {
     LPG_destroy( lpg );
     Warning( "Policy graph in file is not valid.");
     return( NULL );
  }

  return( lpg );

}  /* LPG_scanf */

/**********************************************************************/
LinkedPG 
LPG_read( char *filename, int verify ) {

  LinkedPG lpg = NULL;
  FILE *file;

  Assert( filename != NULL, "NULL filename" );

  if (( file = fopen( filename , "r" )) == NULL) 
    {
	 Abort( "Cannot open policy graph file for reading." );
    }
  
  lpg = LPG_scanf( file, verify );
  
  fclose( file );

  return lpg;
  
}  /* LPG_read */



/**********************************************************************/
void
LPG_printf( LinkedPG lpg, FILE *file ) {

  PG pg;

  pg = PG_convertLPGToPG( lpg );

  PG_printf( pg, file );

  PG_Destructor( pg );

}  /* LPG_printf */

/**********************************************************************/
void 
LPG_display( LinkedPG lpg ) {

  LPG_printf( lpg, stdout );

}  /* LPG_display */

/**********************************************************************/
void
LPG_write( LinkedPG lpg, char *filename ) {

  PG pg;

  pg = PG_convertLPGToPG( lpg );

  PG_write( pg, filename );

  PG_Destructor( pg );

}  /* LPG_write */

/**********************************************************************/
int  
LPG_size( LinkedPG pg ) {
  int count = 0;

  while( pg != NULL ) {
    
    count++;
    pg = pg->next;
  }  /* while */

  return( count );
}  /* LPG_size */

/**********************************************************************/
int  
LPG_getPGNodePosition( LinkedPG pg, int id ) {
/*
   Returns the node number's position in the linked list with '0'
   being at the front.  Returns -1 if 'id' not found or if policy
   graph is NULL.
*/
  int position = 0;

  while( pg != NULL ) {

    if( pg->id == id )
      return( position );

    position++;
    pg = pg->next;
  }

  return( -1 );
}  /* LPG_getPGNodePosition */

/**********************************************************************/
/*****************        PG Routines        **************************/
/**********************************************************************/

/*
  This is for policy graphs stored as a set of fixed sized arrays.
 */

/**********************************************************************/
PG 
PG_Constructor( int num_nodes, int num_states,
			 int num_actions, int num_obs ) 
{
  PG pg;
  int n;
  
  Assert( num_nodes > 0 && num_obs > 0, "Bad parameters" );

  pg = (PG) XMALLOC( sizeof( *pg ));

  pg->num_nodes = num_nodes;
  pg->num_states = num_states;
  pg->num_actions = num_actions;
  pg->num_obs = num_obs;

  pg->marked = (int *) XMALLOC( num_nodes * sizeof( int ));
  pg->id = (int *) XMALLOC( num_nodes * sizeof( int ));
  pg->action = (int *) XMALLOC( num_nodes * sizeof( int ));

  pg->next = (int **) XMALLOC( num_nodes * sizeof( *(pg->next) ) );
  for ( n = 0; n < num_nodes; n++ )
    {
	 pg->marked[n] = FALSE;
	 pg->id[n] = n;
	 pg->next[n] = (int *) XMALLOC( num_obs * sizeof( int ));
    }

  return ( pg );

}  /* PG_Constructor */

/**********************************************************************/
PG 
PG_ConstructorFromAlphaList( AlphaList list ) 
{
  int z;
  int n;
  PG pg;

  Assert( list != NULL, "Bad (NULL) parameter(s)." );

  pg = PG_Constructor( sizeAlphaList(list), 
				   gNumStates, gNumActions, gNumObservations );

  list = list->head;
  n = 0;

  while( list != NULL ) {

    pg->id[n] = list->id;
    pg->action[n] = list->action;

    if ( list->obs_source != NULL ) {

      for ( z = 0; z < gNumObservations; z++ ) {


        if ( list->obs_source[z] != NULL )
		pg->next[n][z] = list->obs_source[z]->id;

	   /* We put a special value for when the observation is
		 impossible. */ 
        else
		pg->next[n][z] = INVALID_NODE;

      }  /* for z */          

    } /* if have a setb of next pg pointers */

    else {

      for ( z = 0; z < gNumObservations; z++ )
	   pg->next[n][z] = NO_INFO_NODE;

    }
    
    list = list->next;
    n++;
  }  /* while list != NULL */
 
  return pg;

} /* PG_ConstructorFromAlphaList */

/**********************************************************************/
void 
PG_Destructor( PG pg ) 
{
  int n;

  if ( pg == NULL )
    return;

  for ( n = 0; n < pg->num_nodes; n++ )
    XFREE( pg->next[n] );
  XFREE( pg->next );
  XFREE( pg->action );
  XFREE( pg );

}  /* PG_Destructor */
/**********************************************************************/
PG 
PG_clone( PG pg ) 
{
  int n, z;
  PG clone;

  Assert( pg != NULL, "Bad parameters" );

  clone = PG_Constructor( pg->num_nodes, pg->num_states,
                          pg->num_actions, pg->num_obs );

  for ( n = 0; n < pg->num_nodes; n++ ) 
    {
	 
	 clone->marked[n] = pg->marked[n];
	 clone->id[n] = pg->id[n];
	 clone->action[n] = pg->action[n];
	 
	 for( z = 0; z < pg->num_obs; z++ )
	   clone->next[n][z] = pg->next[n][z];
	 
    } /* for n */

  return ( clone );

}  /* PG_clone */

/**********************************************************************/
void 
PG_printf( PG pg, FILE *file  ) 
{
  /*
    Displays the policy graph to the file handle specified.
    
    The policy graph will be output with the format of one line per node
    in the policy graph:
    
    ID  ACTION    OBS1  OBS2 OBS3 ... OBSN
    
    where ID is the id of the alpha vector in the current set, ACTION is
    the action for this vector, and OBS1 through OBSN are the id's of
    the vectors in the previous epoch's alpha vector set (one for each
    observation). 
  */
  int n, z;

  Assert( pg != NULL && file != NULL, "Bad parameters" );

  for ( n = 0; n < pg->num_nodes; n++ ) 
    {
	 
	 fprintf( file, "%d %d  ", n, pg->action[n] );
	 
	 for( z = 0; z < pg->num_obs; z++ )
	   if ( pg->next[n][z] != INVALID_NODE )
		fprintf( file, "%d ", pg->next[n][z] );
	   else if ( pg->next[n][z] == NO_INFO_NODE )
		fprintf( file, "%s ", INVALID_NODE_STR );
	   else
		fprintf( file, "%s ", NO_INFO_NODE_STR );
	 
	 fprintf( file, "\n");
    }  /* for n */

}  /* PG_printf */

/**********************************************************************/
PG
PG_scanf( FILE *file, int verify )
{

  LinkedPG lpg;
  PG pg;

  lpg = LPG_scanf( file, verify );
  
  if ( lpg == NULL )
    return NULL;

  pg = PG_convertLPGToPG( lpg );

  LPG_destroy( lpg );

  return pg;
  
} /* PG_scanf */

/**********************************************************************/
void 
PG_display( PG pg  ) 
{
  PG_printf( pg, stdout );
} /* PG_display */

/**********************************************************************/
void 
PG_write( PG pg, char *filename  ) 
{
  FILE *file;

  Assert( pg != NULL && filename != NULL, "Bad parameters" );

  if (( file = fopen( filename , "w" )) == NULL) 
    {
	 Warning( "Cannot open policy graph file for writing." );
	 return;
    }
  
  PG_printf( pg, file );
  
  fclose( file );
}  /* PG_write */

/**********************************************************************/
PG
PG_read( char *filename, int verify )
{
  LinkedPG lpg;
  PG pg;

  lpg = LPG_read( filename, verify );
  
  if ( lpg == NULL )
    return NULL;

  pg = PG_convertLPGToPG( lpg );

  LPG_destroy( lpg );

  return pg;

} /* PG_read */

/**********************************************************************/
PG PG_convertLPGToPG( LinkedPG old_pg ) {
/*
  Converts from a policy graph in the linked-list representation into
  a policy graph in the array-based representation. 
*/

  PG pg;
  int z;

  Assert ( old_pg != NULL, "Bad parameters" );

  pg = PG_Constructor( LPG_size( old_pg ), gNumStates,
                       gNumActions, gNumObservations );
  
  while( old_pg != NULL ) {

    pg->action[old_pg->id] = old_pg->action;

    for( z = 0; z < gNumObservations; z++ )
      pg->next[old_pg->id][z] = old_pg->neighbor[z];
    
    old_pg = old_pg->next;
  }  /* while */

  return ( pg );

}  /* PG_convertLPGToPG */

/**********************************************************************/
void PG_relink( PG pg, int* link_map, int max_idx ) {
  /*
    Will map all the pg observation links from one set of indices to
    another. Changes the policy graph in place. We pass in the max_idx
    which should be the maximum value that any current link index is.
  */
  int n, z;

  Assert( ( pg != NULL ) && ( link_map != NULL ), "Bad parameters" );

  for ( n = 0; n < pg->num_nodes; n++ ) 
    {

	 for( z = 0; z < pg->num_obs; z++ )
	   {
		/* Handle special case of "X" entries */
		if ( pg->next[n][z] < 0 )
		  continue;

		if ( pg->next[n][z] > max_idx )
		  {
		    Warning( "Link index too large. Policy graph mismatch?" );
		    pg->next[n][z] = NO_INFO_NODE;
		    continue;
		  }

		pg->next[n][z] = link_map[pg->next[n][z]];
	   }
	 
    } /* for n */

} /* PG_relink */

/**********************************************************************/
/*****************        APG Routines        *************************/
/**********************************************************************/

/*
  This is for policy graphs stored as a part of an AlpahList linked list.
 */

/**********************************************************************/
void 
APG_displayPolicyGraph( FILE *file, AlphaList list ) 
{
  /*
    Displays the policy graph to the file handle specified.
    
    The policy graph will be output with the format of one line per node
    in the policy graph:
    
    ID  ACTION    OBS1  OBS2 OBS3 ... OBSN
    
    where ID is the id of the alpha vector in the current set, ACTION is
    the action for this vector, and OBS1 through OBSN are the id's of
    the vectors in the previous epoch's alpha vector set (one for each
    observation). 
  */
  PG pg;

  pg = PG_ConstructorFromAlphaList( list );

  PG_printf( pg, file );

  PG_Destructor( pg );

}  /* displayPolicyGraph */

/**********************************************************************/
void 
APG_writePolicyGraph( AlphaList list, char *filename ) 
{
  /*
    Displays the policy graph of a set of vectors to the filename
    specified. 
  */
  PG pg;

  pg = PG_ConstructorFromAlphaList( list );

  PG_write( pg, filename );

  PG_Destructor( pg );

}  /* writePolicyGraph */

/**********************************************************************/
void 
APG_showPolicyGraph( AlphaList list ) 
{
  /*
    Displays the policy graph for an alpha vector set to stdout.
  */
  APG_displayPolicyGraph( stdout, list );
}  /* showPolicyGraph */

/**********************************************************************/
