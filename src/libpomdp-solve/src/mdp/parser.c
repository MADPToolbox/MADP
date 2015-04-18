#ifndef lint
static char yysccsid[] = "@(#)yaccpar	1.8 (Berkeley) 01/20/90";
#endif
#define YYBYACC 1
#line 2 "parser.y"
/*
  *****
  Copyright 1994-1997, Brown University
  Copyright 1998, 1999, Anthony R. Cassandra

                           All Rights Reserved
                           
  Permission to use, copy, modify, and distribute this software and its
  documentation for any purpose other than its incorporation into a
  commercial product is hereby granted without fee, provided that the
  above copyright notice appear in all copies and that both that
  copyright notice and this permission notice appear in supporting
  documentation.
  
  ANTHONY CASSANDRA DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
  INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR ANY
  PARTICULAR PURPOSE.  IN NO EVENT SHALL ANTHONY CASSANDRA BE LIABLE FOR
  ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
  *****

*/
#include <stdio.h>

#include "mdp-common.h"
#include "parse_err.h"
#include "mdp.h"
#include "parse_hash.h"
#include "parse_constant.h"
#include "sparse-matrix.h"
#include "imm-reward.h"

#define YACCtrace(X)       /*   printf(X);fflush(stdout)    */ 

/* When reading in matrices we need to know what type we are reading
   and also we need to keep track of where in the matrix we are
   including how to update the row and col after each entry is read. */
typedef enum { mc_none, mc_trans_single, mc_trans_row, mc_trans_all,
               mc_obs_single, mc_obs_row, mc_obs_all,
               mc_reward_single, mc_reward_row, 
               mc_reward_all, mc_reward_mdp_only,
               mc_start_belief, mc_mdp_start, 
               mc_start_include, mc_start_exclude } Matrix_Context;

extern int yylex();

/* Forward declaration for action routines which appear at end of file */
void yyerror(char *string);
void checkMatrix();
void enterString( Constant_Block *block );
void enterUniformMatrix( );
void enterIdentityMatrix( );
void enterResetMatrix( );
void enterMatrix( double value );
void setMatrixContext( Matrix_Context context, 
                      int a, int i, int j, int obs );
void enterStartState( int i );
void setStartStateUniform();
void endStartStates();
void verifyPreamble();
void checkProbs();

/*  Helps to give more meaningful error messages */
long currentLineNumber = 1;

/* This sets the context needed when names are given the the states, 
   actions and/or observations */
Mnemonic_Type curMnemonic = nt_unknown;

Matrix_Context curMatrixContext = mc_none;

/* These variable are used to keep track what type of matrix is being entered and
   which element is currently being processed.  They are initialized by the
   setMatrixContext() routine and updated by the enterMatrix() routine. */
int curRow;
int curCol;
int minA, maxA;
int minI, maxI;
int minJ, maxJ;
int minObs, maxObs;

/*  These variables will keep the intermediate representation for the
    matrices.  We cannot know how to set up the sparse matrices until
    all entries are read in, so we must have this intermediate 
    representation, which will will convert when it has all been read in.
    We allocate this memory once we know how big they must be and we
    will free all of this when we convert it to its final sparse format.
    */
I_Matrix *IP;   /* For transition matrices. */
I_Matrix *IR;   /* For observation matrices. */
I_Matrix **IW;  /* For reward matrices */

/* These variables are used by the parser only, to keep some state
   information. 
*/
/* These are set when the appropriate preamble line is encountered.  This will
   allow us to check to make sure each is specified.  If observations are not
   defined then we will assume it is a regular MDP, and otherwise assume it 
   is a POMDP
   */
int discountDefined = 0;
int valuesDefined = 0;
int statesDefined = 0;
int actionsDefined = 0;
int observationsDefined = 0;

/* We only want to check when observation probs. are specified, but
   there was no observations in preamble. */
int observationSpecDefined = 0;

/* When we encounter a matrix with too many entries.  We would like
   to only generate one error message, instead of one for each entry.
   This variable is cleared at the start of reading  a matrix and
   set when there are too many entries. */
int gTooManyEntries = 0;

#line 131 "parser.y"
typedef union {
  Constant_Block *constBlk;
  int i_num;
  double f_num;
} YYSTYPE;
#line 131 "parser.c"
#define INTTOK 1
#define FLOATTOK 2
#define COLONTOK 3
#define MINUSTOK 4
#define PLUSTOK 5
#define STRINGTOK 6
#define ASTERICKTOK 7
#define DISCOUNTTOK 8
#define VALUESTOK 9
#define STATETOK 10
#define ACTIONTOK 11
#define OBSTOK 12
#define TTOK 13
#define OTOK 14
#define RTOK 15
#define UNIFORMTOK 16
#define IDENTITYTOK 17
#define REWARDTOK 18
#define COSTTOK 19
#define RESETTOK 20
#define STARTTOK 21
#define INCLUDETOK 22
#define EXCLUDETOK 23
#define EOFTOK 258
#define YYERRCODE 256
short yylhs[] = {                                        -1,
    9,   11,    0,    7,    7,   12,   12,   12,   12,   12,
   13,   14,   18,   18,   20,   15,   19,   19,   23,   16,
   22,   22,   25,   17,   24,   24,   27,    8,    8,   29,
    8,   30,    8,    8,   28,   28,   10,   10,   31,   31,
   31,   32,   36,   35,   37,   35,   39,   35,   33,   41,
   40,   42,   40,   43,   40,   34,   45,   44,   47,   44,
   48,   44,   49,   44,   38,   38,   38,   26,   26,   26,
   50,   50,   46,   46,    2,    2,    2,    1,    1,    1,
    3,    3,    3,   21,   21,    6,    6,    5,    5,    4,
    4,    4,
};
short yylen[] = {                                         2,
    0,    0,    5,    2,    0,    1,    1,    1,    1,    1,
    3,    3,    1,    1,    0,    4,    1,    1,    0,    4,
    1,    1,    0,    4,    1,    1,    0,    4,    3,    0,
    5,    0,    5,    0,    2,    1,    2,    0,    1,    1,
    1,    3,    0,    7,    0,    5,    0,    3,    3,    0,
    7,    0,    5,    0,    3,    3,    0,    9,    0,    7,
    0,    5,    0,    3,    1,    1,    1,    1,    1,    1,
    2,    1,    2,    1,    1,    1,    1,    1,    1,    1,
    1,    1,    1,    2,    1,    1,    1,    2,    2,    1,
    1,    0,
};
short yydefred[] = {                                      5,
    0,    0,    0,    0,    0,    0,    0,    0,    4,    6,
    7,    8,    9,   10,    0,    0,   15,   19,   23,    0,
    2,   91,   90,    0,   11,   13,   14,   12,    0,    0,
    0,    0,    0,    0,   38,   88,   89,   17,   85,   16,
    0,   21,    0,   20,   25,    0,   24,   29,    0,   30,
   32,    0,   84,   86,   87,   68,   69,   72,   28,    0,
    0,    0,    0,    0,    0,   37,   39,   40,   41,   71,
   75,   76,   77,   36,    0,    0,    0,    0,    0,   35,
   78,   79,   80,    0,   42,    0,   49,    0,   56,    0,
    0,    0,    0,    0,    0,    0,   65,   66,   48,    0,
    0,   55,    0,   74,    0,    0,    0,    0,    0,    0,
    0,   73,   43,   46,   81,   82,   83,   50,   53,    0,
    0,    0,    0,    0,    0,   44,   51,   57,    0,    0,
   58,
};
short yydgoto[] = {                                       1,
   84,   74,  118,   24,  104,   58,    2,   21,    8,   52,
   35,    9,   10,   11,   12,   13,   14,   28,   40,   29,
   41,   44,   30,   47,   31,   59,   49,   75,   61,   62,
   66,   67,   68,   69,   85,  122,  107,   99,   91,   87,
  123,  109,   93,   89,  130,  105,  125,  111,   95,   60,
};
short yysindex[] = {                                      0,
    0,  156,   29,   51,   53,   55,   84,   41,    0,    0,
    0,    0,    0,    0,   35,  164,    0,    0,    0,    8,
    0,    0,    0,  183,    0,    0,    0,    0,  137,  152,
  153,   83,   94,   96,    0,    0,    0,    0,    0,    0,
   95,    0,   95,    0,    0,   95,    0,    0,   59,    0,
    0,  166,    0,    0,    0,    0,    0,    0,    0,  185,
    6,    6,  104,  114,  134,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    6,    6,    9,    9,    9,    0,
    0,    0,    0,  141,    0,  149,    0,  157,    0,    6,
   92,    6,   59,    6,   35,  191,    0,    0,    0,  185,
  192,    0,  193,    0,   35,    6,   59,   79,   59,    6,
   35,    0,    0,    0,    0,    0,    0,    0,    0,  194,
   35,  185,  185,   79,   35,    0,    0,    0,   35,   35,
    0,
};
short yyrindex[] = {                                      0,
    0,   38,    0,    0,    0,    0,    0,  116,    0,    0,
    0,    0,    0,    0,  187,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,   72,    0,    0,    0,    0,    0,    0,    0,    0,
   14,    0,   34,    0,    0,   57,    0,    0,    0,    0,
    0,  105,    0,    0,    0,    0,    0,    0,    0,  132,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,  136,  142,    0,    0,    0,    0,
    0,    0,    0,  102,    0,   75,    0,    4,    0,    0,
    0,    0,    0,    0,  187,   80,    0,    0,    0,  148,
   82,    0,  168,    0,  110,    0,    0,    0,    0,    0,
  187,    0,    0,    0,    0,    0,    0,    0,    0,  173,
  120,    0,    0,    0,  187,    0,    0,    0,  126,  187,
    0,
};
short yygindex[] = {                                      0,
  112,  -73,  -11,    0,  -15,  -59,    0,    0,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
  162,    0,    0,    0,    0,  -89,    0,  109,    0,    0,
    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
    0,    0,    0,    0,    0,  -75,    0,    0,    0,   85,
};
#define YYTABLESIZE 197
short yytable[] = {                                      25,
   70,   80,   80,  102,   63,   63,   71,   63,   63,   81,
   32,   72,   73,   18,   82,   83,   96,  114,  101,  119,
  103,   18,   18,   18,   18,   18,   18,   18,   18,   33,
   34,   15,  113,   22,   18,  121,  120,    1,   22,   23,
   70,   22,   22,   22,   22,   22,   22,   22,   22,  129,
    1,    1,    1,   16,   22,   17,   26,   18,    1,   54,
   55,   20,  126,  127,   26,   26,   26,   26,   26,   26,
   26,   26,   27,   27,   56,   54,   54,   26,   57,  115,
   45,   45,   52,   52,  116,  117,   19,   27,   48,  112,
   54,   27,   54,   55,   54,   45,   50,   52,   51,   45,
   53,   52,   47,   47,    3,  112,   77,   97,   98,   64,
   92,   92,  128,  112,  131,   34,   78,   47,   47,   62,
   92,   92,   64,   64,   64,   60,   92,   92,   34,   34,
   34,   70,   62,   62,   62,   31,   79,   38,   60,   60,
   60,   33,   39,   90,   70,   70,   70,   67,   31,   31,
   31,   92,   42,   45,   33,   33,   33,   39,   39,   94,
   67,   67,   67,    3,    4,    5,    6,    7,   61,   61,
   76,   61,   61,   59,   59,  100,   59,   59,   63,   64,
   65,   26,   27,   36,   37,   54,   55,   92,   92,   86,
   88,   43,   46,  106,  108,  110,  124,
};
short yycheck[] = {                                      15,
   60,   75,   76,   93,    1,    2,    1,    4,    5,    1,
    3,    6,    7,    0,    6,    7,   90,  107,   92,  109,
   94,    8,    9,   10,   11,   12,   13,   14,   15,   22,
   23,    3,  106,    0,   21,  111,  110,    0,    4,    5,
  100,    8,    9,   10,   11,   12,   13,   14,   15,  125,
   13,   14,   15,    3,   21,    3,    0,    3,   21,    1,
    2,   21,  122,  123,    8,    9,   10,   11,   12,   13,
   14,   15,    1,    2,   16,    1,    2,   21,   20,    1,
    1,    2,    1,    2,    6,    7,    3,   16,    6,  105,
   16,   20,    1,    2,   20,   16,    3,   16,    3,   20,
    6,   20,    1,    2,    0,  121,    3,   16,   17,    0,
    1,    2,  124,  129,  130,    0,    3,   16,   17,    0,
    1,    2,   13,   14,   15,    0,    1,    2,   13,   14,
   15,    0,   13,   14,   15,    0,    3,    1,   13,   14,
   15,    0,    6,    3,   13,   14,   15,    0,   13,   14,
   15,    3,    1,    1,   13,   14,   15,    6,    6,    3,
   13,   14,   15,    8,    9,   10,   11,   12,    1,    2,
   62,    4,    5,    1,    2,   91,    4,    5,   13,   14,
   15,   18,   19,    1,    2,    1,    2,    1,    2,   78,
   79,   30,   31,    3,    3,    3,    3,
};
#define YYFINAL 1
#ifndef YYDEBUG
#define YYDEBUG 0
#endif
#define YYMAXTOKEN 258
#if YYDEBUG
char *yyname[] = {
"end-of-file","INTTOK","FLOATTOK","COLONTOK","MINUSTOK","PLUSTOK","STRINGTOK",
"ASTERICKTOK","DISCOUNTTOK","VALUESTOK","STATETOK","ACTIONTOK","OBSTOK","TTOK",
"OTOK","RTOK","UNIFORMTOK","IDENTITYTOK","REWARDTOK","COSTTOK","RESETTOK",
"STARTTOK","INCLUDETOK","EXCLUDETOK",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,"EOFTOK",
};
char *yyrule[] = {
"$accept : pomdp_file",
"$$1 :",
"$$2 :",
"pomdp_file : preamble $$1 start_state $$2 param_list",
"preamble : preamble param_type",
"preamble :",
"param_type : discount_param",
"param_type : value_param",
"param_type : state_param",
"param_type : action_param",
"param_type : obs_param",
"discount_param : DISCOUNTTOK COLONTOK number",
"value_param : VALUESTOK COLONTOK value_tail",
"value_tail : REWARDTOK",
"value_tail : COSTTOK",
"$$3 :",
"state_param : STATETOK COLONTOK $$3 state_tail",
"state_tail : INTTOK",
"state_tail : ident_list",
"$$4 :",
"action_param : ACTIONTOK COLONTOK $$4 action_tail",
"action_tail : INTTOK",
"action_tail : ident_list",
"$$5 :",
"obs_param : OBSTOK COLONTOK $$5 obs_param_tail",
"obs_param_tail : INTTOK",
"obs_param_tail : ident_list",
"$$6 :",
"start_state : STARTTOK COLONTOK $$6 u_matrix",
"start_state : STARTTOK COLONTOK STRINGTOK",
"$$7 :",
"start_state : STARTTOK INCLUDETOK COLONTOK $$7 start_state_list",
"$$8 :",
"start_state : STARTTOK EXCLUDETOK COLONTOK $$8 start_state_list",
"start_state :",
"start_state_list : start_state_list state",
"start_state_list : state",
"param_list : param_list param_spec",
"param_list :",
"param_spec : trans_prob_spec",
"param_spec : obs_prob_spec",
"param_spec : reward_spec",
"trans_prob_spec : TTOK COLONTOK trans_spec_tail",
"$$9 :",
"trans_spec_tail : action COLONTOK state COLONTOK state $$9 prob",
"$$10 :",
"trans_spec_tail : action COLONTOK state $$10 u_matrix",
"$$11 :",
"trans_spec_tail : action $$11 ui_matrix",
"obs_prob_spec : OTOK COLONTOK obs_spec_tail",
"$$12 :",
"obs_spec_tail : action COLONTOK state COLONTOK obs $$12 prob",
"$$13 :",
"obs_spec_tail : action COLONTOK state $$13 u_matrix",
"$$14 :",
"obs_spec_tail : action $$14 u_matrix",
"reward_spec : RTOK COLONTOK reward_spec_tail",
"$$15 :",
"reward_spec_tail : action COLONTOK state COLONTOK state COLONTOK obs $$15 number",
"$$16 :",
"reward_spec_tail : action COLONTOK state COLONTOK state $$16 num_matrix",
"$$17 :",
"reward_spec_tail : action COLONTOK state $$17 num_matrix",
"$$18 :",
"reward_spec_tail : action $$18 num_matrix",
"ui_matrix : UNIFORMTOK",
"ui_matrix : IDENTITYTOK",
"ui_matrix : prob_matrix",
"u_matrix : UNIFORMTOK",
"u_matrix : RESETTOK",
"u_matrix : prob_matrix",
"prob_matrix : prob_matrix prob",
"prob_matrix : prob",
"num_matrix : num_matrix number",
"num_matrix : number",
"state : INTTOK",
"state : STRINGTOK",
"state : ASTERICKTOK",
"action : INTTOK",
"action : STRINGTOK",
"action : ASTERICKTOK",
"obs : INTTOK",
"obs : STRINGTOK",
"obs : ASTERICKTOK",
"ident_list : ident_list STRINGTOK",
"ident_list : STRINGTOK",
"prob : INTTOK",
"prob : FLOATTOK",
"number : optional_sign INTTOK",
"number : optional_sign FLOATTOK",
"optional_sign : PLUSTOK",
"optional_sign : MINUSTOK",
"optional_sign :",
};
#endif
#define yyclearin (yychar=(-1))
#define yyerrok (yyerrflag=0)
#ifdef YYSTACKSIZE
#ifndef YYMAXDEPTH
#define YYMAXDEPTH YYSTACKSIZE
#endif
#else
#ifdef YYMAXDEPTH
#define YYSTACKSIZE YYMAXDEPTH
#else
#define YYSTACKSIZE 500
#define YYMAXDEPTH 500
#endif
#endif
int yydebug;
int yynerrs;
int yyerrflag;
int yychar;
short *yyssp;
YYSTYPE *yyvsp;
YYSTYPE yyval;
YYSTYPE yylval;
short yyss[YYSTACKSIZE];
YYSTYPE yyvs[YYSTACKSIZE];
#define yystacksize YYSTACKSIZE
#line 759 "parser.y"

/********************************************************************/
/*              External Routines                                   */
/********************************************************************/

#define EPSILON  0.00001  /* tolerance for sum of probs == 1 */

Constant_Block *aConst;

/******************************************************************************/
void 
yyerror(char *string)
{
   ERR_enter("Parser<yyparse>", currentLineNumber, PARSE_ERR,"");
}  /* yyerror */
/******************************************************************************/
void 
checkMatrix() {
/* When a matrix is finished being read for the exactly correct number of
   values, curRow should be 0 and curCol should be -1.  For the cases
   where we are only interested in a row of entries curCol should be -1.
   If we get too many entries, then we will catch this as we parse the 
   extra entries.  Therefore, here we only need to check for too few 
   entries.
   */

   switch( curMatrixContext ) {
   case mc_trans_row:
      if( curCol < gNumStates )
         ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
                   TOO_FEW_ENTRIES, "");
      break;
   case mc_trans_all:
      if((curRow < (gNumStates-1) )
	 || ((curRow == (gNumStates-1))
	     && ( curCol < gNumStates ))) 
	ERR_enter("Parser<checkMatrix>:", currentLineNumber,  
                   TOO_FEW_ENTRIES, "" );
      break;
   case mc_obs_row:
      if( curCol < gNumObservations )
         ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
                   TOO_FEW_ENTRIES, "");
      break;
   case mc_obs_all:
      if((curRow < (gNumStates-1) )
	 || ((curRow == (gNumStates-1))
	     && ( curCol < gNumObservations ))) 
         ERR_enter("Parser<checkMatrix>:", currentLineNumber,  
                   TOO_FEW_ENTRIES, "" );
      break;
   case mc_start_belief:
      if( curCol < gNumStates )
	ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
		  TOO_FEW_ENTRIES, "");
      break;

    case mc_mdp_start:
      /* We will check for invalid multiple entries for MDP in 
	 enterMatrix() */
      break;

    case mc_reward_row:
      if( gProblemType == POMDP_problem_type )
	if( curCol < gNumObservations )
	  ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
		    TOO_FEW_ENTRIES, "");
      break;

    case mc_reward_all:
      if( gProblemType == POMDP_problem_type ) {
	if((curRow < (gNumStates-1) )
	   || ((curRow == (gNumStates-1))
	       && ( curCol < gNumObservations ))) 
	  ERR_enter("Parser<checkMatrix>:", currentLineNumber,  
		    TOO_FEW_ENTRIES, "" );
      }
      else
	if( curCol < gNumStates )
	  ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
		    TOO_FEW_ENTRIES, "");
      
      break;
    case mc_reward_single:
      /* Don't need to do anything */
      break;

    case mc_reward_mdp_only:
      if((curRow < (gNumStates-1) )
	 || ((curRow == (gNumStates-1))
	     && ( curCol < gNumStates ))) 
	ERR_enter("Parser<checkMatrix>:", currentLineNumber,  
		  TOO_FEW_ENTRIES, "" );
      break;

   default:
      ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
                BAD_MATRIX_CONTEXT, "" );
      break;
   }  /* switch */

   if( gTooManyEntries )
     ERR_enter("Parser<checkMatrix>:", currentLineNumber, 
	       TOO_MANY_ENTRIES, "" );

   /* After reading a line for immediate rewards for a pomdp, we must tell
      the data structures for the special representation that we are done */
   switch( curMatrixContext ) {
   case mc_reward_row:
   case mc_reward_all:
   case mc_reward_mdp_only:
     doneImmReward();
     break;

     /* This case is only valid for POMDPs, so if we have an MDP, we
	never would have started a new immediate reward, so calling 
	the doneImmReward will be in error.  */
   case mc_reward_single:
     if( gProblemType == POMDP_problem_type )
       doneImmReward();
     break;
   default:
     break;
   }  /* switch */
   

   curMatrixContext = mc_none;  /* reset this as a safety precaution */
}  /* checkMatrix */
/******************************************************************************/
void 
enterString( Constant_Block *block ) {
   
   if( H_enter( block->theValue.theString, curMnemonic ) == 0 )
      ERR_enter("Parser<enterString>:", currentLineNumber, 
                DUPLICATE_STRING, block->theValue.theString );

   XFREE( block->theValue.theString );
   XFREE( block );
}  /* enterString */
/******************************************************************************/
void 
enterUniformMatrix( ) {
   int a, i, j, obs;
   double prob;

   switch( curMatrixContext ) {
   case mc_trans_row:
      prob = 1.0/gNumStates;
      for( a = minA; a <= maxA; a++ )
         for( i = minI; i <= maxI; i++ )
            for( j = 0; j < gNumStates; j++ )
	       addEntryToIMatrix( IP[a], i, j, prob );
      break;
   case mc_trans_all:
      prob = 1.0/gNumStates;
      for( a = minA; a <= maxA; a++ )
         for( i = 0; i < gNumStates; i++ )
            for( j = 0; j < gNumStates; j++ )
 	       addEntryToIMatrix( IP[a], i, j, prob );
      break;
   case mc_obs_row:
      prob = 1.0/gNumObservations;
      for( a = minA; a <= maxA; a++ )
         for( j = minJ; j <= maxJ; j++ )
            for( obs = 0; obs < gNumObservations; obs++ )
 	       addEntryToIMatrix( IR[a], j, obs, prob );
      break;
   case mc_obs_all:
      prob = 1.0/gNumObservations;
      for( a = minA; a <= maxA; a++ )
         for( j = 0; j < gNumStates; j++ )
            for( obs = 0; obs < gNumObservations; obs++ )
 	       addEntryToIMatrix( IR[a], j, obs, prob );
      break;
   case mc_start_belief:
      setStartStateUniform();
      break;
   case mc_mdp_start:
      /* This is meaning less for an MDP */
      ERR_enter("Parser<enterUniformMatrix>:", currentLineNumber, 
                BAD_START_STATE_TYPE, "" );
      break;
   default:
      ERR_enter("Parser<enterUniformMatrix>:", currentLineNumber, 
                BAD_MATRIX_CONTEXT, "" );
      break;
   }  /* switch */
}  /* enterUniformMatrix */
/******************************************************************************/
void 
enterIdentityMatrix( ) {
   int a, i,j;

   switch( curMatrixContext ) {
   case mc_trans_all:
      for( a = minA; a <= maxA; a++ )
         for( i = 0; i < gNumStates; i++ )
            for( j = 0; j < gNumStates; j++ )
               if( i == j )
		 addEntryToIMatrix( IP[a], i, j, 1.0 );
               else
		 addEntryToIMatrix( IP[a], i, j, 0.0 );
      break;
   default:
      ERR_enter("Parser<enterIdentityMatrix>:", currentLineNumber, 
                BAD_MATRIX_CONTEXT, "" );
      break;
   }  /* switch */
}  /* enterIdentityMatrix */
/******************************************************************************/
void 
enterResetMatrix( ) {
  int a, i, j;

  if( curMatrixContext != mc_trans_row ) {
    ERR_enter("Parser<enterMatrix>:", currentLineNumber, 
	      BAD_RESET_USAGE, "" );
    return;
  }

  if( gProblemType == POMDP_problem_type )
    for( a = minA; a <= maxA; a++ )
      for( i = minI; i <= maxI; i++ )
	for( j = 0; j < gNumStates; j++ )
	  addEntryToIMatrix( IP[a], i, j, gInitialBelief[j] );
  
  else  /* It is an MDP */
    for( a = minA; a <= maxA; a++ )
      for( i = minI; i <= maxI; i++ )
	addEntryToIMatrix( IP[a], i, gInitialState, 1.0 );
  

}  /* enterResetMatrix */
/******************************************************************************/
void 
enterMatrix( double value ) {
/*
  For the '_single' context types we never have to worry about setting or 
  checking the bounds on the current row or col.  For all other we do and
  how this is done depends on the context.  Notice that we are filling in the 
  elements in reverse order due to the left-recursive grammar.  Thus
  we need to update the col and row backwards 
  */
   int a, i, j, obs;

   switch( curMatrixContext ) {
   case mc_trans_single:
      for( a = minA; a <= maxA; a++ )
         for( i = minI; i <= maxI; i++ )
            for( j = minJ; j <= maxJ; j++ )
	      addEntryToIMatrix( IP[a], i, j, value );
      break;
   case mc_trans_row:
      if( curCol < gNumStates ) {
         for( a = minA; a <= maxA; a++ )
            for( i = minI; i <= maxI; i++ )
	      addEntryToIMatrix( IP[a], i, curCol, value );
         curCol++;
      }
      else
	gTooManyEntries = 1;

      break;
   case mc_trans_all:
      if( curCol >= gNumStates ) {
         curRow++;
         curCol = 0;;
      }

      if( curRow < gNumStates ) {
         for( a = minA; a <= maxA; a++ )
	   addEntryToIMatrix( IP[a], curRow, curCol, value );
         curCol++;
      }
      else
	gTooManyEntries = 1;

      break;

   case mc_obs_single:

      if( gProblemType == POMDP_problem_type )
	/* We ignore this if it is an MDP */

	for( a = minA; a <= maxA; a++ )
	  for( j = minJ; j <= maxJ; j++ )
            for( obs = minObs; obs <= maxObs; obs++ )
	      addEntryToIMatrix( IR[a], j, obs, value );
      break;

   case mc_obs_row:
      if( gProblemType == POMDP_problem_type )
	/* We ignore this if it is an MDP */

	if( curCol < gNumObservations ) {

	  for( a = minA; a <= maxA; a++ )
            for( j = minJ; j <= maxJ; j++ )
	      addEntryToIMatrix( IR[a], j, curCol, value );
	  
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      break;

   case mc_obs_all:
      if( curCol >= gNumObservations ) {
         curRow++;
         curCol = 0;
      }

      if( gProblemType == POMDP_problem_type )
	/* We ignore this if it is an MDP */

	if( curRow < gNumStates ) {
	  for( a = minA; a <= maxA; a++ )
	    addEntryToIMatrix( IR[a], curRow, curCol, value );
	  
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      break;

/* This is a special case for POMDPs, since we need a special 
   representation for immediate rewards for POMDP's.  Note that this 
   is not valid syntax for an MDP, but we flag this error when we set 
   the matrix context, so we ignore the MDP case here.
   */
   case mc_reward_single:
      if( gProblemType == POMDP_problem_type ) {

	if( curCol == 0 ) {
	  enterImmReward( 0, 0, 0, value );
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      }
     break;

    case mc_reward_row:
      if( gProblemType == POMDP_problem_type ) {

	/* This is a special case for POMDPs, since we need a special 
	   representation for immediate rewards for POMDP's */
   
	if( curCol < gNumObservations ) {
	  enterImmReward( 0, 0, curCol, value );
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      }  /* if POMDP problem */

      else /* we are dealing with an MDP, so there should only be 
	      a single entry */
	if( curCol == 0 ) {
	  enterImmReward( 0, 0, 0, value );
	  curCol++;
	}
	else
	  gTooManyEntries = 1;


     break;

   case mc_reward_all:

      /* This is a special case for POMDPs, since we need a special 
	 representation for immediate rewards for POMDP's */

      if( gProblemType == POMDP_problem_type ) {
	if( curCol >= gNumObservations ) {
	  curRow++;
	  curCol = 0;
	}
	if( curRow < gNumStates ) {
	  enterImmReward( 0, curRow, curCol, value );
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      }  /* If POMDP problem */

      /* Otherwise it is an MDP and we should be expecting an entire
	 row of rewards. */

      else  /* MDP */
	if( curCol < gNumStates ) {
	  enterImmReward( 0, curCol, 0, value );
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      break;

      /* This is a special case for an MDP only where we specify
	 the entire matrix of rewards. If we are erroneously 
	 definining a POMDP, this error will be flagged in the 
	 setMatrixContext() routine.
	 */

    case mc_reward_mdp_only:
      if( gProblemType == MDP_problem_type ) {
	if( curCol >= gNumStates ) {
	  curRow++;
	  curCol = 0;
	}
	if( curRow < gNumStates ) {
	  enterImmReward( curRow, curCol, 0, value );
	  curCol++;
	}
	else
	  gTooManyEntries = 1;

      }
      break;

    case mc_mdp_start:

      /* For an MDP we only want to see a single value and */
      /* we want it to correspond to a valid state number. */

      if( curCol > 0 )
	gTooManyEntries = 1;

      else {
	gInitialState = value;
	curCol++;
      }
      break;
	  
   case mc_start_belief:

      /* This will process the individual entries when a starting */
      /* belief state is fully specified.  When it is a POMDP, we need */
      /* an entry for each state, so we keep the curCol variable */
      /* updated.  */

      if( curCol < gNumStates ) {
	gInitialBelief[curCol] = value;
	curCol++;
      }
      else
	gTooManyEntries = 1;

      break;

   default:
      ERR_enter("Parser<enterMatrix>:", currentLineNumber, 
                BAD_MATRIX_CONTEXT, "");
      break;
   }  /* switch */

}  /* enterMatrix */
/******************************************************************************/
void 
setMatrixContext( Matrix_Context context, 
		  int a, int i, int j, int obs ) {
/* 
   Note that we must enter the matrix entries in reverse order because
   the matrices are defined with left-recursive rules.  Set the a, i,
   and j parameters to be less than zero when you want to define it
   for all possible values.  

   Rewards for MDPs and POMDPs differ since in the former, rewards are not
   based upon an observations.  This complicates things since not only is one 
   of the reward syntax options not valid, but the semantics of all the
   rewards change as well.  I have chosen to handle this in this routine.  
   I will check for the appropriate type and set the context to handle the
   proper amount of entries.
*/
  int state;

   curMatrixContext = context;
   gTooManyEntries = 0;  /* Clear this out before reading any */

   curRow = 0;  /* This is ignored for some contexts */
   curCol = 0;

   switch( curMatrixContext ) {

   mc_start_belief:
     
     break;

   case mc_start_include:

     /* When we specify the starting belief state as a list of states */
     /* to include, we initialize all state to 0.0, since as we read */
     /* the states we will set that particular value to 1.0.  After it */
     /* is all done we can then just normalize the belief state */

     if( gProblemType == POMDP_problem_type )
       for( state = 0; state < gNumStates; state++ )
	 gInitialBelief[state] = 0.0;

     else  /* It is an MDP which is not valid */
       ERR_enter("Parser<setMatrixContext>:", currentLineNumber, 
		 BAD_START_STATE_TYPE, "");
      
     break;

   case mc_start_exclude:

     /* When we are specifying the starting belief state as a a list */
     /* of states, we initialize all states to 1.0 and as we read each */
     /* in the list we clear it out to be zero.  fter it */
     /* is all done we can then just normalize the belief state */

     if( gProblemType == POMDP_problem_type )
       for( state = 0; state < gNumStates; state++ )
	 gInitialBelief[state] = 1.0;

     else  /* It is an MDP which is not valid */
       ERR_enter("Parser<setMatrixContext>:", currentLineNumber, 
		 BAD_START_STATE_TYPE, "");

     break;

  /* We need a special representation for the immediate rewards.
     These four cases initialize the data structure that will be
     needed for immediate rewards by calling newImmReward.  Note that
     the arguments will differe depending upon whether it is an
     MDP or POMDP.
     */
  case mc_reward_mdp_only:
    if( gProblemType == POMDP_problem_type )  {
       ERR_enter("Parser<setMatrixContext>:", currentLineNumber, 
		 BAD_REWARD_SYNTAX, "");
    }
    else {
      newImmReward( a, NOT_PRESENT, NOT_PRESENT, 0 );
    } 
    break;
 
  case mc_reward_all:	
    if( gProblemType == POMDP_problem_type ) 
      newImmReward( a, i, NOT_PRESENT, NOT_PRESENT );

    else {
      newImmReward( a, i, NOT_PRESENT, 0 );
    }
    break;
  case mc_reward_row:
    if( gProblemType == POMDP_problem_type ) 
      newImmReward( a, i, j, NOT_PRESENT );
    
    else {
      newImmReward( a, i, j, 0 );
    } 
    break;
  case mc_reward_single:

    if( gProblemType == MDP_problem_type ) {
       ERR_enter("Parser<setMatrixContext>:", currentLineNumber, 
		 BAD_REWARD_SYNTAX, "");
    }
    else {
       newImmReward( a, i, j, obs );
     }
    break;

   default:
     break;
   }

  /* These variable settings will define the range over which the current 
     matrix context will have effect.  This accounts for wildcards by
     setting the range to include everything.  When a single entry was
     specified, the range is that single number.  When we actually 
     start to read the matrix, each entry we see will apply for the
     entire range specified, though for specific entries the range 
     will be a single number.
     */
   if( a < 0 ) {
      minA = 0;
      maxA = gNumActions - 1;
   }
   else
      minA = maxA = a;

   if( i < 0 ) {
      minI = 0;
      maxI = gNumStates - 1;
   }
   else
      minI = maxI = i;

   if( j < 0 ) {
      minJ = 0;
      maxJ = gNumStates - 1;
   }
   else
      minJ = maxJ = j;

   if( obs < 0 ) {
      minObs = 0;
      maxObs = gNumObservations - 1;
   }
   else
      minObs = maxObs = obs;

}  /* setMatrixContext */
/******************************************************************************/
void 
enterStartState( int i ) {
/*
   This is not valid for an MDP, but the error has already been flagged
   in the setMatrixContext() routine.  Therefore, if just igore this if 
   it is an MDP.
*/

  if( gProblemType == MDP_problem_type )
    return;

  switch( curMatrixContext ) {
  case mc_start_include:
    gInitialBelief[i] = 1.0;
    break;
  case mc_start_exclude:
    gInitialBelief[i] = 0.0;
    break;
  default:
    ERR_enter("Parser<enterStartState>:", currentLineNumber, 
	      BAD_MATRIX_CONTEXT, "");
      break;
  } /* switch */
}  /* enterStartState */
/******************************************************************************/
void 
setStartStateUniform() {
  int i;
  double prob;

  if( gProblemType != POMDP_problem_type )
    return;

  prob = 1.0/gNumStates;
  for( i = 0; i < gNumStates; i++ )
    gInitialBelief[i] = prob;

}  /*  setStartStateUniform*/
/******************************************************************************/
void 
endStartStates() {
/*
   There are a few cases where the matrix context will not be
   set at this point.  When there is a list of probabilities
   or if it is an MDP the context will have been cleared.
   */
  int i;
  double prob;

  if( gProblemType == MDP_problem_type ) {
    curMatrixContext = mc_none;  /* just to be sure */
    return;
  }
    
  switch( curMatrixContext ) {
  case mc_start_include:
  case mc_start_exclude:
    /* At this point gInitialBelief should be a vector of 1.0's and 0.0's
       being set as each is either included or excluded.  Now we need to
       normalized them to make it a true probability distribution */
    prob = 0.0;
    for( i = 0; i < gNumStates; i++ )
      prob += gInitialBelief[i];
    if( prob <= 0.0 ) {
      ERR_enter("Parser<endStartStates>:", currentLineNumber, 
                BAD_START_PROB_SUM, "" );
      return;
    }
    for( i = 0; i < gNumStates; i++ )
      gInitialBelief[i] /= prob;
    break;

  default:  /* Make sure we have a valid prob. distribution */
    prob = 0.0;
    for( i = 0; i < gNumStates; i++ ) 
      prob += gInitialBelief[i];
    if((prob < ( 1.0 - EPSILON)) || (prob > (1.0 + EPSILON))) {
      ERR_enter("Parser<endStartStates>:", NO_LINE, 
		BAD_START_PROB_SUM, "" );
    }
    break;
  }  /* switch */

  curMatrixContext = mc_none;

}  /* endStartStates */
/******************************************************************************/
void 
verifyPreamble() {
/* 
   When a param is not defined, set these to non-zero so parsing can
   proceed even in the absence of specifying these values.  When an
   out of range value is encountered the parser will flag the error,
   but return 0 so that more errors can be detected 
   */

   if( discountDefined == 0 )
      ERR_enter("Parser<verifyPreamble>:", currentLineNumber, 
                MISSING_DISCOUNT, "" );
   if( valuesDefined == 0 )
      ERR_enter("Parser<verifyPreamble>:", currentLineNumber,
                MISSING_VALUES, "" );
   if( statesDefined == 0 ) {
      ERR_enter("Parser<verifyPreamble>:", currentLineNumber, 
                MISSING_STATES, "" );
      gNumStates = 1;
   }
   if( actionsDefined == 0 ) {
      ERR_enter("Parser<verifyPreamble>:", currentLineNumber, 
                MISSING_ACTIONS, "" );
      gNumActions = 1;
   }

   /* If we do not see this, them we must be parsing an MDP */
   if( observationsDefined == 0 ) {
     gNumObservations = 0;
     gProblemType = MDP_problem_type;
   }

   else
     gProblemType = POMDP_problem_type;

}  /* verifyPreamble */
/******************************************************************************/
void 
checkProbs() {
   int a,i,j,obs;
   double sum;
   char str[40];

   
   for( a = 0; a < gNumActions; a++ )
      for( i = 0; i < gNumStates; i++ ) {
	 sum = sumIMatrixRowValues( IP[a], i );
         if((sum < ( 1.0 - EPSILON)) || (sum > (1.0 + EPSILON))) {
            sprintf( str, "action=%d, state=%d (%.5lf)", a, i, sum );
            ERR_enter("Parser<checkProbs>:", NO_LINE, 
                      BAD_TRANS_PROB_SUM, str );
         }
      } /* for i */

   if( gProblemType == POMDP_problem_type )
     for( a = 0; a < gNumActions; a++ )
       for( j = 0; j < gNumStates; j++ ) {
	 sum = sumIMatrixRowValues( IR[a], j );
         if((sum < ( 1.0 - EPSILON)) || (sum > (1.0 + EPSILON))) {
	   sprintf( str, "action=%d, state=%d (%.5lf)", a, j, sum );
	   ERR_enter("Parser<checkProbs>:", NO_LINE, 
		     BAD_OBS_PROB_SUM, str );
         } /* if sum not == 1 */
       }  /* for j */

   /* Now see if we had observation specs defined in an MDP */

   if( observationSpecDefined && (gProblemType == MDP_problem_type))
     ERR_enter("Parser<checkProbs>:", NO_LINE, 
	       OBS_IN_MDP_PROBLEM, "" );

}  /* checkProbs */
/************************************************************************/
void 
initParser() {
/*
   This routine will reset all the state variables used by the parser
   in case it will parse multiple files.
*/
   observationSpecDefined = 0;
   discountDefined = 0;
   valuesDefined = 0;
   statesDefined = 0;
   actionsDefined = 0;
   observationsDefined = 0;
   observationSpecDefined = 0;
   currentLineNumber = 1;
   curMnemonic = nt_unknown;
   curMatrixContext = mc_none;

}  /* initParser */
/************************************************************************/
int 
readMDPFile( FILE *file ) {
   int returnValue, dump_status;
   extern FILE *yyin;

   initParser();

   ERR_initialize();
   H_create();
   yyin = file;

   returnValue = yyparse();

   /* If there are syntax errors, then we have to do something if we 
      want to parse another file without restarting.  It seems that
      a syntax error bombs the code out, but leaves the file pointer
      at the place it bombed.  Thus, another call to yyparse() will
      pick up where it left off and not necessarily at the start of a 
      new file.

      Unfortunately, I do not know how to do this yet.
      */
   if (returnValue != 0) {
      printf("\nParameter file contains syntax errors!\n");
    }

   dump_status = ERR_dump();

   ERR_cleanUp();
   H_destroy();

   if (dump_status || returnValue ) 
      return( 0 );

   /* This is where intermediate matrix representation are
      converted into their final representation */
   convertMatrices();

   return( 1 );
}  /* readPomdpFile */
/************************************************************************/
int 
yywrap()
{
   return 1;
}
/************************************************************************/
#line 1267 "parser.c"
#define YYABORT goto yyabort
#define YYACCEPT goto yyaccept
#define YYERROR goto yyerrlab
int
yyparse()
{
    register int yym, yyn, yystate;
#if YYDEBUG
    register char *yys;
    extern char *getenv();

    if (yys = getenv("YYDEBUG"))
    {
        yyn = *yys;
        if (yyn >= '0' && yyn <= '9')
            yydebug = yyn - '0';
    }
#endif

    yynerrs = 0;
    yyerrflag = 0;
    yychar = (-1);

    yyssp = yyss;
    yyvsp = yyvs;
    *yyssp = yystate = 0;

yyloop:
    if (yyn = yydefred[yystate]) goto yyreduce;
    if (yychar < 0)
    {
        if ((yychar = yylex()) < 0) yychar = 0;
#if YYDEBUG
        if (yydebug)
        {
            yys = 0;
            if (yychar <= YYMAXTOKEN) yys = yyname[yychar];
            if (!yys) yys = "illegal-symbol";
            printf("yydebug: state %d, reading %d (%s)\n", yystate,
                    yychar, yys);
        }
#endif
    }
    if ((yyn = yysindex[yystate]) && (yyn += yychar) >= 0 &&
            yyn <= YYTABLESIZE && yycheck[yyn] == yychar)
    {
#if YYDEBUG
        if (yydebug)
            printf("yydebug: state %d, shifting to state %d (%s)\n",
                    yystate, yytable[yyn],yyrule[yyn]);
#endif
        if (yyssp >= yyss + yystacksize - 1)
        {
            goto yyoverflow;
        }
        *++yyssp = yystate = yytable[yyn];
        *++yyvsp = yylval;
        yychar = (-1);
        if (yyerrflag > 0)  --yyerrflag;
        goto yyloop;
    }
    if ((yyn = yyrindex[yystate]) && (yyn += yychar) >= 0 &&
            yyn <= YYTABLESIZE && yycheck[yyn] == yychar)
    {
        yyn = yytable[yyn];
        goto yyreduce;
    }
    if (yyerrflag) goto yyinrecovery;
#ifdef lint
    goto yynewerror;
#endif
yynewerror:
    yyerror("syntax error");
#ifdef lint
    goto yyerrlab;
#endif
yyerrlab:
    ++yynerrs;
yyinrecovery:
    if (yyerrflag < 3)
    {
        yyerrflag = 3;
        for (;;)
        {
            if ((yyn = yysindex[*yyssp]) && (yyn += YYERRCODE) >= 0 &&
                    yyn <= YYTABLESIZE && yycheck[yyn] == YYERRCODE)
            {
#if YYDEBUG
                if (yydebug)
                    printf("yydebug: state %d, error recovery shifting\
 to state %d\n", *yyssp, yytable[yyn]);
#endif
                if (yyssp >= yyss + yystacksize - 1)
                {
                    goto yyoverflow;
                }
                *++yyssp = yystate = yytable[yyn];
                *++yyvsp = yylval;
                goto yyloop;
            }
            else
            {
#if YYDEBUG
                if (yydebug)
                    printf("yydebug: error recovery discarding state %d\n",
                            *yyssp);
#endif
                if (yyssp <= yyss) goto yyabort;
                --yyssp;
                --yyvsp;
            }
        }
    }
    else
    {
        if (yychar == 0) goto yyabort;
#if YYDEBUG
        if (yydebug)
        {
            yys = 0;
            if (yychar <= YYMAXTOKEN) yys = yyname[yychar];
            if (!yys) yys = "illegal-symbol";
            printf("yydebug: state %d, error recovery discards token %d (%s)\n",
                    yystate, yychar, yys);
        }
#endif
        yychar = (-1);
        goto yyloop;
    }
yyreduce:
#if YYDEBUG
    if (yydebug)
        printf("yydebug: state %d, reducing by rule %d (%s)\n",
                yystate, yyn, yyrule[yyn]);
#endif
    yym = yylen[yyn];
    yyval = yyvsp[1-yym];
    switch (yyn)
    {
case 1:
#line 144 "parser.y"
{ 
		    /* The preamble is a section of the file which */
		    /* must come first and whcih contains some global */
		    /* properties of the MDP that the file */
		    /* specifies. (e.g., number of states).  The */
		    /* observations are optional and its presence or */
		    /* absence is what first tells the parser whether */
		    /* it is parsing an MDP or a POMDP. */

		    verifyPreamble();  /* make sure all things are */
				       /* defined */

		    /* While we parse we use an intermediate */
		    /* representation which will be converted to the */
		    /* sparse representation when we are finished */
		    /* parsing.  After the preamble we are ready to */
		    /* start filling in values and we know how big the */
		    /* problem is, so we allocate the space for the */
		    /* intermediate forms */

		    allocateIntermediateMDP();  
		  }
break;
case 2:
#line 168 "parser.y"
{ 
		    /* Some type of algorithms want a place to start */
		    /* off the problem, especially when doing */
		    /* simulation type experiments.  This is an */
		    /* optional argument that allows specification of */
		    /* this.   In a POMDP this is a belief state, but */
		    /* in an MDP this is a single state.  If none is */
		    /* specified for a POMDP, then the uniform */
		    /* distribution over all states is used.  If none */
		    /* is specified for an MDP, then random states */
		    /* will be assumed. */

		    endStartStates(); 
		  }
break;
case 3:
#line 187 "parser.y"
{
		    /* This is the very last thing we do while */
		    /* parsing.  Even though the file may conform to */
		    /* the syntax, the semantics of the problem */
		    /* specification requires probability */
		    /* distributions.  This routine will make sure */
		    /* that the appropriate things sum to 1.0 to make */
		    /* a valid probability distribution. This will */
		    /* also generate the error message when */
		    /* observation probabilities are specified in an */
		    /* MDP problem, since this is illegal. */

                     checkProbs();
		     YACCtrace("pomdp_file -> preamble params\n");
                  }
break;
case 4:
#line 204 "parser.y"
{
		   YACCtrace("preamble -> preamble param_type\n");
		}
break;
case 11:
#line 216 "parser.y"
{
		  /* The discount factor only makes sense when in the */
		  /* range 0 to 1, so it is an error to specify */
		  /* anything outside this range. */

                   gDiscount = yyvsp[0].f_num;
                   if(( gDiscount < 0.0 ) || ( gDiscount > 1.0 ))
                      ERR_enter("Parser<ytab>:", currentLineNumber,
                                BAD_DISCOUNT_VAL, "");
                   discountDefined = 1;
		   YACCtrace("discount_param -> DISCOUNTTOK COLONTOK number\n");
	        }
break;
case 12:
#line 230 "parser.y"
{
                   valuesDefined = 1;
		   YACCtrace("value_param -> VALUESTOK COLONTOK value_tail\n");
	        }
break;
case 13:
#line 243 "parser.y"
{
                   gValueType = REWARD_value_type;
		}
break;
case 14:
#line 247 "parser.y"
{
                   gValueType = COST_value_type;
		}
break;
case 15:
#line 252 "parser.y"
{ 
		  /* Since are able to enumerate the states and refer */
		  /* to them by identifiers, we will need to set the */
		  /* current state to indicate that we are parsing */
		  /* states.  This is important, since we will parse */
		  /* observatons and actions in exactly the same */
		  /* manner with the same code.  */
 
		  curMnemonic = nt_state; 

		}
break;
case 16:
#line 264 "parser.y"
{
                   statesDefined = 1;
                   curMnemonic = nt_unknown;
		   YACCtrace("state_param -> STATETOK COLONTOK state_tail\n");
		}
break;
case 17:
#line 271 "parser.y"
{

		  /*  For the number of states, we can just have a */
		  /*  number indicating how many there are, or ... */

                   gNumStates = yyvsp[0].constBlk->theValue.theInt;
                   if( gNumStates < 1 ) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_NUM_STATES, "");
                      gNumStates = 1;
                   }

 		   /* Since we use some temporary storage to hold the
		      integer as we parse, we free the memory when we
		      are done with the value */

                   XFREE( yyvsp[0].constBlk );
		}
break;
case 19:
#line 293 "parser.y"
{
		  /* See state_param for explanation of this */

		  curMnemonic = nt_action;  
		}
break;
case 20:
#line 299 "parser.y"
{
                   actionsDefined = 1;
                   curMnemonic = nt_unknown;
		   YACCtrace("action_param -> ACTIONTOK COLONTOK action_tail\n");
		}
break;
case 21:
#line 306 "parser.y"
{

		  /*  For the number of actions, we can just have a */
		  /*  number indicating how many there are, or ... */

                   gNumActions = yyvsp[0].constBlk->theValue.theInt;
                   if( gNumActions < 1 ) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_NUM_ACTIONS, "" );
                      gNumActions = 1;
                   }
		   
		   /* Since we use some temporary storage to hold the
		      integer as we parse, we free the memory when we
		      are done with the value */

                   XFREE( yyvsp[0].constBlk );
		}
break;
case 23:
#line 328 "parser.y"
{ 
		  /* See state_param for explanation of this */

		  curMnemonic = nt_observation; 
		}
break;
case 24:
#line 334 "parser.y"
{
                   observationsDefined = 1;
                   curMnemonic = nt_unknown;
		   YACCtrace("obs_param -> OBSTOK COLONTOK obs_param_tail\n");
		}
break;
case 25:
#line 341 "parser.y"
{

		  /*  For the number of observation, we can just have a */
		  /*  number indicating how many there are, or ... */

                   gNumObservations = yyvsp[0].constBlk->theValue.theInt;
                   if( gNumObservations < 1 ) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_NUM_OBS, "" );
                      gNumObservations = 1;
                   }

		   /* Since we use some temporary storage to hold the
		      integer as we parse, we free the memory when we
		      are done with the value */

                   XFREE( yyvsp[0].constBlk );
		}
break;
case 27:
#line 363 "parser.y"
{ 
		  /* There are a number of different formats for the */
		  /* start state.  This one is valid for either a */
		  /* POMDP or an MDP.  With a POMDP it will expect a */
		  /* list of probabilities, one for each state, */
		  /* representing the initial belief state.  For an */
		  /* MDP there can be only a single integer */
		  /* representing the starting state. */

		  if( gProblemType == POMDP_problem_type )
		    setMatrixContext(mc_start_belief, 0, 0, 0, 0); 
		  else
		    setMatrixContext(mc_mdp_start, 0, 0, 0, 0); 
		}
break;
case 29:
#line 396 "parser.y"
{
                   int num;

		   num = H_lookup( yyvsp[0].constBlk->theValue.theString, nt_state );
		   if(( num < 0 ) || (num >= gNumStates )) {
		     ERR_enter("Parser<ytab>:", currentLineNumber, 
					BAD_STATE_STR, yyvsp[0].constBlk->theValue.theString );
		   }
		   else {
		     if( gProblemType == MDP_problem_type )
		       gInitialState = num;
		     else
		       gInitialBelief[num] = 1.0;
		   }

		   XFREE( yyvsp[0].constBlk->theValue.theString );
		   XFREE( yyvsp[0].constBlk );
                }
break;
case 30:
#line 416 "parser.y"
{ 
		  setMatrixContext(mc_start_include, 0, 0, 0, 0); 
		}
break;
case 32:
#line 422 "parser.y"
{ 
		  setMatrixContext(mc_start_exclude, 0, 0, 0, 0); 
		}
break;
case 34:
#line 429 "parser.y"
{ 
		  setStartStateUniform(); 
		}
break;
case 35:
#line 434 "parser.y"
{
		  enterStartState( yyvsp[0].i_num );
                }
break;
case 36:
#line 438 "parser.y"
{
		  enterStartState( yyvsp[0].i_num );
                }
break;
case 40:
#line 447 "parser.y"
{
		    /* If there are observation specifications defined,
		       but no observations listed in the preamble, then
		       this is an error, since regular MDPs don't have
		       the concept of observations.  However, it could 
		       be a POMDP that was just missing the preamble 
		       part.  The way we handle this is to go ahead 
		       and parse the observation specifications, but
		       always check before we actually enter values in
		       a matrix (see the enterMatrix() routine.)  This
		       way we can determine if there are any problems 
		       with the observation specifications.  We cannot
		       add entries to the matrices since there will be
		       no memory allocated for it.  We want to
		       generate an error for this case, but don't want
		       a separate error for each observation
		       specification, so we define a variable that is
		       just a flag for whether or not any observation
		       specificiations have been defined.  After we
		       are all done parsing we will check this flag
		       and generate an error if needed.
		       */

		      observationSpecDefined = 1;
		  }
break;
case 42:
#line 475 "parser.y"
{
		   YACCtrace("trans_prob_spec -> TTOK COLONTOK trans_spec_tail\n");
		}
break;
case 43:
#line 480 "parser.y"
{ setMatrixContext(mc_trans_single, yyvsp[-4].i_num, yyvsp[-2].i_num, yyvsp[0].i_num, 0); }
break;
case 44:
#line 481 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );
		   YACCtrace("trans_spec_tail -> action COLONTOK state COLONTOK state prob \n");
		}
break;
case 45:
#line 486 "parser.y"
{ setMatrixContext(mc_trans_row, yyvsp[-2].i_num, yyvsp[0].i_num, 0, 0); }
break;
case 46:
#line 487 "parser.y"
{
		   YACCtrace("trans_spec_tail -> action COLONTOK state ui_matrix \n");
		}
break;
case 47:
#line 490 "parser.y"
{ setMatrixContext(mc_trans_all, yyvsp[0].i_num, 0, 0, 0); }
break;
case 48:
#line 491 "parser.y"
{
		   YACCtrace("trans_spec_tail -> action ui_matrix\n");
		}
break;
case 49:
#line 496 "parser.y"
{
		   YACCtrace("obs_prob_spec -> OTOK COLONTOK  obs_spec_tail\n");
		}
break;
case 50:
#line 501 "parser.y"
{ setMatrixContext(mc_obs_single, yyvsp[-4].i_num, 0, yyvsp[-2].i_num, yyvsp[0].i_num); }
break;
case 51:
#line 502 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );
		   YACCtrace("obs_spec_tail -> action COLONTOK state COLONTOK obs prob \n");
		}
break;
case 52:
#line 507 "parser.y"
{ setMatrixContext(mc_obs_row, yyvsp[-2].i_num, 0, yyvsp[0].i_num, 0); }
break;
case 53:
#line 508 "parser.y"
{
		   YACCtrace("obs_spec_tail -> action COLONTOK state COLONTOK u_matrix\n");
		}
break;
case 54:
#line 511 "parser.y"
{ setMatrixContext(mc_obs_all, yyvsp[0].i_num, 0, 0, 0); }
break;
case 55:
#line 512 "parser.y"
{
		   YACCtrace("obs_spec_tail -> action u_matrix\n");
		}
break;
case 56:
#line 517 "parser.y"
{
		   YACCtrace("reward_spec -> RTOK COLONTOK  reward_spec_tail\n");
		}
break;
case 57:
#line 524 "parser.y"
{ setMatrixContext(mc_reward_single, yyvsp[-6].i_num, yyvsp[-4].i_num, yyvsp[-2].i_num, yyvsp[0].i_num); }
break;
case 58:
#line 525 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );

		   /* Only need this for the call to doneImmReward */
		   checkMatrix();  
		   YACCtrace("reward_spec_tail -> action COLONTOK state COLONTOK state COLONTOK obs number\n");
		}
break;
case 59:
#line 533 "parser.y"
{ setMatrixContext(mc_reward_row, yyvsp[-4].i_num, yyvsp[-2].i_num, yyvsp[0].i_num, 0); }
break;
case 60:
#line 534 "parser.y"
{
                   checkMatrix();
		   YACCtrace("reward_spec_tail -> action COLONTOK state COLONTOK state num_matrix\n");
		 }
break;
case 61:
#line 539 "parser.y"
{ setMatrixContext(mc_reward_all, yyvsp[-2].i_num, yyvsp[0].i_num, 0, 0); }
break;
case 62:
#line 540 "parser.y"
{
                   checkMatrix();
		   YACCtrace("reward_spec_tail -> action COLONTOK state num_matrix\n");
		}
break;
case 63:
#line 546 "parser.y"
{ setMatrixContext(mc_reward_mdp_only, yyvsp[0].i_num, 0, 0, 0); }
break;
case 64:
#line 547 "parser.y"
{
                   checkMatrix();
		   YACCtrace("reward_spec_tail -> action num_matrix\n");
                }
break;
case 65:
#line 553 "parser.y"
{
                   enterUniformMatrix();
                }
break;
case 66:
#line 557 "parser.y"
{
                   enterIdentityMatrix();
                }
break;
case 67:
#line 561 "parser.y"
{
                   checkMatrix();
                }
break;
case 68:
#line 567 "parser.y"
{
                   enterUniformMatrix();
                }
break;
case 69:
#line 571 "parser.y"
{
		  enterResetMatrix();
		}
break;
case 70:
#line 575 "parser.y"
{
                   checkMatrix();
                }
break;
case 71:
#line 580 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );
                }
break;
case 72:
#line 584 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );
                }
break;
case 73:
#line 589 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );
                }
break;
case 74:
#line 593 "parser.y"
{
                   enterMatrix( yyvsp[0].f_num );
                }
break;
case 75:
#line 598 "parser.y"
{
                   if(( yyvsp[0].constBlk->theValue.theInt < 0 ) 
                      || (yyvsp[0].constBlk->theValue.theInt >= gNumStates )) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_STATE_VAL, "");
                      yyval.i_num = 0;
                   }
                   else
                      yyval.i_num = yyvsp[0].constBlk->theValue.theInt;
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 76:
#line 610 "parser.y"
{
                   int num;
                   num = H_lookup( yyvsp[0].constBlk->theValue.theString, nt_state );
                   if (( num < 0 ) || (num >= gNumStates )) {
				 ERR_enter("Parser<ytab>:", currentLineNumber, 
						 BAD_STATE_STR, yyvsp[0].constBlk->theValue.theString );
				 yyval.i_num = 0;
                   }
                   else
				 yyval.i_num = num;

                   XFREE( yyvsp[0].constBlk->theValue.theString );
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 77:
#line 625 "parser.y"
{
                   yyval.i_num = WILDCARD_SPEC;
                }
break;
case 78:
#line 630 "parser.y"
{
                   yyval.i_num = yyvsp[0].constBlk->theValue.theInt;
                   if(( yyvsp[0].constBlk->theValue.theInt < 0 ) 
                      || (yyvsp[0].constBlk->theValue.theInt >= gNumActions )) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_ACTION_VAL, "" );
                      yyval.i_num = 0;
                   }
                   else
                      yyval.i_num = yyvsp[0].constBlk->theValue.theInt;
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 79:
#line 643 "parser.y"
{
                   int num;
                   num = H_lookup( yyvsp[0].constBlk->theValue.theString, nt_action );
                   if(( num < 0 ) || (num >= gNumActions )) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_ACTION_STR, yyvsp[0].constBlk->theValue.theString );
                      yyval.i_num = 0;
                   }
                   else
                      yyval.i_num = num;

                   XFREE( yyvsp[0].constBlk->theValue.theString );
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 80:
#line 658 "parser.y"
{
                   yyval.i_num = WILDCARD_SPEC;
                }
break;
case 81:
#line 663 "parser.y"
{
                   if(( yyvsp[0].constBlk->theValue.theInt < 0 ) 
                      || (yyvsp[0].constBlk->theValue.theInt >= gNumObservations )) {
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_OBS_VAL, "");
                      yyval.i_num = 0;
                   }
                   else
                      yyval.i_num = yyvsp[0].constBlk->theValue.theInt;
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 82:
#line 675 "parser.y"
{
                   int num;
                   num = H_lookup( yyvsp[0].constBlk->theValue.theString, nt_observation );
                   if(( num < 0 ) || (num >= gNumObservations )) { 
                      ERR_enter("Parser<ytab>:", currentLineNumber, 
                                BAD_OBS_STR, yyvsp[0].constBlk->theValue.theString);
                      yyval.i_num = 0;
                   }
                   else
                      yyval.i_num = num;

                   XFREE( yyvsp[0].constBlk->theValue.theString );
                   XFREE( yyvsp[0].constBlk );
               }
break;
case 83:
#line 690 "parser.y"
{
                   yyval.i_num = WILDCARD_SPEC;
                }
break;
case 84:
#line 695 "parser.y"
{
                   enterString( yyvsp[0].constBlk );
                }
break;
case 85:
#line 699 "parser.y"
{
                   enterString( yyvsp[0].constBlk );
                }
break;
case 86:
#line 704 "parser.y"
{
		  yyval.f_num = yyvsp[0].constBlk->theValue.theInt;
		  if( curMatrixContext != mc_mdp_start )
		    if(( yyval.f_num < 0 ) || (yyval.f_num > 1 ))
		      ERR_enter("Parser<ytab>:", currentLineNumber, 
				BAD_PROB_VAL, "");
		  XFREE( yyvsp[0].constBlk );
		}
break;
case 87:
#line 713 "parser.y"
{
		  yyval.f_num = yyvsp[0].constBlk->theValue.theFloat;
		  if( curMatrixContext == mc_mdp_start )
		    ERR_enter("Parser<ytab>:", currentLineNumber, 
				    BAD_START_STATE_TYPE, "" );
		  else
		    if(( yyval.f_num < 0.0 ) || (yyval.f_num > 1.0 ))
			 ERR_enter("Parser<ytab>:", currentLineNumber, 
					 BAD_PROB_VAL, "" );
		  XFREE( yyvsp[0].constBlk );
		}
break;
case 88:
#line 726 "parser.y"
{
                   if( yyvsp[-1].i_num )
                      yyval.f_num = yyvsp[0].constBlk->theValue.theInt * -1.0;
                   else
                      yyval.f_num = yyvsp[0].constBlk->theValue.theInt;
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 89:
#line 734 "parser.y"
{
                   if( yyvsp[-1].i_num )
                      yyval.f_num = yyvsp[0].constBlk->theValue.theFloat * -1.0;
                   else
                      yyval.f_num = yyvsp[0].constBlk->theValue.theFloat;
                   XFREE( yyvsp[0].constBlk );
                }
break;
case 90:
#line 743 "parser.y"
{
                   yyval.i_num = 0;
                }
break;
case 91:
#line 747 "parser.y"
{
                   yyval.i_num = 1;
                }
break;
case 92:
#line 751 "parser.y"
{
                   yyval.i_num = 0;
                }
break;
#line 2094 "parser.c"
    }
    yyssp -= yym;
    yystate = *yyssp;
    yyvsp -= yym;
    yym = yylhs[yyn];
    if (yystate == 0 && yym == 0)
    {
#if YYDEBUG
        if (yydebug)
            printf("yydebug: after reduction, shifting from state 0 to\
 state %d\n", YYFINAL);
#endif
        yystate = YYFINAL;
        *++yyssp = YYFINAL;
        *++yyvsp = yyval;
        if (yychar < 0)
        {
            if ((yychar = yylex()) < 0) yychar = 0;
#if YYDEBUG
            if (yydebug)
            {
                yys = 0;
                if (yychar <= YYMAXTOKEN) yys = yyname[yychar];
                if (!yys) yys = "illegal-symbol";
                printf("yydebug: state %d, reading %d (%s)\n",
                        YYFINAL, yychar, yys);
            }
#endif
        }
        if (yychar == 0) goto yyaccept;
        goto yyloop;
    }
    if ((yyn = yygindex[yym]) && (yyn += yystate) >= 0 &&
            yyn <= YYTABLESIZE && yycheck[yyn] == yystate)
        yystate = yytable[yyn];
    else
        yystate = yydgoto[yym];
#if YYDEBUG
    if (yydebug)
        printf("yydebug: after reduction, shifting from state %d \
to state %d\n", *yyssp, yystate);
#endif
    if (yyssp >= yyss + yystacksize - 1)
    {
        goto yyoverflow;
    }
    *++yyssp = yystate;
    *++yyvsp = yyval;
    goto yyloop;
yyoverflow:
    yyerror("yacc stack overflow");
yyabort:
    return (1);
yyaccept:
    return (0);
}
