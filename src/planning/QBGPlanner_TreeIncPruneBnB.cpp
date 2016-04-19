/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */


#include "QBGPlanner_TreeIncPruneBnB.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BeliefValue.h"
#include "Belief.h"

#include "BayesianGameIdenticalPayoff.h"
#include "JointPolicyPureVector.h"

#include "Scope.h"

using namespace std;

//do we want to use earlier-found lower bounds?
#define REUSELOWERBOUNDS 0

#define DEBUG_QBGPlanner_TreeIncPruneBnB 0
#define DEBUG_QBGPlanner_TreeIncPruneBnBVerbose  0
#define DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning 0

//apparently 
//"class static objects must also be declared outside any function or class just like normal globals."
//( http://forums.devshed.com/c-programming-42/linker-errors-undefined-reference-to-static-member-data-193010.html )
//otherwise, we get problems linking the debug executables.
const Index QBGPlanner_TreeIncPruneBnB::UNSPECIFIED_ACTION;


//Default constructor
QBGPlanner_TreeIncPruneBnB::QBGPlanner_TreeIncPruneBnB(const PlanningUnitDecPOMDPDiscrete* pu) :
    MonahanBGPlanner(pu),
    _m_Gaoa(0),
    _m_pruneAfterUnion(true),
    _m_pruneAfterCrossSum(true),
    _m_useVectorCache(true)
{
}

QBGPlanner_TreeIncPruneBnB::QBGPlanner_TreeIncPruneBnB(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    MonahanBGPlanner(pu),
    _m_Gaoa(0),
    _m_pruneAfterUnion(true),
    _m_pruneAfterCrossSum(true),
    _m_useVectorCache(true)
{
}

//Destructor
QBGPlanner_TreeIncPruneBnB::~QBGPlanner_TreeIncPruneBnB()
{
    delete _m_Gaoa;
    delete _m_Gaoa_storage;
    delete _m_Gao_storage;
}

void QBGPlanner_TreeIncPruneBnB::Initialize()
{
    AlphaVectorPlanning::Initialize();

    _m_Gao_storage = //new GaobetaVectorSet(
        new boost::multi_array< std::vector<AlphaVector>, 2> (
                boost::extents  [GetPU()->GetNrJointActions()]
                                [GetPU()->GetNrJointObservations()]
                                );

    _m_Gaoa_storage = //new GaobetaVectorSet(
        new boost::multi_array< std::vector<AlphaVector>, 3> (
                boost::extents  [GetPU()->GetNrJointActions()]
                                [GetPU()->GetNrJointObservations()]
                                [GetPU()->GetNrJointActions()]
                                );
    _m_Gaoa = //new GaobetaVectorSet(
        new boost::multi_array< std::vector<AlphaVector>*, 3> (
                boost::extents  [GetPU()->GetNrJointActions()]
                                [GetPU()->GetNrJointObservations()]
                                [GetPU()->GetNrJointActions()]
                                );

    _m_maxDepth=GetPU()->GetNrJointObservations();

    //_m_vectorSetCache.clear();

    //this is now done in InitializeHeuristicsForJA...
    //we only reserve the space for the heuristics here
    //(computation of the heuristic depends on the actual backprojected pomdp vectors, which are different
    //every backup!)
    //_m_heuristic_for_depth = std::vector< std::vector<AlphaVector>* >
        //( GetPU()->GetNrJointObservations(), 0);


    MonahanPlanner::_m_initialized=true;
}

std::vector<AlphaVector>* QBGPlanner_TreeIncPruneBnB::GetGaoa(Index a,
                                          const vector<Index> &os,
                                          const vector<Index> &as) const
{
    Index aPrime=GetPU()->IndividualToJointActionIndices(as);
    Index jo=GetPU()->IndividualToJointObservationIndices(os);
    return(GetGaoa(a,jo,aPrime));
}

std::vector<AlphaVector>* QBGPlanner_TreeIncPruneBnB::GetGaoa(Index a, Index o, Index aPrime) const
{
    if((*_m_Gaoa)[a][o][aPrime]==0)
        throw(E("QBGPlanner_TreeIncPruneBnB::GetGaoa set should have been computed"));

    if(aPrime==UNSPECIFIED_ACTION)
        throw(E("QBGPlanner_TreeIncPruneBnB::GetGaoa the next time step action should be specified"));

    return((*_m_Gaoa)[a][o][aPrime]);
}

void QBGPlanner_TreeIncPruneBnB::InitializeHeuristicsForJA(Index ja)
{
    //throw away any previous heuristics...
    _m_heuristic_for_depth.clear();

    cout << "Computing heuristic..." << endl;
    _m_heuristic_for_depth.resize(_m_maxDepth);
    //use the Gao to define the heuristic sets H for the different depths
    for(Index invT=1; invT <= _m_maxDepth; invT++)
    {
        Index T = _m_maxDepth - invT; //t = h - \ttg
        cout << "starting depth " << T <<"..."; cout.flush();
        //figure out what joint observation is assigned at depth T
        Index jo = GetJointObservationsFromDepth(T);
        std::vector<AlphaVector> Gao = Prune( (*_m_Gao_storage)[ja][jo] );
        std::vector<AlphaVector> H_T;
        if(T < _m_maxDepth - 1)
        {
            //this is not the last joint observation
            const std::vector<AlphaVector>  & H_T_next = _m_heuristic_for_depth.at(T+1);
            CrossSum(Gao, H_T_next, H_T);
        }
        else
            H_T = Gao;
        _m_heuristic_for_depth.at(T) = Prune(H_T);
        size_t nrVecs = _m_heuristic_for_depth.at(T).size();
        cout << "done. (got " << nrVecs << " vectors)" << endl;
        //TODO: bail out when we cannot compute the POMDP solution.
        //_m_heuristic_mindepth = T; 
    }
}

QFunctionsDiscrete
QBGPlanner_TreeIncPruneBnB::BackupStage(const QFunctionsDiscrete &Qs, size_t maxNrAlphas)
{
    _m_nr_bottom_hits = 0;
    _m_nr_backtracks = 0;
    _m_nodes_visited = 0;
    _m_union_nodes_at_level = vector<LIndex>(_m_maxDepth, 0);
    _m_cs_nodes_at_level = vector<LIndex>(_m_maxDepth, 0);

    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Qs);



//    vector<Index> jointActionOrder=GetJointActionOrder();

    Index nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations();
    cout << "TreeIncPruneBGPlanner::BackupStage computing all Gaoa vectors"; cout.flush();

    //because we will need both the POMDP vector sets as well as the QBG vector sets, 
    //we can better inline and addapt ComputeAllGaoa here
//    GaobetaVectorSet temp_Gaoa =  ComputeAllGaoa(V);
//    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
//        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
//            for(GaoVectorSetIndex aPrime=0;aPrime!=nrA;aPrime++) //for each next joint action
//            {
//                (*_m_Gaoa_storage)[a][o][aPrime] =  VectorSetToValueFunction( *temp_Gaoa[a][o][aPrime] );
//                (*_m_Gaoa)[a][o][aPrime] =  &(*_m_Gaoa_storage)[a][o][aPrime];
//            }

    //pre-compute the back-projected POMDP vector sets
    GaoVectorSet Gao=BackProject(V);
    for(unsigned int a=0;a!=nrA;a++)
        for(unsigned int o=0;o!=nrO;o++)
        {
            VectorSet * gao = Gao[a][o];
            (*_m_Gao_storage)[a][o] = VectorSetToValueFunction( *gao );
            delete gao;
        }
    //Note we cannot prune these Gao sets, since then we do not know to which 
    //vector V[i] Gao[a][o][i] corresponds. (and we need the in order to figure
    //out what next-stage action it specifies!)
    //TODO: but we can prune them later?

    //the following allocates space for the different sets G_{a,o,\beta} (i.e., (3.15) of the QBG-techrep)
    //GaobetaVectorSet G(boost::extents[nrA][nrO][nrA]);
    //-> No: We directly add to _m_Gaoa_storage

    size_t total_number_Gaoa = 0;
    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
        {
            //instead of 
            //  for aPrime = 1:nrA
            //      for vI = 1:nrVectors
            //          if vI.GetAction = aPrime
            //we just do
            //  for vI = 1:nrVectors
            //      G[a][o][vI.GetAction].push_back( vI)

            //a pointer used to point to different vectors in Gao
            double gamma=GetPU()->GetDiscount();
            for(Index vectorI=0; vectorI!=V.size(); vectorI++) // <- for each vector in Gao[a][o]
            // ( because Gao[a][o][i] is the backprojection of V[i], this is the same as for each 
            //   vector v[vectorI] in V^t+1 )
            {
                vector<double> v_temp(GetPU()->GetNrStates());
                //add the immediate reward (which is divided equally over the vectors)
                for(Index s=0; s!=GetPU()->GetNrStates(); s++)
                {
                    double R_s_a_o = (GetPU()->GetReward(s,a)) / GetPU()->GetNrJointObservations();
                    double g_a_o_v_s =  (*_m_Gao_storage)[a][o].at(vectorI).GetValue(s);
                    v_temp.at(s) =  R_s_a_o + gamma * g_a_o_v_s;
                }
                AlphaVector v1( GetPU()->GetNrStates() );
                v1.SetValues(v_temp);
                v1.SetAction(a);
                Index aPrime = V[vectorI].GetAction(); //<- Gao does not store the next-stage actions
                (*_m_Gaoa_storage)[a][o][aPrime].push_back(v1);
            }
            //prune:
            for(Index aPrime=0; aPrime < nrA; aPrime++)
            {
#if DEBUG_QBGPlanner_TreeIncPruneBnB
                cout << "_m_Gaoa_storage[a="<<a<<"][o="<<o<<"][a'="<<aPrime<<"]  =  ";
                cout <<  SoftPrintVector( (*_m_Gaoa_storage)[a][o][aPrime] ) << endl;
#endif
                (*_m_Gaoa_storage)[a][o][aPrime] = Prune( (*_m_Gaoa_storage)[a][o][aPrime] );
                (*_m_Gaoa)[a][o][aPrime] = &((*_m_Gaoa_storage)[a][o][aPrime]);

                size_t nr = (*_m_Gaoa_storage)[a][o][aPrime].size();
#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
                cout << "_m_Gaoa_storage[a="<<a<<"][o="<<o<<"][a'="<<aPrime<<"]  has "<<
                    nr << " vectors\n";
#endif
                total_number_Gaoa += nr;
            }
        }

    cout << " - total number of back-projected gamma vectors:" << total_number_Gaoa;
    cout << "." << endl;

    AlphaVector alpha(nrS);
   
    //TODO: could save some time by passing this in as argument (by reference) 
    QFunctionsDiscrete Q(nrA);

    // Initialize a joint policy with all individual observations
    // mapping to UNSPECIFIED_ACTION
    vector<vector<Index> > jpol;
    for(Index k=0;k!=GetPU()->GetNrAgents();++k)
    {
        vector<Index> pol(GetPU()->GetNrObservations(k),UNSPECIFIED_ACTION);
        jpol.push_back(pol);
    }

    
    //
    // About the depths: we sync the 'depth index' with the index of the 
    // joint action vector that is being specified. E.g. the 
    // union node and subsequent cross-sum node that specify the joint 
    // action for the i-th entry of the joint action vector (JAV) representation
    // (of the CBG policy beta)  are at depth i.
    //
    // So we have a tree as follows:
    // node type    depth   comments
    // union        -1      the union over joint actions a at stage t. Not explicitly represented in code.
    // cross-sum    -1      the cross-sum that adds the R^a. Not explicitly represented in code 
    //                      (because R^a vectors are divided over the Gamma_a,o,a )
    // union        0       the union over joint actions a0' to be taken for the first joint observation o0.
    // cross-sum    0       the cross-sum of G_a,o1,a1 with the union at level 1.
    // ...
    // union        k-2     ('k' is _m_maxDepth in the code and corresponds to the number of joint observations)
    // cross-sum    k-2
    // union        k-1     the (degenerate) union over the only possible joint action a{k-1}'. 
    //                      In the bottom-up (memoizer) approach it just returns G_a,o{k-1},a{k-1}'.
    // cross-sum    k-1     Only exist in the top-down approach (BnB). Then it performs the last cross-sum
    //                      (with  G_a,o{k-1},a{k-1}') to create new set of (fully specified) lower-bound vectors that
    //                      are added to L.
    //
    // !!!Attention: there is a unfortunate notation clash here. 'G' is used to denote both back-projected vectors
    // (Gaoa, Ga etc.) as well as the set of vectors that induce the 'g-function' ! (it should usually be clear from
    // context)
    //

#if REUSELOWERBOUNDS
    //if we want to use lowerbounds of previous joint actions...
    //the set of non-dominated (lower bound) vectors found so far.
    std::vector<AlphaVector> L;
#endif
    // We loop over all joint actions in the current time step
    // In BnB, this can be interpreted as a union node! (at depth - 1)
    for(Index a=0; a != nrA; a++)
    {
//#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
        cout << "\n\n--------------------------------------\n";
        cout << "-proceeding with a="<<a<<"-------------------\n";
        cout << "--------------------------------------\n\n";
//#endif
        //the set of non-dominated (lower bound) vectors found so far.
        //(i.e., we do not prune using lowerbounds of previous joint actions)
#if ! REUSELOWERBOUNDS
        std::vector<AlphaVector> L;
#endif

        //Initialize the heuristics for this a:
        {
            stringstream ss;
            ss << "ComputePOMDPheur_ts" << GetTimeStep();
            _m_timing.Start(ss.str());
            InitializeHeuristicsForJA(a);
            _m_timing.Stop(ss.str());
        }
        cout << "Heuristic initialized" << endl;
        //The vectorset representing the implied g-function.
        //It is initialized to contain the null vector.
        std::vector<AlphaVector> G;
        AlphaVector null_vec(nrS, 0.0 );
        null_vec.SetAction(a);
        G.push_back(null_vec);

        // we don't start with CrossSum as the reward is already in Gaoa
        UnionNode(a,0,jpol, L, G);

        //TODO: check if this is not necessary:
        //L = Prune(L);

        //Frans 20111005: euhm...? this confuses me plenty. The set of vectors in L (Ga in memoization version)
        //correspond to all kind of different betaI. Really we would need to track the indices along with the
        //vectors...?!

        //AlphaVector::BGPolicyIndex betaI=-1; // NEED TO FIGURE OUT BETAI DOWN THE TREE SOMEHOW - TODO
        //Q[a]=VectorSetToValueFunction(L,a,betaI); //<- No, L now contains vectors for all actions.
#if !REUSELOWERBOUNDS
        Q[a] = L;
#endif
        //Q[a] = Prune(L);
        cout << SoftPrintStats();
    }
#if REUSELOWERBOUNDS
    //  On second though, I'm not sure if we can prune using lower bounds of other actions.
    //  (gives e.g., BackupStage finished, returning Q=
    //      < < a 0 bI -1 : values -4 -4 >, <  >, <  >, <  >, <  >, <  >, <  >, <  >, <  > >
    //  )

    //sort the vectors per action:
    for(Index lvecI=0; lvecI < L.size(); lvecI++)
    {
        AlphaVector lvec = L[lvecI];
        Index a =  lvec.GetAction();
        if (a > nrA)
        {
            stringstream ss; ss << "ERROR: action index=%d!?!" << a << endl;
            throw E(ss);
        }
        Q[ a ].push_back(lvec);
    }
#endif    

    //clean up:
    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
        {
            (*_m_Gao_storage)[a][o].clear();
            for(GaoVectorSetIndex aPrime=0;aPrime!=nrA;aPrime++) //for each next joint a
            {
                (*_m_Gaoa_storage)[a][o][aPrime].clear();
                (*_m_Gaoa)[a][o][aPrime] = 0; //not necessary I think, but shouldn't hurt
            }
        }

#if DEBUG_QBGPlanner_TreeIncPruneBnB
    cout << "BackupStage finished, returning Q=\n";
    for(Index a=0; a != nrA; a++)
        cout << "a=" << a << " - " << PrintTools::SoftPrintVector(Q[a]) << endl;
#endif
    cout << SoftPrintStats() << endl;
    
    _m_timing.PrintSummary();
    cout << endl;

    return(Q);
}


//Check if v is dominated by thee vectors in vSet
bool 
QBGPlanner_TreeIncPruneBnB::
CheckDomination(
                        const AlphaVector & v,
                        const std::vector<AlphaVector> & vSet
        )
{
    if(vSet.size() == 0)
        return false;
#if DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning
    cout << "CheckDomination called for v=\n"<< v <<"\n"<< "and vset=" << endl; 
    for(Index vI=0; vI < vSet.size(); vI++)
        cout << vSet[vI] << endl;
#endif

    if(CheckDominationPointWise(v, vSet))
    {
#if DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning
        cout << "Yes, PW dominated!"<< endl;
#endif
        return true;
    }
    if(CheckDominationLP(v, vSet))
    {
#if DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning
        cout << "Yes, LP dominated!"<< endl;
#endif
        return true;
    }
    return false;
}
//Check if v is dominated by thee vectors in vSet
bool 
QBGPlanner_TreeIncPruneBnB::
CheckDominationPointWise(
                        const AlphaVector & v,
                        const std::vector<AlphaVector> & vSet
        )
{
    Index nrS=GetPU()->GetNrStates();
    for(Index i=0; i < vSet.size(); i++)
    {
        const AlphaVector & w = vSet[i];
        //check if v is PW dominated by w
        bool dominated = true;
        for(Index s=0; s < nrS; s++)
            if(v.GetValue(s) > w.GetValue(s)  )
            {
                dominated = false;
                break; //v is not PW dominated by w
            }
        //now check if v was dominated by w
        if(dominated)
            return true;
    }
    //v was PW dominated by no w
    return false;
}

//Check if v is dominated by thee vectors in vSet
bool 
QBGPlanner_TreeIncPruneBnB::
CheckDominationLP(
                        const AlphaVector & v,
                        const std::vector<AlphaVector> & vSet
        )
{
    bool nondominated = CheckNotDominatedLP_POMDPSolve(v, vSet); 
    return (!nondominated);
    //Alternatively we may try the LP-domination algorithm from Feng&Zilberstein (I think it is different)
}


#if USE_POMDPSOLVE_LIBRARY
//need to include stdio here
//(to avoid 
//      "error: cannot convert 'pomdpsolve::_IO_FILE*' to 'FILE* {aka _IO_FILE*}' for argument '1' to 'int pomdpsolve::vfprintf(FILE*, const char*, __va_list_tag*)' "
// note that it cannot cast 'pomdpsolve::_IO_FILE*' to '_IO_FILE*', so the namespace is the problem (not 'FILE' vs 'IO_FILE')
#include <stdio.h>
namespace pomdpsolve {
    extern "C" {
#include "common.h"
#include "lp-interface.h"
#include "parsimonious.h"
#include "global.h"
#include "params.h"
#include "pomdp.h"
#include "utils.h"
#include "alpha.h"
#include "stats.h"
#include "lp-interface.h"
#include "region.h"
    }
}



namespace pomdp_solve_private_functions{

using namespace pomdpsolve;
/**********************************************************************/
int 
countNonZeroesAlpha( double *alpha, double epsilon ) 
{
  /*
    Just goes through each component of the vector and
    compares the component to zero.  Returns the number of non-zero
    components.
  */
  int i;
  int count = 0;

  Assert( alpha != NULL, "Vector is NULL." );

  for ( i = 0; i < gNumStates; i++ )
    if ( ! Equal( alpha[i], 0.0, epsilon ))
      count++;
  
  return ( count );
}  /* countNonZeroesAlpha */

int 
countNonZeroesList( pomdpsolve::AlphaList list, double epsilon ) 
{
  /*
    Just goes through each component of each vector in the list and
    compares the component to zero.  Returns the number of non-zero
    components.
  */
  int i;
  int count = 0;

  Assert( list != NULL, "List is NULL." );

  list = list->head;
  while ( list != NULL ) {
    
    for ( i = 0; i < gNumStates; i++ )
      if ( ! Equal( list->alpha[i], 0.0, epsilon ))
        count++;
    
    list = list->next;
  } /* while */
  
  return ( count );
}  /* countNonZeroesList */
void 
setUpObjectiveFunction( LP lp ) 
{
  /*
    Sets up the objective function coefficients and sense.

    Note that we do not keep the objective function in a sparse
    structure, so this is the same regardless of whether we are using
    sparse LPs or not.
  */
  int i;

  Assert( lp != NULL, "LP is NULL." );

  /* We will maximize the objective function. */
  lp->objsen = MAXIMIZE;

  /* The objective function is also specific to the LP we will solve
     during solution of POMDP. Set the objective function. */
  for( i = 0; i < lp->cols; i++ )
    lp->obj[i] = 0.0;
  
  /* ...then set the extra variable (delta) coefficient to '1'. Note
     that regardless of whether USE_NEW_LP_FORMULATION is set or not,
     the delta variable appears in the same position.  The new
     variable added by USE_NE_LP_FORMU:ATION is the last variable
     whose position would be right after the delat variable. */
  lp->obj[gNumStates] = 1.0;  /* maximize delta */

  /* For all the belief state variables in the LP, we know that must
     be >=0 and <=1, so we set their bounds here. */
  for( i = 0; i < gNumStates; i++ ) {
    lp->lowbnd[i] = 0.0;
    lp->upbnd[i] = 1.0;
  } /* for i */

  /* For the rest of the columns, we just make the lowerbound 0.0 and
     the upper bound infinite.  */
  for( i = gNumStates; i < lp->cols; i++ ) {
    lp->lowbnd[i] = 0.0;
    lp->upbnd[i] = INFBOUND;
  } /* for i */

 }  /* setUpObjectiveFunction */

void 
addExtraVarColumn( LP lp, int col, int *index, double sign ) 
{
  /*
    With the new LP region formulation we need to add two extra LP
    variables.  The two extra variables added will represent the value
    of the current vector being tested.  Because the LPs only deal with
    positive variable values, we need to represent the value as the
    difference between to positive variables.  Adding the columns for
    these is identical, except for the sign change, which is done with
    the parameter sent in.
  */
  int row;
  
  lp->matbeg[col] = *index;

  /* If we using dense LPs, we need to explicitly put a zeroe in for
     these variables in the simplex constraint (constraint row 0). */
#ifdef USE_DENSE_LPS
  lp->matval[*index] = 0.0;
  lp->matind[(*index)++] = 0;
#endif
  /* The extra constraint coef value in the first constraint... */
  lp->matval[*index] = sign;
  lp->matind[(*index)++] = 1;

  /* ...and now the value for each list vector constraint. */
  for ( row = 2; row < lp->rows; row++ ) {
    
    /* There are some sign changes that need to be done depending upon
       whether the POMDP immediate values represent rewards or
       costs. */ 
    lp->matval[*index] = sign;
  
    lp->matind[(*index)++] = row;
  } /* for row */

  lp->matcnt[col] = *index - lp->matbeg[col];

}  /* addExtraVarColumn */
/**********************************************************************/
} /* namespace pomdp_solve_private_functions */

#endif // USE_POMDPSOLVE_LIBRARY

// this is an intermediate function that deals with converting to alpha vectors and clean up
// TODO: in the longer term, we of course want to avoid conversion to alpha vectors alltogether...?
bool 
QBGPlanner_TreeIncPruneBnB::
CheckNotDominatedLP_POMDPSolve(
                        const AlphaVector & v,
                        const std::vector<AlphaVector> & vSet
        )
{
#if USE_POMDPSOLVE_LIBRARY
    /* If the list is initially empty, then we know that any simplex
     point is a witness to the vector 'alpha' being bettwr than the
     list.  For this case we just return a simplex corner and forego
     the LPs.  Note that letting this empty list case pass through
     will not work, since the setUpRegionLp() doesn't not set up the
     proper LP for this case. */ 
    if(vSet.size() == 0)
        return false; //<- we want to do this before creating the alpha and list below

    double* alpha = AlphaVectorToDoubleP(v);
    pomdpsolve::AlphaList list = AlphaVectorsToAlphaList(vSet);
    bool b = CheckNotDominatedLP_POMDPSolve(alpha, list);
    //free memory:
    pomdpsolve::destroyAlpha(alpha);
    pomdpsolve::destroyAlphaList(list);

    return b;
#else
    throw(E("QBGPlanner_TreeIncPruneBnB needs to be compiled with USE_POMDPSOLVE_LIBRARY"));
    return(false);
#endif
}

#if USE_POMDPSOLVE_LIBRARY

//This function is an adaptation of Tony's findRegionPoint function (in region.c)
bool 
QBGPlanner_TreeIncPruneBnB::
CheckNotDominatedLP_POMDPSolve( double* alpha, pomdpsolve::AlphaList& list)
//int 
//findRegionPoint( double *alpha, 
//                 AlphaList list, 
//                 double *witness_point, double *diff,
//                 PomdpSolveParams param ) 
{
#if DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning
    cout << "CheckDominationLP_POMDPSolve..." << endl;
#endif
    /**Notes Frans:
     * -throughout this code (and comments)...
     *      * alpha refers to the alpha vector we are checking dominance for (i.e. 'v')
     *      * list refers to the 'list of alpha vectors' (i.e. 'vSet')
     *  TODO: replacing those occurences...
     *
     */
    using namespace pomdpsolve;


    
    PomdpSolveParams param = _m_solve_params;

#if DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning
    cout << "alpha:";
    showAlpha(alpha);
    cout << "list:";
    showAlphaList(list);
#endif

    //I think these are output vars that can be safely ignored:
    double* witness_point = NULL;
    double* diff = NULL;

    /*
    Checks to see if the alpha vector 'alpha' has a non-empty region
    (measurable area) where it is better than all the other vectors in
    the 'list'. If the region is non-empty the routine returns TRUE with
    the witness_point set to a point in that region.  If there is no
    point where alpha is better, then FALSE is returned.
    */
    LP lp=NULL;
    int i;

    //Assert( alpha != NULL && list != NULL && param != NULL, "Vector or list is NULL." );

/*    
    if ( list->length == 0 ) {
    if ( witness_point != NULL ) {
      witness_point[0] = 1.0;
      for( i = 1; i < gNumStates; i++ )
        witness_point[i] = 0.0;
    } / if need to return a witness point. /
    if ( diff != NULL )
      *diff = HUGE_VAL;
    return ( TRUE );
    }
*/    

    /* Set up constraints and rest of memory. */
//    lp = setUpRegionLP( alpha, list, param->sparse_epsilon );
//    inlining this function here (cutting away the parts corresponding to USE_OLD_LP_FORMULATION, and USE_DENSE_LPS:
//LP 
//setUpRegionLP( double *alpha, AlphaList list, double sparse_epsilon ) 
{
    double sparse_epsilon =  param->sparse_epsilon;

    int num_constraints, num_variables, num_non_zeroes;
    /* Calculate the LP size for the new LP formulation. */
    num_constraints = list->length + 2;
    num_variables = gNumStates + 3;

    /* If we are using sparse LPs, we need to count the non-zero
     entries.  This is actually a combination of counting and
     calculating based upon knowledge of the problem. 

     Here we have a definite '1' coef. for the belief state variables
     in the simplex constraint.  Then there is the extra constraint
     which is just the number of non-zeroes in the vector being
     compared, plus the extra variable we introduced to force this
     vector's value to be equal to this variable. For each vector in
     the list, we will have at least two non-zero coefs correponding
     to the delta and extra variable, with more entries for each
     non-zero component of the vector.  */

    /* zzz Not sure that these are the proper epsilons to use, but just
     put them in so it would compile. */    
    num_non_zeroes 
    = gNumStates
    + pomdp_solve_private_functions::countNonZeroesAlpha( alpha, sparse_epsilon ) + 2
    + pomdp_solve_private_functions::countNonZeroesList( list, sparse_epsilon ) 
    + 3 * list->length;

    lp = LP_newLP( num_constraints, num_variables, num_non_zeroes );
    lp->sparse_epsilon = sparse_epsilon;

    pomdp_solve_private_functions::setUpObjectiveFunction( lp );

    //setUpRegionConstraintsNew( lp, alpha, list ); //inlining here:
    //void 
    //setUpRegionConstraintsNew( LP lp, double *alpha, 
                               //AlphaList orig_list ) 
    {
        AlphaList orig_list = list;
        /*
        Sets up all the constraints for this LP. Sets the coefficients in
        lp->matval, the sense, the RHS and the bookkeeping arrays
        lp->matbeg, lp->matcnt, lp->matind defining the start and length of
        columns and the row numbers for each lp->matval entry.

        This routine will work with or without a sparse LP formulation.

        This particular set-up was proposed by Bob Givan and has
        the advantages:

        1) Does not require a substraction computation for each state and
          vector. 
        2) Allows a new vector to be compared to the same list by changing
          just one constraint.  

        The minor disadvantages are that it requires one more variable and
        one more constraint that the original method.  I reality, we
        actually have to add two more variables to the LP, since the extra
        variable that is added is unbounded in range.  The standard LP
        strick is to replace an unbounded varkable as the difference of two
        positive variables.  We employ this trick here because lp_solve only
        deals with variables >= 0.  

        This routine will only work if the LP coefficient matrix is dense!

        The LP looks like this for immediate rewards:

         max: delta
         s.t.
              x * 1 = 1
              x * alpha - v1 + v2 = 0
           x * alphatilde + delta - v1 + v2 <= 0, for all alphatilde in list

        and  looks like this for immediate cost POMDPs:

        max: delta
        s.t.
             x * 1 = 1
             x * alpha - v1 + v2 = 0
          v1 - v2 - x * alphatilde + delta <= 0, for all alphatilde

        where x is a vector of variables and alpha and alphatilde are
        linear hyperplane coefficients (i.e., alpha vectors).  Delta and
        'v1' and 'v2' are LP variables and the region has measurable volume
        if delta > 0.

        */
        int index, row, col;
        AlphaList list;

        /* This routine should only be called if this constant is set,
         otherwise the LPs will not be allocated with too little space. */
#ifdef USE_OLD_LP_FORMULATION
        Abort( "Cannot use this function unless USE_OLD_LP_FORMULATION is set." );
#endif

        Assert( lp != NULL && alpha != NULL && orig_list != NULL,
              "Bad (NULL) parameter(s)." );

        /* Iterate over the constraint coefficient matrix column by
         column. 'index' will be the current index into lp->matval and we
         will increment it everytime we set it. */
        index = 0;

        /****************************************************/
        /* Columns corresponding to belief state variables. */
        /****************************************************/

        for ( col = 0; col < gNumStates; col++ ) {

        /* Need to set the bookkeeping array for where this column
           starts. */
        lp->matbeg[col] = index;

        /* Since the first constraint is the simplex constraint, the first
           thing we need to do for this belief state variable (each column
           is a belief state variable) is set the coefficient to '1'. */
        lp->matval[index] = 1.0;
        lp->matind[index++] = 0;

        /* For the new LP formulation, the second constraint is the extra
           constraint which effectively assigns the vectors value to an
           extra variable. Set this coefficient now. */
#ifndef USE_DENSE_LPS
        if ( ! Equal( alpha[col], 0.0, lp->sparse_epsilon ))
#endif
          {
            lp->matval[index] = alpha[col];
            lp->matind[index++] = 1;
          }

        /* Looping over this list is now looping over the rows, but
           starting at row = 2, since The simplex constraint was row '0'
           and the extra constraint was row '1'. */
        row = 2;
        list = orig_list->head;
        while ( list != NULL ) {

#ifndef USE_DENSE_LPS
        if ( ! Equal( list->alpha[col], 0.0, lp->sparse_epsilon ))
#endif
          {
            lp->matval[index] = list->alpha[col];
            lp->matind[index++] = row;
          }

          list = list->next;
          row++;
        } /* while list != NULL */

        /* We can compute the number of entries we entered by looking at
           the current index position and where this column started. */
        lp->matcnt[col] = index - lp->matbeg[col];

        } /* for col */

        /*******************************************/
        /* Column corresponding to delta variable. */
        /*******************************************/

        lp->matbeg[gNumStates] = index;

        /* Now we add the column corresponding to the delta varible. This
         will be '1.0' for every constraint row of the vector list and
         zero for the first two constraints (simplex and extra one),
         (which we only add if we are using dense matrices.) */
#ifdef USE_DENSE_LPS
        lp->matval[index] = 0.0;
        lp->matind[index++] = 0;
        lp->matval[index] = 0.0;
        lp->matind[index++] = 1;
#endif
        for ( row = 2; row < (orig_list->length + 2); row++ ) {
        lp->matval[index] = 1.0;
        lp->matind[index++] = row;
        } /* for row */

        lp->matcnt[gNumStates] = index - lp->matbeg[gNumStates];

        /************************************************/
        /* Column corresponding to the extra variables. */
        /************************************************/

        /* We now add the two columns that corresponds to the extra
         varibles.  These will appear in every constraint but the simplex
         constraint, though its sign may be different in rows 2 upward due
         to the reward/cost immediate reward issue. We only add the zero
         for the simplex consraint if we are using dense matrices. */

        pomdp_solve_private_functions::addExtraVarColumn( lp, lp->cols-2, &index, -1.0 );
        pomdp_solve_private_functions::addExtraVarColumn( lp, lp->cols-1, &index, 1.0 );

        /* After all is said and done, we should have entered the same
         number of entries as we computed we would need when we allocated
         the LP. */
        Assert( index == lp->matspace, 
              "Computed non-zeroes didn't match actual non-zeroes." );

        /***********************************************/
        /* Constraint RHS and sense.                   */
        /***********************************************/

        /* Simplex constraint. */
        lp->sense[0] = 'E';
        lp->rhs[0] = 1.0;

        /* Extra consraint. */
        lp->sense[1] = 'E';
        lp->rhs[1] = 0.0;

        /* Remainder of constraints. */
        for ( row = 2; row < (orig_list->length + 2); row++ ) {
        lp->sense[row] = 'L';
        lp->rhs[row] = 0.0;
        } /* for row */

    }  /* setUpRegionConstraintsNew */
    /**********************************************************************/


    //return ( lp );
}  /* setUpRegionLP */
/**********************************************************************/

    /* See if we get a feasible solution to the LP, but if not just
     return FALSE. */
    switch ( LP_solveLP( lp, param->stat )) {
    case LP_OPTIMAL:
    /* Ok, fall through to code below. */
    //cout << "case LP_OPTIMAL"<<endl;
    break;

    case LP_INFEASIBLE:
    LP_freeLP( lp );
    //cout << "case LP_INFEASIBLE"<<endl;
    return ( FALSE );

    case LP_UNBOUNDED:
    /* We will assume that an unbounded LP has resulted from numerical
       precision/instability issues and just report this as having no
       region. */
    Warning( "LP return status is unbounded. Assuming infeasible." );
    LP_freeLP( lp );
    return ( FALSE );

    default:
    Abort( "LP return status is unknown." );
    }

    /* If the 'diff' argument is not NULL then we return the objective
     value in it. */
    if ( diff != NULL )
    *diff = lp->objval;

    /* If the LP is feasible, then we need to make sure the objective
     value is > 0.  We want to objective value to be greater than
     zero, but use an epsilon factor for two reasons: first, numerical
     stability requires this and second, it provides the place where
     optimization can be achieved by boosting this up. Since the
     objective function consists of a single variable with a
     coefficient of '1', the objective value and the solution value of
     that variable should be identical. Alas, this does not always
     seem to be the case because of precision issues.  Thus we only
     claim the result is larger than zero if *both* the objective
     value and the variable value are larger than the epilon value we
     are interested in. */
    if( ( lp->objval < param->epsilon )
      || ( lp->x[gNumStates] < param->epsilon ))   {
        LP_freeLP( lp );
        return ( FALSE );
    }

    /* Just copy solution, if it was wanted. */
    if ( witness_point != NULL )
    for( i = 0; i < gNumStates; i++ )
      witness_point[i] = lp->x[i];

    LP_freeLP( lp );
    return ( TRUE );

}

#endif // USE_POMDPSOLVE_LIBRARY

bool 
QBGPlanner_TreeIncPruneBnB::IsDominatedNode(
                    Index depth, 
                    //const VectorSet & L,
                    //const VectorSet & G
                        std::vector<AlphaVector> & L, // <- the lower bound vectors
                        std::vector<AlphaVector> & G  //The implied g-function
        )
{
    std::vector<AlphaVector> & H = _m_heuristic_for_depth[depth];
    std::vector<AlphaVector> F;
    CrossSum(G, H, F);
#if DEBUG_QBGPlanner_TreeIncPruneBnB_Pruning
    cout << "IsDominatedNode:: this node's F set=\n" <<SoftPrintVector(F) << endl;
#endif
    for(Index fI=0; fI < F.size(); fI++)
    {
        AlphaVector f = F[fI];
        bool dominated = CheckDomination(f, L);
        if(!dominated)
            return false;
    }
    return true;
}

//VectorSet 
void
QBGPlanner_TreeIncPruneBnB::UnionNode(Index a,
                                Index depth,
                                const std::vector< std::vector<Index> > &jpol,
                                std::vector<AlphaVector> & L, // <- the lower bound vectors
                                std::vector<AlphaVector> & G  //The implied g-function
                                //VectorSet & L, // <- the lower bound vectors
                                //VectorSet & G
                            )
{
    _m_union_nodes_at_level.at(depth)++;
    _m_nodes_visited++;

    vector<Index> os = GetObservationsFromDepth(depth);
#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
    cout << "UnionNode:    a=" << a << ", depth=" << depth;
#if DEBUG_QBGPlanner_TreeIncPruneBnB
    cout << "(obs="
         << SoftPrintVector(os) << "),jpol="
         << SoftPrintJpol(jpol) << endl;
    cout << "L: " << SoftPrintVector(L) << endl;
    cout << "G: " << SoftPrintVector(G) << endl;
#endif
    cout << " L.size="<<L.size() << ", G.size=" << G.size() << endl;

#endif

    //check if this node can be pruned
    if(this->IsDominatedNode(depth, L, G))
    {
        _m_nr_backtracks++;
        if(_m_nr_backtracks % 1000 == 0)
            cout << "nr back tracks (pruned nodes) = "<< _m_nr_backtracks<<endl;
#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
        cout << "---NODE DOMINATED, backtracking...---" << endl;
#endif
        return;
    }
    
    vector<Scope> jas = GetValidJointActions(depth,jpol);
    if(depth == _m_maxDepth-1)
        if(jas.size()>1)
            throw(E("QBGPlanner_TreeIncPruneBnB::UnionNode should only a single joint action possible"));
        //G=*GetGaoa(a,os,jas[0]); // No, in top-down approach we do create the last cross-sum nodes.

    for(Index i = 0; i != jas.size(); ++i)
    {
        // extend the current jpol with one of the valid joint actions
        vector<vector<Index> > jpolNew = jpol;
        for(Index agI=0; agI!=GetPU()->GetNrAgents(); ++agI)
        {
            if(jpolNew[agI][os[agI]]==UNSPECIFIED_ACTION || jpolNew[agI][os[agI]]==jas[i][agI])
                jpolNew[agI][os[agI]]=jas[i][agI];
            else
                throw(E("QBGPlanner_TreeIncPruneBnB::UnionNode observation already specified (or different)"));
        }
#if 0 && DEBUG_QBGPlanner_TreeIncPruneBnB
        cout << SoftPrintJpol(jpol) << " + " << SoftPrintVector(jas[i])
             << " = " << SoftPrintJpol(jpolNew) << endl;
#endif
        CrossSumNode(a, depth, jpolNew, L, G);
       
        //TODO check if necessary? 
        //L=Prune(L);
    }

}

//VectorSet
void
QBGPlanner_TreeIncPruneBnB::CrossSumNode(Index a,
                        Index depth,
                        const vector<vector<Index> > &jpol,
                        std::vector<AlphaVector> & L, // <- the lower bound vectors
                        std::vector<AlphaVector> & G    //The implied g-function:
                        //VectorSet & L, // <- the lower bound vectors
                        //VectorSet G    //The implied g-function:
                )
{
    _m_cs_nodes_at_level.at(depth)++;
    _m_nodes_visited++;
    vector<Index> os=GetObservationsFromDepth(depth);

#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
    cout << "CrossSumNode: a=" << a << ", depth=" << depth;
#if DEBUG_QBGPlanner_TreeIncPruneBnB
    cout << "(obs="
         << SoftPrintVector(os) << "),jpol="
         << SoftPrintJpol(jpol) << endl;
    cout << "L: " << SoftPrintVector(L) << endl;
    cout << "G: " << SoftPrintVector(G) << endl;
#endif
    cout << " L.size="<<L.size() << ", G.size=" << G.size() << endl;
#endif


    // figure out which a,o,aPrime we need to get
    vector<Index> as(jpol.size());
    for(Index agI = 0; agI != as.size(); ++agI) 
        as[agI] = jpol[agI][os[agI]];
    std::vector<AlphaVector> *Gaoa = GetGaoa(a,os,as);

#if DEBUG_QBGPlanner_TreeIncPruneBnB
    cout << "Set for crosssum, Gaoa["<<a<<"]["<<SoftPrintVector(os)<<"]["<<SoftPrintVector(as)<<"] = ";
    cout << SoftPrintVector( *Gaoa) << endl;
#endif



    std::vector<AlphaVector> newG;
    AlphaVectorPlanning::CrossSum(G, *Gaoa, newG);
    newG = Prune(newG);
    if(depth < _m_maxDepth-1)
        //deeper down the rabbit hole...
        UnionNode(a, depth+1, jpol, L, newG);
    else
    {
        _m_nr_bottom_hits++;
        if(_m_nr_bottom_hits % 1000 == 0)
            cout << "nr bottom hits = "<< _m_nr_bottom_hits << endl;

#if DEBUG_QBGPlanner_TreeIncPruneBnBVerbose
        cout << "\n---HIT BOTTOM------------------------\n";
#endif
#if DEBUG_QBGPlanner_TreeIncPruneBnBV
        cout << "adding newG=" << SoftPrintVector(newG);
        cout << "\nto L       =" << SoftPrintVector(L);
#endif
        //G now is a lower bound.    
        //1) add G to L
        L.insert( L.begin(), newG.begin(), newG.end());
        //2) prune L
        L = Prune(L);

#if DEBUG_QBGPlanner_TreeIncPruneBnB
        cout << "\npruned res.=" << SoftPrintVector(L);
        cout << endl;
#endif
    }
}


ValueFunctionPOMDPDiscrete QBGPlanner_TreeIncPruneBnB::GetValueFunction(size_t horizon)
{
    return(QFunctionsToValueFunction(_m_qFunction[horizon-1]));
}

vector<Scope> QBGPlanner_TreeIncPruneBnB::GetValidActions(
    Index depth,
    const vector<vector<Index> > &jpol) const
{
    size_t nrAgents=jpol.size();

    vector<Index> Os=GetObservationsFromDepth(depth);
    vector<Scope> as;
    for(Index k=0;k!=nrAgents;++k)
    {
        Scope aI;
        if(jpol[k][Os[k]]==UNSPECIFIED_ACTION)
            // we didn't specify this obs yet, so all actions are valid
            for(Index a=0;a!=GetPU()->GetNrActions(k);++a)
                aI.push_back(a);
        else // we already assigned this obs, so we can only use that
             // particular action
            aI.push_back(jpol[k][Os[k]]);
        as.push_back(aI);
    }

    return(as);
}

vector<Scope> QBGPlanner_TreeIncPruneBnB::GetValidJointActions(
    Index depth,
    const vector<vector<Index> > &jpol) const
{
    size_t nrAgents=jpol.size();

    vector<Scope> as=GetValidActions(depth,jpol);

    // now construct the set of joint actions
    vector<Scope> jas;
    switch(nrAgents)
    {
    case 2:
    {
#if DEBUG_TreeIncPruneBGPlanner    
        cout << "GetValidJointActions for depth " << depth
             << " jpol " << SoftPrintJpol(jpol) << ":";
#endif

        for(Index a0=0;a0!=as[0].size();++a0)
            for(Index a1=0;a1!=as[1].size();++a1)
            {
                vector<Index> asI(nrAgents);
                asI[0]=as[0][a0];
                asI[1]=as[1][a1];
                jas.push_back(asI);
#if DEBUG_TreeIncPruneBGPlanner    
                cout << SoftPrintVector(asI);
#endif
            }
#if DEBUG_TreeIncPruneBGPlanner    
        cout << endl;
#endif
        break;
    }
    case 3:
    {
        for(Index a0=0;a0!=as[0].size();++a0)
            for(Index a1=0;a1!=as[1].size();++a1)
                for(Index a2=0;a2!=as[2].size();++a2)
                {
                    vector<Index> asI(nrAgents);
                    asI[0]=as[0][a0];
                    asI[1]=as[1][a1];
                    asI[2]=as[2][a2];
                    jas.push_back(asI);
                }
        break;
    }
    case 4:
    {
        for(Index a0=0;a0!=as[0].size();++a0)
            for(Index a1=0;a1!=as[1].size();++a1)
                for(Index a2=0;a2!=as[2].size();++a2)
                    for(Index a3=0;a3!=as[3].size();++a3)
                    {
                        vector<Index> asI(nrAgents);
                        asI[0]=as[0][a0];
                        asI[1]=as[1][a1];
                        asI[2]=as[2][a2];
                        asI[3]=as[3][a3];
                        jas.push_back(asI);
                    }
        break;
    }
    default:
        throw(E("TreeIncPruneBGPlanner::GetValidJointActions this number of agents"));
    }

    return(jas);
}

Index QBGPlanner_TreeIncPruneBnB::GetJointObservationsFromDepth(Index depth) const
{
    // here we just enumerate all joint observations in their original
    // order, so the jo index is equal to the depth
    return depth;
}


vector<Index> QBGPlanner_TreeIncPruneBnB::GetObservationsFromDepth(Index depth) const
{
    // here we just enumerate all joint observations in their original
    // order, so the jo index is equal to the depth
    Index jo=depth;
    vector<Index> os=GetPU()->JointToIndividualObservationIndices(jo);
#if 0 && DEBUG_QBGPlanner_TreeIncPruneBnB
    cout << "Depth " << depth << " Obs " << SoftPrintVector(os) << endl;
#endif
    return(os);
}

string QBGPlanner_TreeIncPruneBnB::SoftPrintJpol(
    const vector<vector<Index> > &jpol) const
{
    stringstream ss;
    for(Index k=0;k!=jpol.size();++k)
    {
        ss << "[";
        for(Index o=0;o!=jpol[k].size();++o)
        {
            if(jpol[k][o]==UNSPECIFIED_ACTION)
                ss << "U";
            else
                ss << jpol[k][o];
        }
        ss << "]";
    }
    
    return(ss.str());
}



string QBGPlanner_TreeIncPruneBnB::SoftPrintBrief() const
{
    stringstream ss;
    ss << "QBGPlanner_TreeIncPruneBnB";
    //TODO
    return(ss.str());
}

string QBGPlanner_TreeIncPruneBnB::SoftPrintStats() const
{
    stringstream ss;
    double nr_jp = 1;
    for(Index agI=0; agI < GetPU()->GetNrAgents(); agI++)
    {
        double nrAcsI = (double) GetPU()->GetNrActions(agI);
        double nrObsI = (double) GetPU()->GetNrObservations(agI);
        double pols_agI = (size_t) pow(nrAcsI, nrObsI);
        nr_jp *= pols_agI;
    }
    size_t nrJA = GetPU()->GetNrJointActions();
    ss << "nr joint BG pols:   " << nr_jp << ", nr JA=" << nrJA;
    ss << "-> number of expected bottom leafs = " << nr_jp * nrJA <<  endl;
    ss << "_m_nr_bottom_hits:  " << _m_nr_bottom_hits << endl;
    ss << "_m_nr_backtracks:   " << _m_nr_backtracks << endl;
    ss << "_m_nodes_visited:   " << _m_nodes_visited << endl;
    ss << "depth  union nodes  cs nodes" << endl;
    for(Index d=0; d < _m_maxDepth; d++)
    {
#if 0 // the following doesn't compile with arbitrary length integers
        char s[50];
        sprintf(s, "%2d     %6u     %6u\n", d, _m_union_nodes_at_level.at(d), _m_cs_nodes_at_level.at(d) );
        ss << s;
#else
        ss <<  d << "        " 
           << _m_union_nodes_at_level.at(d) << "        " 
           << _m_cs_nodes_at_level.at(d) << endl;
#endif
    }

/*    
    ss << "Cache hits " << _m_cacheHit << " misses " << _m_cacheMiss
       << " ratio " << static_cast<double>(_m_cacheHit)/(_m_cacheHit+_m_cacheMiss) << endl;
    ss << "Prune after Union stats (size before, size after):";
    for(Index k=0; k!=_m_pruneAfterUnionStats.size();++k)
        ss << " (" << _m_pruneAfterUnionStats[k][0] << ","
           << _m_pruneAfterUnionStats[k][1] << ")";
    ss << endl;
    ss << "Prune after CrossSum stats (size before, size after):";
    for(Index k=0; k!=_m_pruneAfterCrossSumStats.size();++k)
        ss << " (" << _m_pruneAfterCrossSumStats[k][0] << ","
           << _m_pruneAfterCrossSumStats[k][1] << ")";
    ss << endl;
*/
    return(ss.str());
}
