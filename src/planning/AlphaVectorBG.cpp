/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "AlphaVectorBG.h"
#include <float.h>
#include <sys/times.h>
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "BGIP_SolverAlternatingMaximization.h"
#include "BGIP_SolverBranchAndBound.h"
#include "BeliefValue.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "AlphaVector.h"
#include "BayesianGameIdenticalPayoff.h"

using namespace std;

#define DEBUG_AlphaVectorBG_BeliefBackup 0
#define DEBUG_AlphaVectorBG_CheckBGIP_SolverExhaustive 0

//Default constructor
AlphaVectorBG::AlphaVectorBG(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu)
{
    _m_bgip = boost::shared_ptr<BayesianGameIdenticalPayoff>(
        new BayesianGameIdenticalPayoff(pu->GetNrAgents(),
                                        pu->GetDPOMDPD()->GetNrActions(), 
                                        pu->GetDPOMDPD()->GetNrObservations()));
}

AlphaVectorBG::AlphaVectorBG(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu)
{
    _m_bgip = boost::shared_ptr<BayesianGameIdenticalPayoff>(
        new BayesianGameIdenticalPayoff(pu->GetNrAgents(),
                                        pu->GetDPOMDPD()->GetNrActions(), 
                                        pu->GetDPOMDPD()->GetNrObservations()));
}

//Destructor
AlphaVectorBG::~AlphaVectorBG()
{
}

AlphaVector
AlphaVectorBG::BeliefBackup(const JointBeliefInterface &b,
                            Index a,
                            const GaoVectorSet &G,
                            const ValueFunctionPOMDPDiscrete &V,
                            BGBackupType type) const
{
#if DEBUG_AlphaVectorBG_BeliefBackup
    tms timeStruct;
    clock_t ticks_before, ticks_after;
    ticks_before = times(&timeStruct);
#endif

    StartTimer("BeliefBackupBG");

    AlphaVector alpha(b.Size());
    switch(type)
    {
    case EXHAUSTIVE_ONLYKEEPMAX:
        alpha=BeliefBackupExhaustiveOnlyKeepMax(b,a,G,V);
        break;
    case EXHAUSTIVE_STOREALL:
        alpha=BeliefBackupExhaustiveStoreAll(b,a,G,V);
        break;
    case BGIP_SOLVER_EXHAUSTIVE:
    case BGIP_SOLVER_ALTERNATINGMAXIMIZATION:
    case BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS:
    case BGIP_SOLVER_BRANCH_AND_BOUND:
        alpha=BeliefBackupBGIP_Solver(b,a,G,V,type);
        break;
    }

    StopTimer("BeliefBackupBG");

#if DEBUG_AlphaVectorBG_BeliefBackup
    ticks_after = times(&timeStruct);
    cout << "AlphaVectorBG::BeliefBackup backuptype " << type
         << " done in " 
         << ticks_after - ticks_before << " clock ticks, "
         << static_cast<double>((ticks_after - ticks_before))
        / sysconf(_SC_CLK_TCK) 
         << "s" << endl;
#endif

#if DEBUG_AlphaVectorBG_CheckBGIP_SolverExhaustive
    if(type==BGIP_SOLVER_EXHAUSTIVE)
    {
        AlphaVector alphaOK=BeliefBackup(b,a,G,V,EXHAUSTIVE_ONLYKEEPMAX);
        if(!alphaOK.Equal(alpha))
            abort();
    }
#endif

    return(alpha);
}

vector<vector<bool> >
AlphaVectorBG::GetMask(const ValueFunctionPOMDPDiscrete &V) const
{
    vector<vector<bool> > mask;
    size_t nrInV=V.size();
    for(unsigned int a1=0;a1!=GetPU()->GetNrJointActions();++a1)
    {
        vector<bool> maskA(nrInV,false);
        for(unsigned int i=0;i!=nrInV;++i)
            if(V[i].GetAction()==a1)
                maskA[i]=true;

        mask.push_back(maskA);
    }

    return(mask);
}

AlphaVector
AlphaVectorBG::BeliefBackupBGIP_Solver(const JointBeliefInterface &b,
                                    Index a,
                                    const GaoVectorSet &G,
                                    const ValueFunctionPOMDPDiscrete &V,
                                    BGBackupType type) const
{
    // Equation numbers refer to PWLC_Dec-POMDPs_b.ps of Nov 7

    unsigned int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates();
    double gamma=GetPU()->GetDiscount();
    double value;

    // the mask selects which vectors to consider:
    // mask[jaI][vI] is true <-> vector vI specifies action jaI
    vector<vector<bool> > mask=GetMask(V);

    boost::numeric::ublas::matrix<int> bestG_oa1(nrO,nrA);

    for(unsigned int o=0;o!=nrO;o++)
        for(unsigned int a1=0;a1!=nrA;++a1)
        {
            bestG_oa1(o,a1)=
                BeliefValue::GetMaximizingVectorIndexAndValue(b,*G[a][o],
                                                              mask[a1],value);

            if(bestG_oa1(o,a1)==-1)
                abort();

            _m_bgip->SetUtility(o,a1,value);
            _m_bgip->SetProbability(o,1.0/nrO);
        }

    //BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> *BGIP_Solver;
    BayesianGameIdenticalPayoffSolver *BGIP_Solver;
    switch(type)
    {
    case BGIP_SOLVER_EXHAUSTIVE:
        BGIP_Solver=new BGIP_SolverBruteForceSearch<JointPolicyPureVector>(_m_bgip);
        break;
    case BGIP_SOLVER_ALTERNATINGMAXIMIZATION:
        BGIP_Solver=new BGIP_SolverAlternatingMaximization<JointPolicyPureVector>(_m_bgip);
        break;
    case BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS:
        BGIP_Solver=new BGIP_SolverAlternatingMaximization<JointPolicyPureVector>(_m_bgip,100);
        break;
    case BGIP_SOLVER_BRANCH_AND_BOUND:
        BGIP_Solver=new BGIP_SolverBranchAndBound<JointPolicyPureVector>(_m_bgip,
                                                                         false, 1, false,
                                                                         BGIP_BnB::MaxContributionDifference,
                                                                         true);
        break;
    default:
        throw(E("AlphaVectorBG::BeliefBackupBGIP_Solver type not supported"));
    }

    BGIP_Solver->Solve();
    JointPolicyPureVector jpol=BGIP_Solver->GetJointPolicyPureVector();

    delete BGIP_Solver;

    vector<double> best(nrS);
    Index a1;
    for(unsigned int s=0;s!=nrS;++s)
    {
        best[s]=0;
        for(unsigned int o=0;o!=nrO;o++)
        {
            a1=jpol.GetJointActionIndex(o);
            if(bestG_oa1(o,a1)!=-1)
                best[s]+=(*G[a][o])(bestG_oa1(o,a1),s);
        }
    }

    double x;
    // create the vector for b
    AlphaVector newVector(nrS);
    newVector.SetAction(a);
    newVector.SetBetaI(jpol.GetIndex());
    for(unsigned int s=0;s!=nrS;s++)
    {
        // (19)
        x=GetPU()->GetReward(s,a)+gamma*best[s];
        newVector.SetValue(x,s);
    }

    return(newVector);
}

AlphaVector
AlphaVectorBG::BeliefBackupExhaustiveOnlyKeepMax(const JointBeliefInterface &b,
                                                 Index a,
                                                 const GaoVectorSet &G,
                                                 const 
                                                 ValueFunctionPOMDPDiscrete &V)
    const
{
    // Equation numbers refer to PWLC_Dec-POMDPs_b.ps of Nov 7

    unsigned int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates();
    double gamma=GetPU()->GetDiscount();

    // the mask selects which vectors to consider:
    // mask[jaI][vI] is true <-> vector vI specifies action jaI
    vector<vector<bool> > mask=GetMask(V);

    // given a particular \beta
    //  for all o,   g*_bao\beta = arg max_g^va'_ao \sum_s g^va'_ao(s) * b(s)
    //
    // i.e., we select the g^va'_ao from a set consistent with \beta
    //  (which means that \beta(o) = a') that maximizes the current belief.
    //
    // In this code, however, we do something slightly different namely:
    //
    //  for all o, forall a' 
    //      g*_baoa' =  arg max_g^va'_ao \sum_s g^va'_ao(s) * b(s)
    //
    // I.e., we select the maximizing vector for each possible a'.
    // we store this in bestG_oa1[o][a'].
    boost::numeric::ublas::matrix<int> bestG_oa1(nrO,nrA);

    // (14)
    for(unsigned int o=0;o!=nrO;o++)
        for(unsigned int a1=0;a1!=nrA;++a1)
            bestG_oa1(o,a1)=BeliefValue::GetMaximizingVectorIndex(b,*G[a][o],
                                                                  mask[a1]);

    // now we create a jpol for the induced Bayesian game
    // (i.e. \beta mentioned above, is the policy for a Bayesian game)
    // and use that to combine the bestG_oa1 to g_a-vectors:
    //  g_ba\beta = \sum_o bestG_oa1[o][ \beta(o) ]
    JointPolicyPureVector jpol(_m_bgip);
    Index a1;
    bool round=false;

    vector<double> current(nrS),best(nrS);
    double v,bestValue=-DBL_MAX;
    AlphaVector::BGPolicyIndex betaMaxI=-1;
    int k=0;
    // (16)
    while (!round) // i.e. forall \beta
    {
        // first create the current g_baBeta-vector:
        //  current g_ba\beta = \sum_o bestG_oa1[o][ \beta(o) ]
        for(unsigned int s=0;s!=nrS;++s)
        {
            current[s]=0;
            for(unsigned int o=0;o!=nrO;o++)
            {
                a1=jpol.GetJointActionIndex(o);
                /* following code implements:
                 * VectorSet V=*G[a][o];
                 * int i=bestG_oa1(o,a1);
                 * current[s]+=V(i,s); // +=V[i][s]; */
                 if(bestG_oa1(o,a1)!=-1)
                     current[s]+=(*G[a][o])(bestG_oa1(o,a1),s);
            }
        }
        // check if it is better...
        v=b.InnerProduct(current);
        if(v>bestValue)
        {
            bestValue=v;
            best=current;
            betaMaxI=k;
        }
        round = ++(jpol);
        k++;
    }

    double x;
    // create the vector for b
    AlphaVector newVector(nrS);
    newVector.SetAction(a);
    newVector.SetBetaI(betaMaxI);
    for(unsigned int s=0;s!=nrS;s++)
    {
        // (19)
        x=GetPU()->GetReward(s,a)+gamma*best[s];
        newVector.SetValue(x,s);
    }

    return(newVector);
}

AlphaVector
AlphaVectorBG::BeliefBackupExhaustiveStoreAll(const JointBeliefInterface &b,
                                              Index a,
                                              const GaoVectorSet &G,
                                              const 
                                              ValueFunctionPOMDPDiscrete &V)
    const
{
    // Equation numbers refer to PWLC_Dec-POMDPs_b.ps of Nov 7

    unsigned int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates();
    double gamma=GetPU()->GetDiscount();

    // the mask selects which vectors to consider:
    // mask[jaI][vI] is true <-> vector vI specifies action jaI
    vector<vector<bool> > mask=GetMask(V);

    // given a particular \beta
    //  for all o,   g*_bao\beta = arg max_g^va'_ao \sum_s g^va'_ao(s) * b(s)
    //
    // i.e., we select the g^va'_ao from a set consistent with \beta
    //  (which means that \beta(o) = a') that maximizes the current belief.
    //
    // In this code, however, we do something slightly different namely:
    //
    //  for all o, forall a' 
    //      g*_baoa' =  arg max_g^va'_ao \sum_s g^va'_ao(s) * b(s)
    //
    // I.e., we select the maximizing vector for each possible a'.
    // we store this in bestG_oa1[o][a'].
    boost::numeric::ublas::matrix<int> bestG_oa1(nrO,nrA);

    // (14)
    for(unsigned int o=0;o!=nrO;o++)
        for(unsigned int a1=0;a1!=nrA;++a1)
            bestG_oa1(o,a1)=BeliefValue::GetMaximizingVectorIndex(b,*G[a][o],
                                                                  mask[a1]);

    // now we create a jpol for the induced Bayesian game
    // (i.e. \beta mentioned above, is the policy for a Bayesian game)
    // and use that to combine the bestG_oa1 to g_a-vectors:
    //  g_ba\beta = \sum_o bestG_oa1[o][ \beta(o) ]
    JointPolicyPureVector jpol(_m_bgip);
    Index a1;
    bool round=false;

    VectorSet g_baBeta(CastLIndexToIndex(_m_bgip->GetNrJointPolicies()),nrS);
    g_baBeta.clear();
    int k=-1;

    // (16)
    while (!round) // i.e. forall \beta
    {
        k++;
        for(unsigned int o=0;o!=nrO;o++)
            for(unsigned int s=0;s!=nrS;++s)
            {
                a1=jpol.GetJointActionIndex(o);
                /* following code implements:
                 * VectorSet V=*G[a][o];
                 * int i=bestG_oa1(o,a1);
                 * g_baBeta(k,s)+=V(i,s); // +=V[i][s]; */
                 if(bestG_oa1(o,a1)!=-1)
                     g_baBeta(k,s)+=(*G[a][o])(bestG_oa1(o,a1),s);
            }
        round = ++(jpol);
    }

    AlphaVector::BGPolicyIndex betaMaxI=
        BeliefValue::GetMaximizingVectorIndex(b,g_baBeta);

    double x;
    // create the aplha-vector for b (i.e. add immediate reward)
    AlphaVector newVector(nrS);
    newVector.SetAction(a);
    newVector.SetBetaI(betaMaxI);
    for(unsigned int s=0;s!=nrS;s++)
    {
        // (19)
        x=GetPU()->GetReward(s,a)+gamma*g_baBeta(CastLIndexToIndex(betaMaxI),s);
        newVector.SetValue(x,s);
    }

    return(newVector);
}

string AlphaVectorBG::SoftPrintBackupType(BGBackupType bgBackupType)
{
    string str;
    if(bgBackupType==-1)
        return("POMDP");

    switch(bgBackupType)
    {
    case EXHAUSTIVE_ONLYKEEPMAX:
        str="EXHAUSTIVE_ONLYKEEPMAX";
        break;
    case EXHAUSTIVE_STOREALL:
        str="EXHAUSTIVE_STOREALL";
        break;
    case BGIP_SOLVER_EXHAUSTIVE:
        str="BGIP_SOLVER_EXHAUSTIVE";
        break;
    case BGIP_SOLVER_ALTERNATINGMAXIMIZATION:
        str="BGIP_SOLVER_ALTERNATINGMAXIMIZATION";
        break;
    case BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS:
        str="BGIP_SOLVER_ALTERNATINGMAXIMIZATION_100STARTS";
        break;
    case BGIP_SOLVER_BRANCH_AND_BOUND:
        str="BGIP_SOLVER_BRANCH_AND_BOUND";
        break;
    }
    return(str);
}
