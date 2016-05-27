/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "MonahanBGPlanner.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BeliefValue.h"
#include "Belief.h"

#include "BayesianGameIdenticalPayoff.h"
#include "JointPolicyPureVector.h"

using namespace std;

#define DEBUG_VERIFYBACKUPSTAGE 0

//Default constructor
MonahanBGPlanner::MonahanBGPlanner(const PlanningUnitDecPOMDPDiscrete* pu,
                                   bool doIncPrune) :
    MonahanPlanner(pu,doIncPrune)
{
}

MonahanBGPlanner::MonahanBGPlanner(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                                   bool doIncPrune) :
    MonahanPlanner(pu,doIncPrune)
{
}

//Destructor
MonahanBGPlanner::~MonahanBGPlanner()
{
}

void MonahanBGPlanner::Initialize()
{
    AlphaVectorPlanning::Initialize();
    MonahanPlanner::_m_initialized=true;
}

QFunctionsDiscrete
MonahanBGPlanner::BackupStage(const QFunctionsDiscrete &Qs, size_t maxNrAlphas)
{
    ValueFunctionPOMDPDiscrete V=QFunctionsToValueFunction(Qs);

    int nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrInV=V.size();
    double gamma=GetPU()->GetDiscount();

    GaobetaVectorSet G=ComputeAllGaoa(V);

    boost::shared_ptr<BayesianGameIdenticalPayoff> bg_time_step=
        boost::shared_ptr<BayesianGameIdenticalPayoff>(new BayesianGameIdenticalPayoff(GetPU()->GetNrAgents(), 
                                                                                       GetPU()->GetNrActions(),
                                                                                       GetPU()->GetNrObservations()));
    JointPolicyPureVector jp(bg_time_step);

    AlphaVector alpha(nrS);
    
    QFunctionsDiscrete Q(nrA);

    // initialize with the number of vectors already computed for other time steps
    size_t nrVectorsComputed=GetNrVectors();

    LIndex nrJPols=bg_time_step->GetNrJointPolicies();
    for(GaoVectorSetIndex a=0;a!=nrA;a++)
    {
        for(Index betaI=0;betaI!=nrJPols;++betaI)
        {
            PrintProgress("BG Jpol #",betaI,nrJPols,1000);
            jp.SetIndex(betaI);
            VectorSet Ga=*G[a][0][jp.GetJointActionIndex(static_cast<Globals::Index>(0))];//*(Gbeta[0]); 
            for(Index o=1;o!=nrO;o++)
            {
                if(_m_doIncPrune)
                {
                    VectorSet Ga2=CrossSum(Ga,*G[a][o][jp.GetJointActionIndex(o)]);//*(Gbeta[o]));
                    Ga=Prune(Ga2);
                }
                else
                {
                    VectorSet Ga1=CrossSum(Ga,*G[a][o][jp.GetJointActionIndex(o)]);//*(Gbeta[o]));
                    Ga=Ga1;
                }
#if DEBUG_AlphaVectorPlanning_CrossSum    
                if(maxNrAlphas)
                    cout << "AlphaVectorPlanning::MonahanCrossSum nrAlphas " << Ga.size1()
                         << " (max " << maxNrAlphas << ")" << " nrVectorsComputed="
                         << nrVectorsComputed << endl;
#endif
            }
     
            // Add the resulting vectors to V (HV_n of (3.25))
            ValueFunctionPOMDPDiscrete Vtemp=VectorSetToValueFunction(Ga,a,betaI);
            for(Index k=0;k!=Vtemp.size();++k)
                Q[a].push_back(Vtemp[k]);

            Q[a]=Prune(Q[a]);
            CheckMaxNrVectors(maxNrAlphas,nrVectorsComputed+Q[a].size());
        }
        nrVectorsComputed+=Q[a].size();
    }

#if DEBUG_VERIFYBACKUPSTAGE
    QFunctionsDiscrete Qok=BackupStageSlow(Qs);
    for(GaoVectorSetIndex a=0;a!=nrA;a++)
    {
#if 0
        cout << "ok " << _m_doIncPrune << endl;
        for(Index i=0;i!=Qok.at(a).size();++i)
            cout << Qok.at(a).at(i).SoftPrint() << endl;
        cout << "new" << endl;
        for(Index i=0;i!=Q.at(a).size();++i)
            cout << Q.at(a).at(i).SoftPrint() << endl;
#endif   
        if(Qok.at(a).size()!=Q.at(a).size())
            throw(E("MonahanBGPlanner::BackupStage error"));
        for(Index i=0;i!=Qok.at(a).size();++i)
            if(!Qok.at(a).at(i).Equal(Q.at(a).at(i)))
                throw(E("MonahanBGPlanner::BackupStage error"));
    }
#endif

    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
            for(GaoVectorSetIndex aPrime=0;aPrime!=nrA;aPrime++) //for each next joint a
                delete G[a][o][aPrime];

    return(Q);
}

QFunctionsDiscrete
MonahanBGPlanner::BackupStageSlow(const QFunctionsDiscrete &Qs)
{
    int nrA=GetPU()->GetNrJointActions();
    QFunctionsDiscrete Qs1(nrA);

    GaobetaVectorSet G=BackProjectMonahanBG(Qs);

    // Do the cross-sums, results are stored in V1
    for(GaoVectorSetIndex a=0;a!=nrA;a++)
        MonahanCrossSum(G,Qs1,a,_m_doIncPrune);

    return(Qs1);
}


GaobetaVectorSet 
MonahanBGPlanner::
BackProjectMonahanBG(const QFunctionsDiscrete &Qs) const
{
    return(BackProjectMonahanBG(QFunctionsToValueFunction(Qs)));
}


GaobetaVectorSet 
MonahanBGPlanner::BackProjectMonahanBG(const ValueFunctionPOMDPDiscrete &V) const
{
    // Equation numbers refer to:
    // Frans A. Oliehoek, Nikos Vlassis, and Matthijs
    // T. J. Spaan. Properties of the QBG-value function. Technical Report
    // IAS-UVA-07-03, Informatics Institute, University of Amsterdam,
    // 2007.

    int nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrInV=V.size();
    double gamma=GetPU()->GetDiscount();

    //the set of 'regular' backprojected vectors (of form (3.14) in the QBG techrep)

    GaoVectorSet Gao=BackProject(V);

    boost::shared_ptr<BayesianGameIdenticalPayoff> bg_time_step=
        boost::shared_ptr<BayesianGameIdenticalPayoff>(new BayesianGameIdenticalPayoff(GetPU()->GetNrAgents(), 
                                                                                       GetPU()->GetNrActions(),
                                                                                       GetPU()->GetNrObservations()));
    JointPolicyPureVector jp(bg_time_step);
    //FRANS: what does the following line do? - "boost::extents" is just a way to specify the size of the boost::multi_array *
    //  (typedef boost::multi_array<VectorSet*,3> GaobetaVectorSet;)
    //the following allocates space for the different sets G_{a,o,\beta} (i.e., (3.15) of the QBG-techrep)
    GaobetaVectorSet G(boost::extents   [nrA]
                                        [nrO]
                                        [CastLIndexToIndex( bg_time_step->GetNrJointPolicies() )]
                                        );

    //a pointer used to point to different vectors in Gao
    VectorSet *vGao;
    //VectorSet v1(nrInV,nrS);

    // Create Gaobeta vectors of (3.15) of Technical Report IAS-UVA-07-03
    // FRANS:?!?! (3.15) does not creeat vectors, it specifies the set of vectors g in Gao that are consistent with BG policy \beta?!?! 
    //
    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
        {
            vGao=Gao[a][o];
            for(Index betaI=0;betaI!=bg_time_step->GetNrJointPolicies();++betaI) //for each joint BG policy (index)
            {
                vector<vector<double> > vBetaI;
                jp.SetIndex(betaI);
                //for the current a,o, beta, we are now going to construct a set of vectors...
                //  Note: the tech-report specifies the value for a (joint) belief. Therefore
                //  it is possible to select the maximizing back-projected vector (g_ao^v') in
                //  (3.22).
                //
                //  In contrast, here we are generating the the vectors over the complete 
                //  joint belief space. As such, we can not generate the vector 
                //  v*_{b,a,\beta} as done in (3.22). Instead we generate the set of possible
                //      v_{a,\beta}^k   = R_a + \sum_{o} g_{ao}^v'[k] 
                //                      = \sum_{o} [  (R_a / |o|) +  g_{ao}^v'[k]  ]      (A)
                //                                    ==========================
                //                                           
                //  
                //  (   Note that, because each \beta will specify at least one different joint action
                //      (for one joint observation), the set of v_{a,\beta}^k vectors generated for 
                //      different \beta are in fact different.                                            ) 
                //
                //!!HOWEVER!! these vectors that are called 'v1' below corresponds to the  
                //  part of (A) in brackets. It just adds (an 1/|o| fraction of) the immediate
                //  reward to  g_{ao}^v[k], which is just some back projected vector. Let us call 
                //  this new function 'rg'. I.e.:
                //
                //      rg_ao^k = (R_a / |o|) +  g_{ao}^v'[k]
                //
                //  while it is the case that each \beta will only be consistent with a subset of these
                //  vectors, it is *NOT* the case that each \beta specifies completely different sets of
                //  rg vectors!!!
                //
                //  As such, it does not make sense to have G[a][o][betaI] store sets of rg_ao^k,
                //  many of which contain duplicates!
                //
                //  INSTEAD I propose the following:
                //      1) store G[a][o][a'], the set of rg_ao^k such that v'[k] specifies a'
                //      2) then in 'cross-sum' only loop over the a' consistent with the jpol
                //
                for(int k=0;k!=nrInV;k++) //for each vector v[k] in V^t+1
                {
                    //if the vector V[k] specifies the joint action that is consistent with \beta (see (3.15))
                    if(V[k].GetAction()==jp.GetJointActionIndex(static_cast<Index>(o)))
                    {

                        vector<double> v1(nrS);
                        for(int s=0;s!=nrS;s++)
                        {
                            //the immediate reward is divided equally over the vectors:   
                            double R_s_a_o = (GetPU()->GetReward(s,a)) / nrO;

                            //the following is just one of the vectors g_ao^v[k] (g_{ao}^{v^{t+1}_a'}) from (3.15)
                            double g_a_o_vk_s =  (*vGao)(k,s);
                            v1.at(s) =  R_s_a_o + gamma * g_a_o_vk_s;
                        }
                        vBetaI.push_back(v1);
                    }
//                     else
//                         for(int s=0;s!=nrS;s++)
//                             v1(k,s)=0.0;
                }
//                cout << "vBetaI size " << vBetaI.size() << endl;
//                G[a][o][betaI]=new VectorSet(v1);
                G[a][o][betaI]=VectorOfVectorsToVectorSet(vBetaI);
            }
        }


    return(G);
}

void MonahanBGPlanner::MonahanCrossSum(const GaobetaVectorSet &G,
                                       QFunctionsDiscrete &Q,
                                       Index a,
                                       bool doIncPrune,
                                       size_t maxNrAlphas) const
{
    Index nrS=GetPU()->GetNrStates(),
        nrO=GetPU()->GetNrJointObservations(),
        nrBetaI=G.shape()[2];
//        nrInV=G[0][0][0]->size1();

    AlphaVector alpha(nrS);

#if DEBUG_AlphaVectorPlanning_CrossSum    
    cout << "AlphaVectorPlanning::MonahanCrossSum for action " << a << endl;
#endif

//     vector<VectorSet*> Gbeta;
//     for(Index o=0;o!=nrO;o++)
//     {
// //        VectorSet GbetaO(nrBetaI*nrInV,nrS);
//         vector<vector<double> > GbetaO;
//         Index j=0;
//         for(Index betaI=0;betaI!=nrBetaI;++betaI)
//             for(Index k=0;k!=G[a][o][betaI]->size1();++k)
//             {
//                 vector<double> Gtemp(nrS);
//                 for(Index s=0;s!=nrS;++s)
//                     Gtemp.at(s)=(*G[a][o][betaI])(k,s);
//                 j++;
//                 GbetaO.push_back(Gtemp);
//             }
//         Gbeta.push_back(VectorOfVectorsToVectorSet(GbetaO));
//     }

    for(Index betaI=0;betaI!=nrBetaI;++betaI)
    {
        VectorSet Ga=*G[a][0][betaI];//*(Gbeta[0]); 
        for(Index o=1;o!=nrO;o++)
        {
            if(doIncPrune)
            {
                VectorSet Ga2=CrossSum(Ga,*G[a][o][betaI]);//*(Gbeta[o]));
#if 0
                VectorSet Ga1=Prune(Ga2,maxNrAlphas);
                Ga=Ga1;
#else
                QFunctionsDiscrete Q;
                Q.push_back(VectorSetToValueFunction(Ga2));
                QFunctionsDiscrete Q1=Prune(Q);
                Ga=ValueFunctionToVectorSet(Q1[0]);
#endif
            }
            else
            {
                VectorSet Ga1=CrossSum(Ga,*G[a][o][betaI]);//*(Gbeta[o]));
                Ga=Ga1;
            }
#if DEBUG_AlphaVectorPlanning_CrossSum    
            if(maxNrAlphas)
                cout << "AlphaVectorPlanning::MonahanCrossSum nrAlphas " << Ga.size1()
                     << " (max " << maxNrAlphas << ")" << endl;
#endif
        
            if(maxNrAlphas && Ga.size1()>maxNrAlphas)
            {
                stringstream ss;
                ss << "AlphaVectorPlanning::MonahanCrossSum() too many alpha vectors "
                   << Ga.size1() << ">" << maxNrAlphas;
                throw(E(ss.str()));
            }
        }
     
        // Add the resulting vectors to V (HV_n of (3.25))
        for(Index k=0;k!=Ga.size1();++k)
        {
            alpha.SetAction(a);
            alpha.SetBetaI(betaI);
            for(Index s=0;s!=nrS;s++)
                alpha.SetValue(Ga(k,s),s);
            Q[a].push_back(alpha);
        }
        Q=Prune(Q);
    }

#if 0 // don't prune for BG testing
    if(!doIncPrune)
    {
        Q=Prune(Q);
        if(maxNrAlphas && Q.size()>maxNrAlphas)
        {
            stringstream ss;
            ss << "AlphaVectorPlanning::MonahanCrossSum() too many alpha vectors "
               << Q.size() << ">" << maxNrAlphas;
            throw(E(ss.str()));
        }
    }
#endif

//     for(Index k=0;k!=Gbeta.size();++k)
//         delete Gbeta[k];
}

ValueFunctionPOMDPDiscrete MonahanBGPlanner::GetValueFunction(size_t horizon)
{
    return(QFunctionsToValueFunction(_m_qFunction[horizon-1]));
}


/** This function need V for the following reason:
 *  the Gao set does not specify which action (aPrime) is taken at the next stage.
 *  However, because Gao[a][o][i] is the backprojection of V[i],
 *  we can use V to look up aPrime.
 */
VectorSet* MonahanBGPlanner::ComputeGaoa(const GaoVectorSet &Gao,
                                         const ValueFunctionPOMDPDiscrete &V,
                                         Index a,
                                         Index o,
                                         Index aPrime) const
{
    vector<vector<double> > v;
    //for the current a,o, beta, we are now going to construct a set of vectors...
    //  Note: the tech-report specifies the value for a (joint) belief. Therefore
    //  it is possible to select the maximizing back-projected vector (g_ao^v') in
    //  (3.22).
    //
    //  In contrast, here we are generating the the vectors over the complete 
    //  joint belief space. As such, we can not generate the vector 
    //  v*_{b,a,\beta} as done in (3.22). Instead we generate the set of possible
    //      v_{a,\beta}^k   = R_a + \sum_{o} g_{ao}^v'[k] 
    //                      = \sum_{o} [  (R_a / |o|) +  g_{ao}^v'[k]  ]      (A)
    //                                    ==========================
    //                                           
    //  
    //  (   Note that, because each \beta will specify at least one different joint action
    //      (for one joint observation), the set of v_{a,\beta}^k vectors generated for 
    //      different \beta are in fact different.                                            ) 
    //
    //!!HOWEVER!! these vectors that are called 'v1' below corresponds to the  
    //  part of (A) in brackets. It just adds (an 1/|o| fraction of) the immediate
    //  reward to  g_{ao}^v[k], which is just some back projected vector. Let us call 
    //  this new function 'rg'. I.e.:
    //
    //      rg_ao^k = (R_a / |o|) +  g_{ao}^v'[k]
    //
    //  while it is the case that each \beta will only be consistent with a subset of these
    //  vectors, it is *NOT* the case that each \beta specifies completely different sets of
    //  rg vectors!!!
    //
    //  As such, it does not make sense to have G[a][o][betaI] store sets of rg_ao^k,
    //  many of which contain duplicates!
    //
    //  INSTEAD I propose the following:
    //      1) store G[a][o][a'], the set of rg_ao^k such that v'[k] specifies a'
    //      2) then in 'cross-sum' only loop over the a' consistent with the jpol
    //

    //a pointer used to point to different vectors in Gao
    VectorSet *vGao=Gao[a][o];
    double gamma=GetPU()->GetDiscount();

    for(Index k=0;k!=V.size();k++) //for each vector v[k] in V^t+1
    {
        //if the vector V[k] specifies the joint action that is consistent with \beta (see (3.15))
        if(V[k].GetAction()==aPrime)
        {
            vector<double> v1(GetPU()->GetNrStates());
            for(Index s=0;s!=GetPU()->GetNrStates();s++)
            {
                //the immediate reward is divided equally over the vectors:   
                double R_s_a_o = (GetPU()->GetReward(s,a)) / GetPU()->GetNrJointObservations();
                
                //the following is just one of the vectors g_ao^v[k] (g_{ao}^{v^{t+1}_a'}) from (3.15)
                double g_a_o_vk_s =  (*vGao)(k,s);
                v1.at(s) =  R_s_a_o + gamma * g_a_o_vk_s;
            }
            v.push_back(v1);
        }
    }
    return(VectorOfVectorsToVectorSet(v));
}

GaobetaVectorSet MonahanBGPlanner::ComputeAllGaoa(
    const ValueFunctionPOMDPDiscrete &V) const
{
    int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations();

    //the set of 'regular' backprojected vectors (of form (3.14) in the QBG techrep)
    GaoVectorSet Gao=BackProject(V);

    //the following allocates space for the different sets G_{a,o,\beta} (i.e., (3.15) of the QBG-techrep)
    GaobetaVectorSet G(boost::extents[nrA]
                                     [nrO]
                                     [nrA]);

    for(GaoVectorSetIndex a=0;a!=nrA;a++) //for each joint action
        for(GaoVectorSetIndex o=0;o!=nrO;o++) //for each joint observation
        {
            for(GaoVectorSetIndex aPrime=0;aPrime!=nrA;aPrime++) //for each next joint action
            {
                VectorSet *Gaoa=ComputeGaoa(Gao,V,a,o,aPrime);
                if(Gaoa->size1()>1)
                {
                    VectorSet *GaoaPruned=new VectorSet(Prune(*Gaoa));
#if 0
                    cout << Gaoa->size1() << " " << GaoaPruned->size1() << endl;
#endif
                    G[a][o][aPrime]=GaoaPruned;
                    delete Gaoa;
                }
                else
                    G[a][o][aPrime]=Gaoa;
            }
        }

    // cleanup
    for(unsigned int a=0;a!=nrA;a++)
        for(unsigned int o=0;o!=nrO;o++)
            delete Gao[a][o];

    return(G);
}
