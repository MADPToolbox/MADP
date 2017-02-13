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

#include "AlphaVectorPlanning.h"
#include <float.h>
#include <fstream>
#include <limits>
#include <sys/times.h>

// for fork() and waitpid()
#include <unistd.h>
#include <sys/wait.h>

// for getrusage()
#include <sys/time.h>
#include <sys/resource.h>

#include "boost/numeric/ublas/matrix.hpp"
#include "boost/numeric/ublas/matrix_proxy.hpp"
#include "boost/numeric/ublas/io.hpp"

#include "AlphaVector.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "ValueFunctionPOMDPDiscrete.h"
#include "JointBeliefSparse.h"
#include "JointObservation.h"
#include "JointAction.h"
#include "State.h"
#include "argumentHandlers.h"
#include "QMDP.h"
#include "AlphaVectorPruning.h"

#define DEBUG_AlphaVectorPlanning_BeliefSampling 0
#define DEBUG_AlphaVectorPlanning_BackProject 0
#define DEBUG_AlphaVectorPlanning_BackProjectFullPrintout 0
#define DEBUG_AlphaVectorPlanning_BackProjectFullSanityCheck 0
#define DEBUG_AlphaVectorPlanning_Prune 0
#define DEBUG_AlphaVectorPlanning_CrossSum 0
#define DEBUG_AlphaVectorPlanning_ValueFunctionToQ 0
#define DEBUG_AlphaVectorPlanning_ImportValueFunction 0
#define DEBUG_AlphaVectorPlanning_ImportValueFunction 0
#define DEBUG_AlphaVectorPlanning_CheckPruning 0

#define AlphaVectorPlanning_CheckForDuplicates 1
#define AlphaVectorPlanning_UseUBLASinBackProject 1
#define AlphaVectorPlanning_VerifyUBLASinBackProject 0
#define AlphaVectorPlanning_UseFastSparseBackup 1

using namespace std;

AlphaVectorPlanning::AlphaVectorPlanning(const 
                                         PlanningUnitDecPOMDPDiscrete* pu) :
    _m_pu(pu),
    _m_TsForBackup(0),
    _m_OsForBackup(0),
    _m_eventOsForBackup(0),
    _m_TsOsForBackup(0),
    _m_acceleratedPruningThreshold(200)
{
    const TransitionModelMappingSparse *tms;
    const TransitionModelMapping *tm;
    const TransitionModelDiscrete *td=GetPU()->GetTransitionModelDiscretePtr();

    if((tms=dynamic_cast<const TransitionModelMappingSparse *>(td)))
        _m_useSparse=true;
    else if((tm=dynamic_cast<const TransitionModelMapping *>(td)))
        _m_useSparse=false;
    else 
        throw(E("AlphaVectorPlanning::Ctor() TransitionModelDiscretePtr not handled. Use the --cache-flat-models option."));

    _m_initialized=false;
}

AlphaVectorPlanning::AlphaVectorPlanning(const boost::shared_ptr<const 
                                         PlanningUnitDecPOMDPDiscrete> &pu) :
    _m_pu(0),
    _m_puShared(pu),
    _m_TsForBackup(0),
    _m_OsForBackup(0),
    _m_eventOsForBackup(0),
    _m_TsOsForBackup(0)
{
    const TransitionModelMappingSparse *tms;
    const TransitionModelMapping *tm;
    const TransitionModelDiscrete *td=GetPU()->GetTransitionModelDiscretePtr();

    if((tms=dynamic_cast<const TransitionModelMappingSparse *>(td)))
        _m_useSparse=true;
    else if((tm=dynamic_cast<const TransitionModelMapping *>(td)))
        _m_useSparse=false;
    else 
        throw(E("AlphaVectorPlanning::Ctor() TransitionModelDiscretePtr not handled. Use the --cache-flat-models option."));

    _m_initialized=false;
}

AlphaVectorPlanning::AlphaVectorPlanning(const 
                                         PlanningUnitFactoredDecPOMDPDiscrete* pu) :
    _m_TsForBackup(0),
    _m_OsForBackup(0),
    _m_eventOsForBackup(0),
    _m_TsOsForBackup(0)
{
  //cout << "AlphaVectorPlanning Constructor Factored Version 1" << endl;

  if(!(_m_pu = dynamic_cast<const PlanningUnitFactoredDecPOMDPDiscrete*>(pu)))
    throw(E("Cannot cast to PlanningUnitFactoredDecPOMDPDiscrete"));
}

AlphaVectorPlanning::AlphaVectorPlanning(const boost::shared_ptr<const 
                                         PlanningUnitFactoredDecPOMDPDiscrete> &pu) :
    _m_pu(0),
    _m_puShared(pu),
    _m_TsForBackup(0),
    _m_OsForBackup(0),
    _m_eventOsForBackup(0),
    _m_TsOsForBackup(0)
{
  //cout << "AlphaVectorPlanning Constructor Factored Version 2" << endl;
}

AlphaVectorPlanning::~AlphaVectorPlanning()
{
    DeInitialize();
}


void AlphaVectorPlanning::Initialize()
{
    if(_m_initialized)
        DeInitialize();

    const TransitionModelDiscrete *td=GetPU()->GetTransitionModelDiscretePtr();
    const ObservationModelDiscrete *od=GetPU()->GetObservationModelDiscretePtr();

    if(!(GetPU()->GetParams().GetEventObservability())) //standard, synchronous MADP
    { 
        if(_m_useSparse)
        {
            const TransitionModelMappingSparse *tms;
            tms=dynamic_cast<const TransitionModelMappingSparse *>(td);
            const ObservationModelMappingSparse *oms;
            oms=dynamic_cast<const ObservationModelMappingSparse *>(od);

#if AlphaVectorPlanning_UseFastSparseBackup
            for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
            {
                _m_Ts.push_back(tms->GetMatrixPtr(a));
                _m_Os.push_back(oms->GetMatrixPtr(a));

                _m_TsForBackup.push_back(vector<SparseVector *>());
                for(unsigned int s=0;s!=GetPU()->GetNrStates();s++)
                {
                    SparseVector *temp=new SparseVector(GetPU()->GetNrStates());
                    for(unsigned int s1=0;s1!=GetPU()->GetNrStates();s1++)
                    {
                        if((*_m_Ts[a])(s,s1)!=0)
                            (*temp)(s1)=(*_m_Ts[a])(s,s1);
                    }
                    _m_TsForBackup.at(a).push_back(temp);
                }

                _m_OsForBackup.push_back(vector<SparseVector *>());
                for(unsigned o=0;o!=GetPU()->GetNrJointObservations();++o)
                {
                    SparseVector *temp=new SparseVector(GetPU()->GetNrStates());
                    for(unsigned int s1=0;s1!=GetPU()->GetNrStates();s1++)
                    {
                        if((*_m_Os[a])(s1,o)!=0)
                            (*temp)(s1)=(*_m_Os[a])(s1,o);
                    }
                    _m_OsForBackup.at(a).push_back(temp);
                }

                _m_TsOsForBackup.push_back(vector< vector<SparseVector *> >());
                for(unsigned int s=0;s!=GetPU()->GetNrStates();s++)
                {
                    _m_TsOsForBackup.at(a).push_back(vector<SparseVector *>());
                    for(unsigned o=0;o!=GetPU()->GetNrJointObservations();++o)
                    {
                        SparseVector *temp=
                            new SparseVector(element_prod(*_m_TsForBackup[a][s],
                                                        *_m_OsForBackup[a][o]));
                        _m_TsOsForBackup.at(a).at(s).push_back(temp);
                    }
                }
            }
#endif
        }
        else
        {
            const TransitionModelMapping *tm;
            tm=dynamic_cast<const TransitionModelMapping *>(td);
            const ObservationModelMapping *om;
            om=dynamic_cast<const ObservationModelMapping *>(od);
            
            for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
            {
                _m_T.push_back(tm->GetMatrixPtr(a));
                _m_O.push_back(om->GetMatrixPtr(a));
            }
        }
    }
    else
    {
        if(_m_useSparse)
        {
            const TransitionModelMappingSparse *tms;
            tms=dynamic_cast<const TransitionModelMappingSparse *>(td);
            const EventObservationModelMappingSparse *oms;
            oms=dynamic_cast<const EventObservationModelMappingSparse *>(od);

#if AlphaVectorPlanning_UseFastSparseBackup
            for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
            {
                _m_Ts.push_back(tms->GetMatrixPtr(a));
                _m_Oes.push_back(vector<const EventObservationModelMappingSparse::SparseMatrix * >());

                _m_TsForBackup.push_back(vector<SparseVector *>());
                for(unsigned int s=0;s!=GetPU()->GetNrStates();s++)
                {
                    SparseVector *temp=new SparseVector(GetPU()->GetNrStates());
                    for(unsigned int s1=0;s1!=GetPU()->GetNrStates();s1++)
                    {
                        if((*_m_Ts[a])(s,s1)!=0)
                            (*temp)(s1)=(*_m_Ts[a])(s,s1);
                    }
                    _m_TsForBackup.at(a).push_back(temp);
                }

                _m_eventOsForBackup.push_back(vector< vector<SparseVector *> >());
                for(unsigned o=0;o!=GetPU()->GetNrJointObservations();++o)
                {
                    _m_Oes.at(a).push_back(oms->GetMatrixPtr(a,o));
                    
                    _m_eventOsForBackup.at(a).push_back(vector<SparseVector *>());

                    for(unsigned int s=0;s!=GetPU()->GetNrStates();s++)
                    {
                        SparseVector *temp=new SparseVector(GetPU()->GetNrStates());
                        for(unsigned int s1=0;s1!=GetPU()->GetNrStates();s1++)
                        {
                            if((*_m_Oes[a][o])(s,s1)!=0)
                                (*temp)(s1)=(*_m_Oes[a][o])(s,s1);
                        }
                        _m_eventOsForBackup.at(a).at(o).push_back(temp);
                    }
                }

                _m_TsOsForBackup.push_back(vector< vector<SparseVector *> >());
                for(unsigned int s=0;s!=GetPU()->GetNrStates();s++)
                {
                    _m_TsOsForBackup.at(a).push_back(vector<SparseVector *>());
                    for(unsigned o=0;o!=GetPU()->GetNrJointObservations();++o)
                    {
                        SparseVector *temp=
                            new SparseVector(element_prod(*_m_TsForBackup[a][s],
                                                          *_m_eventOsForBackup[a][o][s]));
                        _m_TsOsForBackup.at(a).at(s).push_back(temp);
                    }
                }
            }
#endif
        }
        else
        {
            const TransitionModelMapping *tm;
            tm=dynamic_cast<const TransitionModelMapping *>(td);
            const EventObservationModelMapping *ome;
            ome=dynamic_cast<const EventObservationModelMapping *>(od);
            for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
            {
                _m_T.push_back(tm->GetMatrixPtr(a));
                std::vector <const EventObservationModelMapping::Matrix* > S;
                for(unsigned int joi=0;joi!=GetPU()->GetNrJointObservations();++joi)
                    S.push_back(ome->GetMatrixPtr(a,joi));
                _m_Oe.push_back(S);
            }
        }
    }

    _m_initialized=true;
}

void AlphaVectorPlanning::DeInitialize()
{
    // don't try to deinitialize if we haven't initialized stuff, that
    // causes problems in Cassandra's code
    if(!_m_initialized)
        return;

    _m_T.clear();
    _m_O.clear();

    _m_Ts.clear();
    _m_Os.clear();

    if(_m_Oe.size())
    {
        for(size_t i = 0; i < _m_Oe.size(); ++i)
        {
	  _m_Oe.at(i).clear();
	}
	_m_Oe.clear();
    }

    if(_m_TsForBackup.size())
    {
        for(unsigned i=0;i!=_m_TsForBackup.size();++i)
            for(unsigned j=0;j!=_m_TsForBackup[i].size();++j)
                delete _m_TsForBackup[i][j];
    }
    _m_TsForBackup.clear();

    if(_m_OsForBackup.size())
    {
        for(unsigned i=0;i!=_m_OsForBackup.size();++i)
            for(unsigned j=0;j!=_m_OsForBackup[i].size();++j)
                delete _m_OsForBackup[i][j];
    }
    _m_OsForBackup.clear();

    if(_m_eventOsForBackup.size())
    {
        for(unsigned i=0;i!=_m_eventOsForBackup.size();++i)
            for(unsigned j=0;j!=_m_eventOsForBackup[i].size();++j)
                for(unsigned k=0;k!=_m_eventOsForBackup[i][j].size();++k)
                    delete _m_eventOsForBackup[i][j][k];
    }
    _m_eventOsForBackup.clear();
    
    if(_m_TsOsForBackup.size())
    {
        for(unsigned i=0;i!=_m_TsOsForBackup.size();++i)
            for(unsigned j=0;j!=_m_TsOsForBackup[i].size();++j)
                for(unsigned k=0;k!=_m_TsOsForBackup[i][j].size();++k)
                    delete _m_TsOsForBackup[i][j][k];
    }
    _m_TsOsForBackup.clear();

}

GaoVectorSet
AlphaVectorPlanning::BackProject(const ValueFunctionPOMDPDiscrete &v) const
{
    // convert the valuefunction (a set of AlphaVectors) to a
    // VectorSet, basically removing the action information
    int nrInV=v.size();
    if(nrInV>0)
    {
        int nrS=(v[0].GetValues()).size();
        VectorSet v1(nrInV,nrS);
        for(int k=0;k!=nrInV;k++)
	{
            for(int s=0;s!=nrS;s++)
                v1(k,s)=v[k].GetValue(s);
	}
        if(_m_useSparse)
            return(BackProjectSparse(v1));
        else
	    return(BackProjectFull(v1));
    }
    else
    {
        // v is empty, so return empty G
        GaoVectorSet G;
        return(G);
    }
}

/**
 * Implements equation (3.11) of PhD thesis Matthijs.
 */
GaoVectorSet AlphaVectorPlanning::BackProject(const VectorSet &v) const
{
#if 0
    TOIDecPOMDPDiscrete* toi=0;
    // check for TOI
    if((toi=const_cast<TOIDecPOMDPDiscrete*>(GetPU()->GetDPOMDPD()))!=0)
    {
        unsigned int nrJA=GetPU()->GetNrJointActions(),
            nrJO=GetPU()->GetNrJointObservations(),
            nrS=GetPU()->GetNrStates(),
            nrInV=v.size1();
        
        StartTimer("BackProjectTOI");

        GaoVectorSet G(boost::extents[nrA][nrO]);
        vector<GaoVectorSet> Gs;

        VectorSet v1(nrInV,nrS);
#if AlphaVectorPlanning_UseUBLASinBackProject
        VectorSet vv=v;
#endif
        double x;

        using namespace boost::numeric::ublas;

        for(unsigned int a=0;a!=nrJA;a++)
        {
            for(unsigned int o=0;o!=nrJO;o++)
            {
            for(unsigned int k=0;k!=nrInV;k++)
            {
                    for(unsigned int s=0;s!=nrS;s++)
                    {
                        x=0;
                        for(unsigned int s1=0;s1!=nrS;s1++)
                            x+=(*_m_O[a])(s1,o)*(*_m_T[a])(s,s1)*v(k,s1);
                        v1(k,s)=x;
                    }
            }
            G[a][o]=new VectorSet(v1);
        }

    StopTimer("BackProjectTOI");
        
    }
        else
#endif
    if(_m_useSparse)
        return(BackProjectSparse(v));
    else
        return(BackProjectFull(v));
}

/**
 * Implements equation (3.11) of PhD thesis Matthijs.
 */
GaoVectorSet AlphaVectorPlanning::BackProjectFull(const VectorSet &v) const
{
    unsigned int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates(),
        nrInV=v.size1();
    bool isEventDriven = GetPU()->GetParams().GetEventObservability();
    if(nrInV==0)
        throw(E("AlphaVectorPlanning::BackProjectFull attempting to backproject empty value function"));

#if DEBUG_AlphaVectorPlanning_BackProject
    tms timeStruct;
    clock_t ticks_before, ticks_after;
    ticks_before = times(&timeStruct);
#endif

    StartTimer("BackProjectFull");
    
    GaoVectorSet G(boost::extents[nrA][nrO]);
    VectorSet v1(nrInV,nrS);
#if AlphaVectorPlanning_UseUBLASinBackProject
    VectorSet vv=v;
#endif
    double x;

#if AlphaVectorPlanning_CheckForDuplicates
    vector<int> duplicates=GetDuplicateIndices(v);
#else
    vector<int> duplicates(nrInV,-1);
#endif
    int dup;

    using namespace boost::numeric::ublas;

    for(unsigned int a=0;a!=nrA;a++)
        for(unsigned int o=0;o!=nrO;o++)
        {
            for(unsigned int k=0;k!=nrInV;k++)
            {
                if(duplicates[k]==-1)
                {
#if AlphaVectorPlanning_UseUBLASinBackProject
                    const matrix_row<VectorSet> mV(vv,k);
#endif
                    for(unsigned int s=0;s!=nrS;s++)
                    {
#if AlphaVectorPlanning_UseUBLASinBackProject
                        matrix_row<const TransitionModelMapping::Matrix>
                            mT(*_m_T[a],s);
                        if(isEventDriven)
                        {
                            /// The distinction between row and column stems from the fact that
                            /// in a standard model, _m_O[a] is (s',o), while in an event-driven
                            /// model, _m_Oe[a][o] is (s,s') (because _m_T[a] is also (s,s'))
                            /// The distinction between row and column stems from the fact that
                            /// in a standard model, _m_O[a] is (s',o), while in an event-driven
                            /// model, _m_Oe[a][o] is (s,s') (because _m_T[a] is also (s,s'))
                            matrix_row<const ObservationModelMapping::Matrix> mO(*_m_Oe[a][o],s);
                            x=inner_prod(element_prod(mT,mO),mV);
                        }else{
                            matrix_column<const ObservationModelMapping::Matrix> mO(*_m_O[a],o);
                            x=inner_prod(element_prod(mT,mO),mV);
                        }
#if AlphaVectorPlanning_VerifyUBLASinBackProject
                        double x1=0;
                        for(unsigned int s1=0;s1!=nrS;s1++)
                            x1+=(*_m_O[a])(s1,o)*(*_m_T[a])(s,s1)*v(k,s1);
                        if(abs(x-x1)>1e-14)
                        {
                            cerr << x << " " << x1 << " " << x-x1 << endl;
                            abort();
                        }
#endif
#else // AlphaVectorPlanning_UseUBLASinBackProject
                        x=0;
                        for(unsigned int s1=0;s1!=nrS;s1++)
                            x+=(*_m_O[a])(s1,o)*(*_m_T[a])(s,s1)*v(k,s1);
#endif
                        v1(k,s)=x;
                    }
                }
                else
                {
                    dup=duplicates[k];
                    for(unsigned int s=0;s!=nrS;s++)
                        v1(k,s)=v1(dup,s);
                }
            }
            G[a][o]=new VectorSet(v1);
        }

    StopTimer("BackProjectFull");

#if DEBUG_AlphaVectorPlanning_BackProjectFullPrintout
    cout << "BackProjectFull of:" << endl;
    for(unsigned int k=0;k!=nrInV;k++)
    {
        for(unsigned int s=0;s!=nrS;s++)
            cout << v(k,s) << " ";
        cout << endl;
    }

    VectorSet *VS;
    for(unsigned int a=0;a!=nrA;a++)
        for(unsigned int o=0;o!=nrO;o++)
        {
            cout << "Gao a " << a << " o " << o << endl;

            VS=G[a][o];
            for(unsigned int k=0;k!=VS->size1();k++)
            {
                for(unsigned int s=0;s!=VS->size2();s++)
                    cout << (*VS)(k,s) << " ";
                cout << endl;
            }
        }
#endif

#if DEBUG_AlphaVectorPlanning_BackProjectFullSanityCheck
    double maxInV=-DBL_MAX;
    for(unsigned int k=0;k!=nrInV;k++)
        for(unsigned int s=0;s!=nrS;s++)
            maxInV=max(maxInV,v(k,s));

    double maxInGao=-DBL_MAX;
    for(unsigned int a=0;a!=nrA;a++)
        for(unsigned int o=0;o!=nrO;o++)
            for(unsigned int k=0;k!=nrInV;k++)
                for(unsigned int s=0;s!=nrS;s++)
                    maxInGao=max(maxInGao,(*G[a][o])(k,s));

    if(maxInGao>maxInV)
    {
        cout << "Max value in V is " << maxInV << ", max in Gao is "
             << maxInGao << endl;
        abort();
    }
#endif

#if DEBUG_AlphaVectorPlanning_BackProject
    ticks_after = times(&timeStruct);
    cout << "AlphaVectorPlanning::BackProject done in " 
         << ticks_after - ticks_before << " clock ticks, "
         << static_cast<double>((ticks_after - ticks_before))
        / sysconf(_SC_CLK_TCK) 
         << "s" << endl;
    // test results on 22-12-2006
    // using dense Matrix is about 2 faster than calling the Get*Prob()
    // using sparse Matrix is almost 3 times slower then dense Matrix
#endif
    return(G);
}

/**
 * Implements equation (3.11) of PhD thesis Matthijs.
 */
GaoVectorSet AlphaVectorPlanning::BackProjectSparse(const VectorSet &v) const
{
    unsigned int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates(),
        nrInV=v.size1();

    if(nrInV==0)
        throw(E("AlphaVectorPlanning::BackProjectSparse attempting to backproject empty value function"));

    StartTimer("BackProjectSparse");
    
    GaoVectorSet G(boost::extents[nrA][nrO]);
    VectorSet v1(nrInV,nrS);
    VectorSet vv=v;
    double x;

#if AlphaVectorPlanning_CheckForDuplicates
    vector<int> duplicates=GetDuplicateIndices(v);
#else
    vector<int> duplicates(nrInV,-1);
#endif
    int dup;

    using namespace boost::numeric::ublas;

    for(unsigned int a=0;a!=nrA;a++)
        for(unsigned int o=0;o!=nrO;o++)
        {
#if !AlphaVectorPlanning_UseFastSparseBackup
            matrix_column<const ObservationModelMappingSparse::SparseMatrix>
                mO(*_m_Os[a],o);
#endif
            for(unsigned int k=0;k!=nrInV;k++)
            {
                if(duplicates[k]==-1)
                {
                    const matrix_row<VectorSet> mV(vv,k);

                    for(unsigned int s=0;s!=nrS;s++)
                    {
#if AlphaVectorPlanning_UseFastSparseBackup
                        x=inner_prod(*_m_TsOsForBackup[a][s][o],mV);
#else
                        matrix_row<const TransitionModelMappingSparse::
                            SparseMatrix> mT(*_m_Ts[a],s);
                        x=inner_prod(element_prod(mT,mO),mV);
#endif
                        v1(k,s)=x;
                    }
                }
                else
                {
                    dup=duplicates[k];
                    for(unsigned int s=0;s!=nrS;s++)
                        v1(k,s)=v1(dup,s);
                }
            }
            G[a][o]=new VectorSet(v1);
        }

    StopTimer("BackProjectSparse");

    return(G);
}

BeliefSet AlphaVectorPlanning::SampleBeliefs(
    const ArgumentHandlers::Arguments &args) const
{
    StartTimer("SampleBeliefs");
    
    int resetAfter=args.resetAfter;
    int h=GetPU()->GetHorizon(),
        nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions(),
        i,d=0,s;
    Index a;
    Index o;
    Index s0=0,s1;
    JointBeliefInterface *b0p, *b1p;
    b0p = GetPU()->GetNewJointBeliefInterface(nrS);
    b1p = GetPU()->GetNewJointBeliefInterface(nrS);
    JointBeliefInterface& b0 = *b0p;
    JointBeliefInterface& b1 = *b1p;
    BeliefSet S(args.nrBeliefs);
    bool foundEqualBelief,equal,addBelief;
    int nrEqualFound=0;

    // we don't want to artificially reset the problem
    if(resetAfter==0)
    {
        if(h==static_cast<int>(MAXHORIZON))
            cout << "Warning: sampling beliefs for an infinite horizon "
                 << "without reset." << endl;
        resetAfter=h;
    }

    QMDP *qmdp=0;
    if(args.useQMDPforSamplingBeliefs)
    {
        qmdp=new QMDP(GetPU());
        qmdp->Compute();
    }

    i=0;
    // make sure we don't try to keep on sampling beliefs if there are
    // not enough unique beliefs (<args.nrBeliefs)
    while(i<args.nrBeliefs && nrEqualFound<(args.nrBeliefs*2))
    {
        // reset the problem if either we exceeded the horizon, or the
        // user-supplied parameter (used in the infinite-horizon case)
        if(d>h || d>resetAfter)
            d=0;

        if(d==0)
        {
            s1=GetPU()->GetDPOMDPD()->SampleInitialState();
            b1.Set(* GetPU()->GetProblem()->GetISD());
        }
        else
        {
            if(args.useQMDPforSamplingBeliefs &&
               (rand() / (RAND_MAX + 1.0)) > args.QMDPexploreProb)
            {
                double valMax=-DBL_MAX;
                a=INT_MAX;
                for(int aQMDP=0;aQMDP!=nrA;++aQMDP)
                {
                    double qQMDP=qmdp->GetQ(b0,aQMDP);
                    if(qQMDP>valMax)
                    {
                        valMax=qQMDP;
                        a=aQMDP;
                    }
                }
            }
            else
            {
                // sample an action uniformly at random
                a=static_cast<int>(nrA*(rand() / (RAND_MAX + 1.0)));
            }

            s1=GetPU()->GetDPOMDPD()->SampleSuccessorState(s0,a);
            if(GetPU()->GetParams().GetEventObservability())
            {
                o=GetPU()->GetDPOMDPD()->SampleJointObservation(s0,a,s1);
            }
            else
            {
                o=GetPU()->GetDPOMDPD()->SampleJointObservation(a,s1);
            }
            b1=b0;
            b1.Update(*GetPU()->GetDPOMDPD(),a,o);
        }

        if(args.uniqueBeliefs)
        {
            foundEqualBelief=false;
            // loop over all beliefs already in S
            for(int j=0;j!=i;j++)
            {
                equal=true;
                for(s=0;s!=nrS;s++)
                {
                    // if one number differs we can move on to the next j
                    if(S[j]->Get(s)!=b1.Get(s))
                    {
                        equal=false;
                        break;
                    }
                }
                // if we found an equal belief we can stop
                if(equal)
                {
                    foundEqualBelief=true;
                    break;
                }
            }
            if(foundEqualBelief)
            {
                nrEqualFound++;
                addBelief=false;
            }
            else
                addBelief=true;
        }
        else
            addBelief=true;
        
        if(addBelief)
        {
            if(!b1.SanityCheck())
                throw(E("AlphaVectorPlanning::BeliefSampling belief fails sanity check"));

            S[i]=b1.Clone();
            i++;

            if(DEBUG_AlphaVectorPlanning_BeliefSampling)
            {
                cout << "AlphaVectorPlanning::SampleBeliefs sampled belief nr "
                     << i << "/" << args.nrBeliefs << " (nrEqualFound " 
                     << nrEqualFound << ")" << endl;
            }
        }
        
        d++;
        s0=s1;
        b0=b1;
    }

    delete b0p;
    delete b1p;
    delete qmdp;

    StopTimer("SampleBeliefs");

    // we did not manage to sample args.nrBeliefs unique beliefs
    if(i<args.nrBeliefs)
    {
        cout << "AlphaVectorPlanning::SampleBeliefs: warning, only " 
             << "managed to sample " << i << " unique beliefs instead of " 
             << args.nrBeliefs << endl;
        BeliefSet S1(i);
        for(int j=0;j!=i;j++)
            S1[j]=S[j];
        return(S1);
    }
    else
        return(S);
}


BeliefSetNonStationary
AlphaVectorPlanning::SampleBeliefsNonStationary(
    const ArgumentHandlers::Arguments &args) const
{
    return(SampleBeliefsNonStationary(args.nrBeliefs,
                                      args.uniqueBeliefs,
                                      args.resetAfter,
                                      args.useQMDPforSamplingBeliefs,
                                      args.QMDPexploreProb));
}

BeliefSetNonStationary
AlphaVectorPlanning::SampleBeliefsNonStationary(
    int nrBeliefs,
    int uniqueBeliefs,
    int resetAfter,
    int useQMDPforSamplingBeliefs,
    double QMDPexploreProb) const
{
    StartTimer("SampleBeliefsNonStationary");
    
    int h=GetPU()->GetHorizon(),
        nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions(),
        i,d=0;
    Index a;
    Index o;
    Index s0=0,s1;
    JointBeliefInterface *b0p, *b1p;
    b0p = GetPU()->GetNewJointBeliefInterface(nrS);
    b1p = GetPU()->GetNewJointBeliefInterface(nrS);
    JointBeliefInterface& b0 = *b0p;
    JointBeliefInterface& b1 = *b1p;

    if(nrBeliefs<=h)
    {
        nrBeliefs=h+1;
        cout << "AlphaVectorPlanning::SampleBeliefsNonStationary need to sample at least one belief for each time step, sampling " << h+1 << " beliefs" << endl;
    }

    if(h==static_cast<int>(MAXHORIZON))
        throw(E("AlphaVectorPlanning::SampleBeliefsNonStationary does not make sense in an infinite-horizon setting"));

    BeliefSetNonStationary S(h);

    int nrEqualFound=0;

    // we don't want to artificially reset the problem
    if(resetAfter==0)
        resetAfter=h;

    i=0;
    // make sure we don't try to keep on sampling beliefs if there are
    // not enough unique beliefs (<nrBeliefs)
    while(i<nrBeliefs && nrEqualFound<(nrBeliefs*2))
    {
        // reset the problem if either we exceeded the horizon, or the
        // user-supplied parameter (used in the infinite-horizon case)
        if(d>h || d>resetAfter)
            d=0;

        if(d==0)
        {
            s1=GetPU()->GetDPOMDPD()->SampleInitialState();
            b1.Set(* GetPU()->GetProblem()->GetISD());
        }
        else
        {
            // sample an action uniformly at random
            a=static_cast<int>(nrA*(rand() / (RAND_MAX + 1.0)));

            s1=GetPU()->GetDPOMDPD()->SampleSuccessorState(s0,a);
            o=GetPU()->GetDPOMDPD()->SampleJointObservation(a,s1);

            b1=b0;
            b1.Update(*GetPU()->GetDPOMDPD(),a,o);
        }

        if(S.Add(d,b1,uniqueBeliefs)) // we add it
            i++;
        else
            nrEqualFound++;

        d++;
        s0=s1;
        b0=b1;
    }

    delete b0p;
    delete b1p;

    StopTimer("SampleBeliefsNonStationary");

    // we did not manage to sample nrBeliefs unique beliefs
    if(i<nrBeliefs)
    {
        cout << "Warning, only managed to sample " << i 
             << " unique beliefs instead of " << nrBeliefs << endl;
    }

    return(S);
}

VectorSet AlphaVectorPlanning::CrossSum(const VectorSet &A,
                                        const VectorSet &B) const
{
    int nrInA=A.size1(),
        nrInB=B.size1(),
        nrS=A.size2();

#if DEBUG_AlphaVectorPlanning_CrossSum
    cout << "AlphaVectorPlanning::CrossSum of " << nrInA 
         << " times " << nrInB << endl;
#endif

    VectorSet C(nrInA*nrInB,nrS);
    
    int k=-1;
    for(int i=0;i!=nrInA;i++)
        for(int j=0;j!=nrInB;j++)
        {
            k++;
            for(int s=0;s!=nrS;s++)
                C(k,s)=A(i,s)+B(j,s);
        }

    return(C);
}

void
AlphaVectorPlanning::
CrossSum( const std::vector< AlphaVector > &A,
          const std::vector< AlphaVector > &B,
          std::vector< AlphaVector > & output )
{
    output.clear();
    size_t nrInA = A.size(),
        nrInB = B.size();
    output.reserve( nrInA * nrInB );

#if DEBUG_AlphaVectorPlanning_CrossSum
    cout << "AlphaVectorPlanning::CrossSum of " << nrInA 
         << " times " << nrInB << endl;
#endif
    for(Index i=0; i!=nrInA; i++)
        for(Index j=0; j!=nrInB; j++)
            output.push_back( A[i].Add(B[j]) );
}

VectorSet 
AlphaVectorPlanning::
Union(const VectorSet &A, const VectorSet &B) const
{
    ValueFunctionPOMDPDiscrete V1=VectorSetToValueFunction(A);
    ValueFunctionPOMDPDiscrete V2=VectorSetToValueFunction(B);
    for(Index k=0;k!=V2.size();++k)
        V1.push_back(V2[k]);
    return(ValueFunctionToVectorSet(V1));
}


ValueFunctionPOMDPDiscrete 
AlphaVectorPlanning::
Prune(const ValueFunctionPOMDPDiscrete &V) const
{
    ValueFunctionPOMDPDiscrete V1;

    Timing time;

    if(V.size()>1000)
    {
        cout << "AlphaVectorPlanning::Prune() pruning a set of " << V.size()
             << " vectors";
        cout.flush();
        time.Start("Prune");
    }

    V1=AlphaVectorPruning::Prune(V,_m_acceleratedPruningThreshold);
       
#if DEBUG_AlphaVectorPlanning_CheckPruning

    ValueFunctionPOMDPDiscrete V1test;

    ExportValueFunction("/tmp/beforePruning",V,false);

    pid_t pID = fork();
    if (pID == 0)                // child
    {
        // Code only executed by child process
        stringstream ss;
        ss << V[0].GetNrValues();
        
        char *const pruneArgv[]={
            "/usr/local/bin/pomdp-tools",
            "-tool",
            "purge_alphas",
            "-alpha1",
            "/tmp/beforePruning",
            "-o",
            "/tmp/afterPruning",
            "-states",
            const_cast<char *>(ss.str().c_str()),
//            " > /dev/null",
            NULL};

        execvp(pruneArgv[0],
               pruneArgv);
    }
    else if(pID < 0)            // failed to fork
        throw(E("AlphaVectorPlanning::Prune failed to fork"));
    else                                   // parent
    {
        // parent waits for pruning to finish
        int status;
        wait(&status);
    }

    V1test=ImportValueFunction("/tmp/afterPruning");

    // because Cassandra's code does not know about BGPolicy
    // indices, we need to recover them now by looking up the
    // vector in the un-pruned value function
    for(Index k=0;k!=V1test.size();++k)
        for(Index j=0;j!=V.size();++j)
        {
            if(V[j].EqualValues(V1test[k]))
            {
                V1test[k].SetBetaI(V[j].GetBetaI());
                continue;
            }
        }

#if 0
    cout << "Value function MADP Pruning: " << endl;
    for(VFPDcit j=V1.begin();j!=V1.end();++j)
        cout << " a " << j->GetAction() << " bI " << j->GetBetaI() << " "
             << SoftPrintVector(j->GetValues());
    cout << endl;
    cout << "Value function POMDP-Tools Pruning: " << endl;
    for(VFPDcit j=V1test.begin();j!=V1test.end();++j)
        cout << " a " << j->GetAction() << " bI " << j->GetBetaI() << " "
             << SoftPrintVector(j->GetValues());
    cout << endl;
#endif

    // check if the value functions contains the same vectors, taking
    // into account that the order might be different
    if(V1.size()!=V1test.size())
    {
        cout << "MADP pruning: " << V1.size()
             << " vectors, pomdp-tools pruning: "
             << V1test.size() << " vectors" << endl;
        abort();
    }
    vector<bool> vectorMarked(V1test.size(),false);
    for(Index k=0;k!=V1test.size();++k)
    {
        bool vectorFound=false;
        for(Index j=0;j!=V1.size();++j)
        {
            if(vectorMarked[j]) // only match a vector a single time
                continue;
            if(vectorFound) // vector found, can stop looping
                break;

            if(V1test[k].GetAction() == V1[j].GetAction() &&
               V1test[k].GetBetaI() == V1[j].GetBetaI() && 
               V1test[k].EqualValues(V1[j]))
            {
                vectorFound=true;
                vectorMarked[j]=true;
            }
        }

        if(!vectorFound)
            abort();
    }

#endif

    if(V.size()>1000)
    {
        time.Stop("Prune");
        vector<double> times=time.GetEventDurations("Prune");
        cout << " (took " << times[0] << "s)." << endl;
    }

    return(V1);
}

QFunctionsDiscrete AlphaVectorPlanning::Prune(const QFunctionsDiscrete &Q) const
{
    QFunctionsDiscrete Q1(Q.size());
    size_t nrAlphas=0;

#if 0
    cout << "Calling Cassandra's pomdp-tools for pruning a Q-value function of size <";
    for(Index a=0;a!=Q.size();++a)
        cout << " " << Q[a].size();
    cout << " >" << endl;
#endif

    for(Index a=0;a!=Q.size();++a)
    {

#if DEBUG_AlphaVectorPlanning_Prune
//        cout << "AlphaVectorPlanning::Prune for action " << a << ": " << Q.size() << " vectors" << endl;
#endif
        Q1[a]=Prune(Q[a]);
        nrAlphas+=Q1[a].size();
        
    }

#if DEBUG_AlphaVectorPlanning_Prune
        cout << "AlphaVectorPlanning::Prune for action " << a << ": reduced "
             << Q[a].size() << " to " 
             << Q1[a].size()
             << " (total #vectors: " << nrAlphas << ")" << endl;
#endif

    return(Q1);
}

VectorSet AlphaVectorPlanning::Prune(const VectorSet &V) const
{
    // a bit cumbersome, but well...
    return(ValueFunctionToVectorSet(
               Prune(
                   VectorSetToValueFunction(V))));
}

void AlphaVectorPlanning::ExportValueFunction(const string & filename,
                                              const QFunctionsDiscrete &Q,
                                              bool includeBGindices)
{
    ExportValueFunction(filename,QFunctionsToValueFunction(Q),includeBGindices);
}

void 
AlphaVectorPlanning::
ExportValueFunction(const string & filename,
                    const QFunctionsDiscreteNonStationary &Q,
                    bool includeBGindices)
{
    for(Index t=0;t!=Q.size();++t)
    {
        stringstream filenameT;
        filenameT << filename << "_t" << t;
        ExportValueFunction(filenameT.str(),Q[t],includeBGindices);
    }
}

QFunctionsDiscreteNonStationary
AlphaVectorPlanning::ImportValueFunction(const string & filename, size_t nr,
                                         size_t nrA, size_t nrS)
{
    QFunctionsDiscreteNonStationary Q;
    for(Index t=0;t!=nr;++t)
    {
        stringstream filenameT;
        filenameT << filename << "_t" << t;
        Q.push_back(ValueFunctionToQ(
                        ImportValueFunction(filenameT.str()),nrA,nrS));
    }
    return(Q);
}

/** Export is in a simple text format used by Tony Cassandra for
 * .alpha files. The format is simply:
 *
 * <action>
 * <list of vector components>
 * 
 * <action>
 * <list of vector components>
 */
void
AlphaVectorPlanning::ExportValueFunction(const string & filename,
                                         const ValueFunctionPOMDPDiscrete &V,
                                         bool includeBGindices)
{
    vector<double> values;

    ofstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "AlphaVectorPlanning::ExportValueFunction: failed to "
             << "open file " << filename << endl;            
    }

    // floating point precision must be digits10 + 2, to ensure that conversions are correct
    int maxPrecision = numeric_limits<double>::digits10 + 2;
    fp.precision(maxPrecision);

    for(unsigned int i=0;i!=V.size();i++)
    {
        values=V[i].GetValues();
        int nrS=values.size();

        fp << V[i].GetAction();
        // this breaks compatability with Cassandra's software
        if(includeBGindices)
            fp << " " << V[i].GetBetaI();
        fp << endl;

        for(int s=0;s!=nrS;s++)
        {
            fp << values[s];
            if(s<(nrS-1))
                fp << " ";
        }
        fp << endl << endl;
    }
}

/** Function lacks error checking. */
ValueFunctionPOMDPDiscrete 
AlphaVectorPlanning::ImportValueFunction(const string & filename)
{
    ValueFunctionPOMDPDiscrete V;

    int lineState=0; /* lineState=0 -> read action
                      * lineState=1 -> read values
                      * lineState=2 -> empty line, skip */
    int nrStates=-1;
    bool first=true;
    Index action=0;
    AlphaVector::BGPolicyIndex betaI=-1;
    double value;
    vector<double> values;
    vector<AlphaVector::BGPolicyIndex> actionBetaI;
    AlphaVector::BGPolicyIndex actionOrBetaI;

    ifstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "AlphaVectorPlanning::ImportValueFunction: failed to "
             << "open file " << filename << endl;            
    }

    string buffer;
    while(!getline(fp,buffer).eof())
    {
        switch(lineState)
        {
        case 0:
            // read action
//            action=strtol(buffer,NULL,10);
            actionBetaI.clear();
            {
                istringstream is(buffer);
                while(is >> actionOrBetaI)
                    actionBetaI.push_back(actionOrBetaI);
            }

            switch(actionBetaI.size())
            {
            case 1:
                action=CastLIndexToIndex(actionBetaI[0]);
                betaI=-1;
                break;
            case 2:
                action=CastLIndexToIndex(actionBetaI[0]);
                betaI=actionBetaI[1];
                break;
            default:
                throw(E("AlphaVectorPlanning::ImportValueFunction parse error"));
            }

            lineState++;
            break;
        case 1:
            // read values
            values.clear();

            {
                istringstream is(buffer);
                while(is >> value)
                    values.push_back(value);
            }
            
            if(first)
            {
                nrStates=values.size();
                first=false;
            }
            
            // create new alpha vector and store it
            {
                AlphaVector alpha(nrStates);
                alpha.SetAction(action);
                alpha.SetValues(values);
                // only set betaI if different than default, to avoid
                // setting it to -1
                if(betaI!=alpha.GetBetaI())
                    alpha.SetBetaI(betaI);

#if DEBUG_AlphaVectorPlanning_ImportValueFunction
                cout << "AlphaVectorPlanning::ImportValueFunction "
                     << "added vector " << V.size() << " for action " 
                     << action << " betaI " << betaI << endl;
#endif

                V.push_back(alpha);
            }

            lineState++;
            break;
        case 2:
            // do nothing, line is empty
            lineState=0;
            break;
        }
    }

    return(V);
}

ValueFunctionPOMDPDiscrete
AlphaVectorPlanning::
GetImmediateRewardValueFunction() const
{
    return(GetImmediateRewardValueFunction(GetPU()));
}

ValueFunctionPOMDPDiscrete
AlphaVectorPlanning::
GetImmediateRewardValueFunction(const PlanningUnitDecPOMDPDiscrete *pu)
{
    size_t nrA=pu->GetNrJointActions(),
        nrS=pu->GetNrStates();
    ValueFunctionPOMDPDiscrete V0;
    AlphaVector alpha(nrS);

    for(Index a=0;a<nrA;a++)
    {
        alpha.SetAction(a);
        for(Index s=0;s<nrS;s++)
            alpha.SetValue(pu->GetReward(s,a),s);
        V0.push_back(alpha);
    }

    return(V0);
}

ValueFunctionPOMDPDiscrete
AlphaVectorPlanning::QFunctionsToValueFunction(const 
                                               QFunctionsDiscrete &Q)
{
    //Cryptic shit...
    //
    //a QFunctionsDiscrete is a typedef defined in ValueFunctionPOMDPDiscrete:
    //  "A QFunctionsDiscrete is just a vector of                           "
    //  "ValueFunctionPOMDPDiscrete, one for each action.                   "
    //  "typedef std::vector<ValueFunctionPOMDPDiscrete> QFunctionsDiscrete;"
    //  "typedef QFunctionsDiscrete::const_iterator QFDcit;                 "
    //and those are a typedef in turn:
    //  "A ValueFunctionPOMDPDiscrete is just a vector of AlphaVector's.    "
    //  "typedef std::vector<AlphaVector> ValueFunctionPOMDPDiscrete;       "
    //  "typedef ValueFunctionPOMDPDiscrete::const_iterator VFPDcit;        "
    //
    ValueFunctionPOMDPDiscrete V;
    for(QFDcit i=Q.begin();i!=Q.end();++i)
        // (*i) isa ValueFunctionPOMDPDiscrete isa vector of AlphaVectors
        for(VFPDcit j=i->begin();j!=i->end();++j)
            // (*j) isa AlphaVector 
            V.push_back(*j);
    
    return(V);
}

void AlphaVectorPlanning::ExportPOMDPFile(const string & filename) const
{
    ExportPOMDPFile(filename,GetPU()->GetDPOMDPD());
}

/// Export is in Tony Cassandra's POMDP file format.
void AlphaVectorPlanning::ExportPOMDPFile(const string & filename,
                                          const DecPOMDPDiscreteInterface *decpomdp)
{
    int nrA=decpomdp->GetNrJointActions(),
        nrO=decpomdp->GetNrJointObservations(),
        nrS=decpomdp->GetNrStates();
    ofstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "AlphaVectorPOMDP::ExportPOMDPFile: failed to open file "
             << filename << endl;            
    }

    fp << "discount: " << decpomdp->GetDiscount() << endl;
    switch(decpomdp->GetRewardType())
    {
    case REWARD:
        fp << "values: reward" << endl;
        break;
    case COST:
        fp << "values: cost" << endl;
    }

    // Tony's parser chokes on "_" in names of
    // states/actions/observations, so we just remove them from the
    // strings.
    // Also, we have to ensure that strings don't start with a number,
    // which we solve by prepending each string with s/a/o.
    fp << "states:";
    for(int s=0;s<nrS;s++)
    {
        string state="s" + decpomdp->GetState(s)->SoftPrintBrief();
        state.erase(remove(state.begin(), state.end(), '_'),state.end());
        fp << " "  << state;
    }
    fp << endl;

    fp << "actions:";
    for(int a=0;a<nrA;a++)
    {
        string action="a" + decpomdp->GetJointAction(a)->SoftPrintBrief();
        action.erase(remove(action.begin(), action.end(), '_'),action.end());
        fp << " "  << action;
    }
    fp << endl;

    fp << "observations:";
    for(int o=0;o<nrO;o++)
    {
        string obs="o" + decpomdp->GetJointObservation(o)->SoftPrintBrief();
        obs.erase(remove(obs.begin(), obs.end(), '_'),obs.end());
        fp << " "  << obs;
    }
    fp << endl;


    StateDistribution* isd = decpomdp->GetISD();
    fp << "start: ";
    for(int s0=0;s0<nrS;s0++)
    {
        double bs = isd->GetProbability(s0);
        fp <<  bs << " ";
    }
    fp << endl;

    delete isd;

    double p;
    for(int a=0;a<nrA;a++)
        for(int s0=0;s0<nrS;s0++)
            for(int s1=0;s1<nrS;s1++)
            {
                p=decpomdp->GetTransitionProbability(s0,a,s1);
                if(p!=0)
                    fp << "T: " << a << " : " << s0 << " : " << s1 << " " 
                       << p << endl;
            }

    for(int a=0;a<nrA;a++)
        for(int o=0;o<nrO;o++)
            for(int s1=0;s1<nrS;s1++)
            {
                p=decpomdp->GetObservationProbability(a,s1,o);
                if(p!=0)
                    fp << "O: " << a << " : " << s1 << " : " << o << " " 
                       << p << endl;
            }

    for(int a=0;a<nrA;a++)
        for(int s0=0;s0<nrS;s0++)
        {
            p=decpomdp->GetReward(s0,a);
            if(p!=0)
                fp << "R: " << a << " : " << s0 << " : * : * "
                   << p << endl;
        }

}

void AlphaVectorPlanning::ExportBeliefSet(const BeliefSet &B,
                                          const string &filename)
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "AlphaVectorPlanning::ExportBeliefSet: failed to open file "
           << filename;
        throw(E(ss.str()));
    }

    int nrB=B.size();
    for(int b=0;b!=nrB;b++)
    {
        for(unsigned int s=0;s!=B[b]->Size();s++)
            fp << B[b]->Get(s) << " ";
        fp << endl;
    }
}

QFunctionsDiscrete
AlphaVectorPlanning::ValueFunctionToQ(const ValueFunctionPOMDPDiscrete &V) const
{
    return(ValueFunctionToQ(V,GetPU()->GetNrJointActions(),
                            GetPU()->GetNrStates()));
}

QFunctionsDiscrete
AlphaVectorPlanning::ValueFunctionToQ(const ValueFunctionPOMDPDiscrete &V,
                                      size_t nrA, size_t nrS)
{
    QFunctionsDiscrete Qs;

    for(Index a=0;a!=nrA;a++)
    {
        ValueFunctionPOMDPDiscrete Q;
        for(Index i=0;i!=V.size();i++)
        {
            if(V[i].GetAction()==a)
                Q.push_back(V[i]);
        }
        // if the action has no vector, it's dominated everywhere, so
        // must never be chosen
        if(Q.size()==0)
        {
            AlphaVector dominatedVector(nrS,-DBL_MAX);
            dominatedVector.SetAction(a);
            Q.push_back(dominatedVector);
#if DEBUG_AlphaVectorPlanning_ValueFunctionToQ
            cout << "AlphaVectorPlanning::ValueFunctionToQ: action " << a 
                 << " is dominated" << endl;
#endif
        }
        else
        {
#if DEBUG_AlphaVectorPlanning_ValueFunctionToQ
            cout << "AlphaVectorPlanning::ValueFunctionToQ: action " << a 
                 << " has " << Q.size() 
                 << " vector(s) " << endl;
#endif
        }
        Qs.push_back(Q);
    }
    
    return(Qs);
}

/** Returns a vector<int> which for each vector k in V specifies
 * whether it is unique, in which case vector<int>[k] is set to -1, or
 * whether it is a duplicate of another vector l in V, in which case
 * vector<int>[k] is set to the index of l, where 0 <= l <
 * V.size()). */
vector<int> AlphaVectorPlanning::GetDuplicateIndices(const VectorSet &V)
{
    int nrInV=V.size1(), nrS=V.size2();
    bool equal;

    vector<int> duplicates(nrInV,-1);

    for(int i=1;i!=nrInV;++i) // start at 1, first is never a duplicate
    {
        for(int j=0;j!=i;++j) // loop over all previous vectors
        {
            equal=true;
            for(int s=0;s!=nrS;++s)
            {
                if(abs(V(i,s)-V(j,s))>PROB_PRECISION)
                {
                    equal=false;
                    break; // if 1 number differs, they are not equal
                }
            }
            if(equal)
            {
                duplicates[i]=j;
                break; // we need to find only the first duplicate
            }
        }
    }

#if 0 // reduce verbosity
    PrintVectorCout(duplicates); cout << endl;
#endif
    return(duplicates);
}

bool AlphaVectorPlanning::
VectorIsInValueFunction(const AlphaVector &alpha,
                        const ValueFunctionPOMDPDiscrete &V)
{
    bool equal;

    for(VFPDcit it=V.begin(); it!=V.end(); ++it)
    {
        equal=true;
        // if the actions differ we don't need to check the values
        if(alpha.GetAction()!=it->GetAction())
        {
            equal=false;
            continue;
        }

        for(unsigned i=0; i!=alpha.GetNrValues(); ++i)
            if(abs(alpha.GetValue(i)-it->GetValue(i))>REWARD_PRECISION)
            {
                equal=false;
                continue;
            }

        if(equal)
            return(true);
    }

    // if we get here we did not find a match, so alpha is not in V
    return(false);
}

bool AlphaVectorPlanning::
VectorIsDominated(const AlphaVector &alpha,
                  const ValueFunctionPOMDPDiscrete &V)
{
    for(ValueFunctionPOMDPDiscrete::const_iterator it=V.begin();
        it!=V.end();
        ++it)
    {
        bool alphaIsDominated=true;
        for(Index s=0;s!=alpha.GetNrValues();++s)
            if(alpha.GetValue(s) > it->GetValue(s))
                alphaIsDominated=false;

        if(alphaIsDominated)
            return(true);
    }

    // if alpha was not dominated by any vector in V, it is not dominated
    return(false);
}

bool AlphaVectorPlanning::
VectorIsDominated(Index i,
                  const VectorSet &V,
                  const vector<bool> &vectorsInVtoConsider)
{
    if(V.size1() !=
       vectorsInVtoConsider.size())
        throw(E("AlphaVectorPlanning::VectorIsDominated dimension mismatch"));

    for(Index j=0; j!=V.size1(); ++j)
    {
        if(j==i)
            continue;
        if(vectorsInVtoConsider[j]==false)
            continue;

        bool alphaIsDominated=true;
        for(Index s=0;s!=V.size2();++s)
            if(V(i,s) > V(j,s))
                alphaIsDominated=false;

        if(alphaIsDominated)
            return(true);
    }

    // if alpha was not dominated by any vector in V, it is not dominated
    return(false);
}

VectorSet AlphaVectorPlanning::ValueFunctionToVectorSet(const ValueFunctionPOMDPDiscrete& V)
{
    if(V.size()==0)
        return(VectorSet());
    else
    {
        size_t nrS=V[0].GetNrValues();
        VectorSet VS(V.size(),nrS);
        for(Index i=0;i!=V.size();++i)
            for(Index s=0;s!=nrS;++s)
                VS(i,s)=V[i].GetValue(s);
        return(VS);
    }
}

ValueFunctionPOMDPDiscrete
AlphaVectorPlanning::VectorSetToValueFunction(const VectorSet& VS,
                                              Index a,
                                              AlphaVector::BGPolicyIndex betaI)
{
    ValueFunctionPOMDPDiscrete V;

    for(Index i=0;i!=VS.size1();++i)
    {
        AlphaVector alpha(VS.size2());
        alpha.SetAction(a);
        alpha.SetBetaI(betaI);
        for(Index s=0;s!=VS.size2();++s)
            alpha.SetValue(VS(i,s),s);
        V.push_back(alpha);
    }
    return(V);
}

VectorSet* AlphaVectorPlanning::VectorOfVectorsToVectorSet(
    const vector<vector<double> > &vectors)
{
    size_t nrInV=vectors.size();
    VectorSet *V=0;
    if(nrInV==0)
    {
        V=new VectorSet();
    }
    else
    {
        size_t nrD=vectors.at(0).size();
        V=new VectorSet(nrInV,nrD);
        for(Index k=0;k!=nrInV;++k)
        {
            if(vectors.at(k).size()!=nrD)
                throw(E("AlphaVectorPlanning::VectorOfVectorsToVectorSet: all vectors must have the same size."));
            for(Index s=0;s!=nrD;++s)
                (*V)(k,s)=vectors.at(k).at(s);
        }
    }

    return(V);
}

string AlphaVectorPlanning::SoftPrint(const VectorSet &VS)
{
    stringstream ss;

    for(Index i=0;i!=VS.size1();++i)
    {
        ss << "[";
        for(Index j=0;j!=VS.size2();++j)
        {
            ss << VS(i,j);
            if(j==VS.size2()-1)
                ss << "]";
            else
                ss << " ";
        }
    }
    return(ss.str());
}

// this cannot be called "Equal", as Cassandra's code already defines a
// macro called Equal...
bool AlphaVectorPlanning::EqualVS(const VectorSet &VS1, const VectorSet &VS2)
{
    if(VS1.size1()!=VS2.size1())
        return(false);
    if(VS1.size2()!=VS2.size2())
        return(false);
    for(Index i=0;i!=VS1.size1();++i)
        for(Index j=0;j!=VS1.size2();++j)
            if(!Globals::EqualReward(VS1(i,j),
                                     VS2(i,j)))
                return(false);

    // if we made it here the sets are equal
    return(true);
}

size_t AlphaVectorPlanning::GetAcceleratedPruningThreshold() const
{
    return(_m_acceleratedPruningThreshold);
}

void AlphaVectorPlanning::SetAcceleratedPruningThreshold(size_t acceleratedPruningThreshold)
{
    _m_acceleratedPruningThreshold=acceleratedPruningThreshold;
}
