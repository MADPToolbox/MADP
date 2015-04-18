/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

#include <list>

#include "BayesianGameWithClusterInfo.h"
#include "BeliefIteratorGeneric.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "JointPolicyDiscretePure.h"
#include "JointActionObservationHistoryTree.h"
#include "JointActionObservationHistory.h"
#include "ObservationHistoryTree.h"
#include "ObservationHistory.h"
#include "JointBeliefInterface.h"
#include "QFunctionJAOHInterface.h"
#include "QFunctionJointBeliefInterface.h"
#include "TypeCluster.h"
#include "Type_AOHIndex.h"
#include "Type_PointerTuple.h"
#include "QHybrid.h"
#include "boost/enable_shared_from_this.hpp"

using namespace std;

#define BGCLUSTER_OUTPUT_TESTEQUIVALENCE 0
#define BGCLUSTER_REMOVE_ZEROPROB_TYPES 1

//Default constructor
BayesianGameWithClusterInfo::BayesianGameWithClusterInfo(                
        const PlanningUnitDecPOMDPDiscrete* pu,
        const QFunctionJAOHInterface* q,
        const boost::shared_ptr<const PartialJointPolicyDiscretePure> &pastJPol,
        BGClusterAlgorithm clusterAlg
    )
        :
            BayesianGameForDecPOMDPStage(  pu, q, pastJPol ),
            _m_pBG()
            ,_m_qJB(dynamic_cast<const QFunctionJointBeliefInterface *>(q))
            ,_m_qHybrid(dynamic_cast<const QHybrid *>(q))
            ,_m_pBGJPol()
            ,_m_clusterAlgorithm(clusterAlg)
{
    //extra stuff we need to do for cluster info...


    //Construct the individual type sets
    Index ts = GetStage();
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        _m_typeLists.push_back( new TypeClusterList() );
        Index tcI = 0;
        Index fI = CastLIndexToIndex(GetPUDecPOMDPDiscrete()->GetFirstObservationHistoryIndex(agI, ts));
        for(Index typeI=0; typeI < GetNrTypes(agI); typeI++)
        {
            //the type corresponds to the observation history, offset by the
            //first ohI index for this stage
            Index ohI = typeI + fI;
            
            vector<Index> actions;
            vector<Index> observations;
            if(ts > 0)
            {
                Index obsArr[ts];
                Index acsArr[ts];
                GetPUDecPOMDPDiscrete()->GetObservationHistoryArrays(agI, ohI, ts, obsArr);
                Index ohI2 = 0;
                Index tI = 0;
                while(tI < ts)
                {
                    Index aI = pastJPol->GetActionIndex(agI, ohI2);
                    acsArr[tI] = aI;

                    Index next_oI = obsArr[tI];
                    ohI2 = GetPUDecPOMDPDiscrete()->GetSuccessorOHI(agI, ohI2, next_oI);
                    tI++;
                }
                actions = vector<Index>(acsArr[0], acsArr[ts-1]);
                observations = vector<Index>(obsArr[0], obsArr[ts-1]);
            }


            Index aohI = GetPUDecPOMDPDiscrete()->GetActionObservationHistoryIndex(agI, ts, 
                    actions, observations);
            Type_AOHIndex* t =  new Type_AOHIndex(aohI);
            TypeCluster* tc = new TypeCluster(tcI++);
            tc->AddType(t);
            ///store this type:
            _m_typeLists.at(agI)->push_back( tc );
        }
    }

    //on initialization we called BayesianGameForDecPOMDPStage(  pu, q, pastJPol ),
    //which is the public constructor of BayesianGameForDecPOMDPStage, which already
    //computed all joint types.
    //_m_JBs = vector< JointBeliefInterface* >(GetNrJointTypes());
    _m_jaohReps = vector< Index >( GetNrJointTypes() );

    //loop over joint types and store the joint beliefs.
    for(Index jtI=0; jtI < GetNrJointTypes(); jtI++)
    {
        //compute the jaohI
        const vector<Index>& indTypeIs = JointToIndividualTypeIndices(jtI);
        vector<Index> indAOHs(GetNrAgents());
        for(Index agI=0; agI < GetNrAgents(); agI++)
        {
            Index tI = indTypeIs.at(agI);
            TypeClusterList* tcl = _m_typeLists.at(agI);
            TypeCluster* tc =  tcl->at(tI);
            //we know there is just 1 element in this cluster (we just created
            //it above and only added 1 Type_AOHIndex )
            Type* t1 =  *(tc->begin());
            Type_AOHIndex* t =  dynamic_cast< Type_AOHIndex * >(t1);
            Index aohI = t->GetAOHIndex();
            indAOHs.at(agI) = aohI;
        }
        Index jaohI = IndividualToJointTypeIndices(indAOHs);
        _m_jaohReps.at(jtI) = jaohI;
        //the following returns a copy that should be deleted:
        //JointBeliefInterface* jb = GetPUDecPOMDPDiscrete()->GetJointBeliefInterface(jaohI);
        //_m_JBs.at(jtI) = jb;
    }


}

BayesianGameWithClusterInfo::BayesianGameWithClusterInfo(                
        const PlanningUnitDecPOMDPDiscrete* pu
    )
        :
    BayesianGameForDecPOMDPStage(  pu )
    , _m_pBG()
    , _m_qJB(0)
    , _m_qHybrid(0)
    , _m_pBGJPol()
    , _m_typeLists(pu->GetNrAgents(), 0)
{
    size_t nrAgents = pu->GetNrAgents();
    for(Index agI=0; agI < nrAgents; agI++)
    {
        //remember things are defined as follows:
        //typedef std::vector<TypeCluster* > TypeClusterList
        //std::vector< TypeClusterList* > _m_typeLists;
        Index typeIndex = 0;
        TypeCluster* tc = new TypeCluster( typeIndex );
        TypeClusterList* tcl =  new TypeClusterList(1, tc);
        _m_typeLists.at(agI) = tcl;
    }
}


BayesianGameWithClusterInfo::BayesianGameWithClusterInfo(                
        const PlanningUnitDecPOMDPDiscrete* pu,
        const QFunctionJAOHInterface* q,
        Index t,
        const BGwCI_constPtr &prevBG,
        const boost::shared_ptr<const JointPolicyDiscretePure> &prevJPolBG,
        size_t nrAgents,
        const vector<size_t>& nrActions,
        const vector<size_t>& nrTypes,
        BGClusterAlgorithm clusterAlg
    )
        :

            BayesianGameForDecPOMDPStage( 
                    pu, q, t, nrAgents, nrActions, nrTypes)
            ,_m_pBG(prevBG)
            ,_m_qJB(dynamic_cast<const QFunctionJointBeliefInterface *>(q))
            ,_m_qHybrid(dynamic_cast<const QHybrid *>(q))
            ,_m_pBGJPol(prevJPolBG)
            //,_m_JBs( GetNrJointTypes() )
            ,_m_jaohReps( GetNrJointTypes() )
            ,_m_clusterAlgorithm(clusterAlg)
{
}

//Copy constructor.    
BayesianGameWithClusterInfo::BayesianGameWithClusterInfo(const BayesianGameWithClusterInfo& o) :
        BayesianGameForDecPOMDPStage( o )
{
    _m_pBG = o._m_pBG;
    _m_qJB = o._m_qJB;
    _m_qHybrid = o._m_qHybrid;
    _m_pBGJPol = o._m_pBGJPol;
    _m_jaohReps = o._m_jaohReps;
    _m_clusterAlgorithm = o._m_clusterAlgorithm;
    _m_typeLists= std::vector< TypeClusterList* > (o._m_typeLists.size(),0);
    for(Index k=0;k!=o._m_typeLists.size();++k)
    {
        if(o._m_typeLists.at(k))
        {
            _m_typeLists.at(k)=new TypeClusterList(*o._m_typeLists.at(k));
            for(Index j=0;j!=o._m_typeLists.at(k)->size();++j)
                _m_typeLists.at(k)->at(j)=new TypeCluster(*o._m_typeLists.at(k)->at(j));
        }
    }
}

//Destructor
BayesianGameWithClusterInfo::~BayesianGameWithClusterInfo()
{
    //free all the type information:
    std::vector< TypeClusterList* >::iterator it1 =  _m_typeLists.begin();
    std::vector< TypeClusterList* >::iterator last1 =  _m_typeLists.end();
    while(it1 != last1)
    {
        TypeClusterList* aohl = *it1;
        //TypeClusterList defined as std::vector<TypeCluster* > 
         std::vector<TypeCluster* >::iterator it2 = aohl->begin();
         std::vector<TypeCluster* >::iterator last2 = aohl->end();
         while(it2 != last2)
         {
             delete *it2;
             it2++;
         }            
        delete aohl;
        it1++;
    }
}

//Copy assignment operator
BayesianGameWithClusterInfo& BayesianGameWithClusterInfo::operator= (const BayesianGameWithClusterInfo& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    throw E("BayesianGameWithClusterInfo::operator= not fully implemented yet...");

    return *this;
}



BGwCI_sharedPtr 
BayesianGameWithClusterInfo::ConstructExtendedBGWCI(
        const BGwCI_constPtr &pBG,
        const JointPolicyDiscretePure& prevJPolBG,
        //const JointPolicyPureVector& prevJPolBG,
        const QFunctionJAOHInterface* q
 )
{
    //compute extended individual histories for each agent
    Index t = pBG->GetStage()+1;
    vector<size_t> nrTypes = pBG->GetNrTypes();   
    //each agent will form newtype = <type, action, observation>
    //tuples. Since the action is fixed (by prevJPolBG), the number of new
    //types for agent i is: nrTypes_i * nrObservations_i
    for(Index agI=0; agI < pBG->GetNrAgents(); agI++)
        nrTypes.at(agI) *= pBG->GetPUDecPOMDPDiscrete()->GetNrObservations(agI);

    boost::shared_ptr<JointPolicyDiscretePure> prevJPolBGptr=
        boost::shared_ptr<JointPolicyDiscretePure>(prevJPolBG.Clone());
    BGwCI_sharedPtr bg(new BayesianGameWithClusterInfo(
                           pBG->GetPUDecPOMDPDiscrete(), q, t,
                           pBG,
                           prevJPolBGptr, 
                           pBG->GetNrAgents(),
                           pBG->GetNrActions(),
                           nrTypes,
                           pBG->GetClusterAlgorithm()
                           ));

    bg->Extend();
    //return the newly constructed bg
    return(bg);
}

void BayesianGameWithClusterInfo::Extend()
{
        //const PlanningUnitDecPOMDPDiscrete* pu,
    const QFunctionJAOHInterface* q = GetQHeur();
        //Index t,
    BGwCI_constPtr prevBG = _m_pBG;
    BayesianGameWithClusterInfo* bg = this;
    boost::shared_ptr<const JointPolicyDiscretePure> prevJPolBG_p = _m_pBGJPol;
        //size_t nrAgents,
        //const vector<size_t>& nrActions,
        //const vector<size_t>& nrTypes
    
    //create all extended individual type clusters
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        bg->_m_typeLists.push_back( new TypeClusterList() );
        Index tcI = 0;
        for(Index ptI=0; ptI < prevBG->GetNrTypes(agI); ptI++)
        {
            const TypeCluster* tc = prevBG->GetTypeCluster(agI, ptI);
            Index aI = prevJPolBG_p->GetActionIndex(agI, ptI);
            for(Index oI=0; oI < GetPUDecPOMDPDiscrete()->GetNrObservations(agI); oI++)
            {
                Type_PointerTuple* t = new Type_PointerTuple(tc, aI, oI);
                TypeCluster* tc_new = new TypeCluster(tcI++);
                tc_new->AddType(t);
                bg->_m_typeLists.at(agI)->push_back(tc_new);
            }
        }
    }
    
    //loop over all joint types of this (new) BG and compute the joint
    //beliefs and payoffs
    for(Index jtI=0; jtI < GetNrJointTypes(); jtI++)
    {
        const vector<Index>& indTypeIs = JointToIndividualTypeIndices(jtI);
        //NOTE:
        //Here we assume that if types are clustered, then their heuristic
        //values will also be the same. Therefore we will only compute 1
        //joint action observation history for each joint type(cluster), and 
        //use the heuristic value of this one jaoh for the entire joint type
        //cluster.
        //
        //(if the criterion holds, we know that their optimal value
        //is the same, and for heuristics as QMDP we also know that they are the
        //same. However, when using approximate clustering or a strange 
        //heuristic, this might not hold!)
        
        //
        vector<Index> prec_indTypeIs(GetNrAgents());
        vector<Index> prec_action(GetNrAgents());
        vector<Index> prec_observation(GetNrAgents());
        for(Index agI=0; agI<GetNrAgents(); agI++)
        {
            Index tcI = indTypeIs.at(agI);
            TypeClusterList* tcl = bg->_m_typeLists.at(agI);
            TypeCluster* tc = tcl->at(tcI);
            //since we have just constructed the TypeClusterList, we know that
            //this TypeCluster contains just 1 Type*, and that it is a
            //Type_PointerTuple:
            Type* t1 = *(tc->begin());
            Type_PointerTuple *t = dynamic_cast<Type_PointerTuple*>(t1);
            //store the action taken and observation received.
            prec_action.at(agI) = t->GetAction();
            prec_observation.at(agI) = t->GetObservation();
            const TypeCluster* prec_tc = t->GetPredecessor();
            prec_indTypeIs.at(agI) = prec_tc->GetIndex();
        }
        Index prec_jtI = prevBG->IndividualToJointTypeIndices(prec_indTypeIs);

        // Make sure that this prec_jtI actually has a non-zero prob of occurring
        if(prevBG->GetProbability(prec_jtI)>0)
        {
            //get the previous joint belief and do the update:
            JointBeliefInterface* prec_jb = prevBG->_m_JBs.at(prec_jtI);
            if(!prec_jb)
                throw(E("BayesianGameWithClusterInfo::Extend() previous joint belief has not been computed"));
            JointBeliefInterface* new_jb = GetPUDecPOMDPDiscrete()->GetNewJointBeliefInterface();
            *new_jb = *prec_jb;
            Index prec_JA = IndividualToJointActionIndices(prec_action);
            Index prec_JO = 
                GetPUDecPOMDPDiscrete()->IndividualToJointObservationIndices(prec_observation);
            // when this observation is impossible, condPjtI will be 0, check for that
            double condPjtI = new_jb->Update(*GetPUDecPOMDPDiscrete()->GetDPOMDPD(), prec_JA, prec_JO);
            if(condPjtI>0)
            {
                bg->_m_JBs.at(jtI) = new_jb;
                double PjtI = condPjtI * prevBG->GetProbability(prec_jtI);
                bg->SetProbability(jtI, PjtI);
        
                // check if we can directly use the joint belief
                bool useJointBeliefDirectly=false;
                // if we have a QFunctionJointBeliefInterface, then we don't
                // need the JAOH indices, also faster
                if(_m_qJB)
                    useJointBeliefDirectly=true;
                // if we have a Hybrid Qheur, only the first k stages are
                // represented as trees, after that we can use the joint
                // beliefs (also faster)
                if(_m_qHybrid && !_m_qHybrid->StageRepresentedAsTree(bg->GetStage()))
                    useJointBeliefDirectly=true;

                if(useJointBeliefDirectly)
                {
                    // In case of high horizons, when the jaohI index might
                    // overflow, we will typically use a Qfunction that
                    // operates on beliefs instead of history indices, so we
                    // can directly use the belief we just computed.
                    // This also saves us from the index overflow.
                    for(Index jaI=0; jaI < GetNrJointActions(); jaI++)
                    {
                        double qval=42;
                        if(_m_qJB)
                            qval=_m_qJB->GetQ(*new_jb,bg->GetStage(),jaI);
                        else
                            qval=_m_qHybrid->GetQ(*new_jb,bg->GetStage(),jaI);
                        bg->SetUtility(jtI, jaI, qval);
                    }
                }
                else
                {
                    // only maintain jaohRep if we are actually going to use them
                    Index prec_jaohRep = prevBG->_m_jaohReps.at(prec_jtI);
                    Index jaohRep = CastLIndexToIndex(GetPUDecPOMDPDiscrete()->GetSuccessorJAOHI(prec_jaohRep,
                                                                               prec_JA,prec_JO));
                    bg->_m_jaohReps.at(jtI) = jaohRep;
                    
                    for(Index jaI=0; jaI < GetNrJointActions(); jaI++)
                        bg->SetUtility(jtI, jaI, q->GetQ(jaohRep, jaI));
                }
            }
        }
        else
        {
            bg->SetProbability(jtI, 0.0);
            for(Index jaI=0; jaI < GetNrJointActions(); jaI++)
                bg->SetUtility(jtI, jaI, 0.0);
        }
    }
}


BGwCI_sharedPtr  
BayesianGameWithClusterInfo::Cluster() 
{
    //first construct new clustered individual type set for each agent
    std::vector< TypeClusterList* > newTypeLists;
    std::vector< size_t > nrNewTypes(GetNrAgents(), 0);

    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        //we create a new (empty) TypeClusterList for this agent.
        TypeClusterList* newTypeList = new TypeClusterList();
        newTypeLists.push_back(newTypeList);
        nrNewTypes.at(agI) = ConstructClusteredIndividualTypes(agI, 
                newTypeList);
    }

    BGwCI_sharedPtr bgc(new BayesianGameWithClusterInfo(
            GetPUDecPOMDPDiscrete(),
            GetQHeur(),
            GetStage(),
            _m_pBG,
            _m_pBGJPol,
            _m_nrAgents,
            _m_nrActions,
            nrNewTypes,
            _m_clusterAlgorithm
                            ));
#if 0
    cout << GetStage() << " " << bgc->GetNrJointTypes()
         << "      stage " << GetStage()
         << " nrNewTypes " << SoftPrintVector(nrNewTypes)
         << " old nrJointTypes " << GetNrJointTypes()
         << " new nrJointTypes " << bgc->GetNrJointTypes()
         << endl;
#endif

    bgc->_m_typeLists = newTypeLists;


    //copy all the relevant stuff from the original BG (from *this*)
    //the the new clustered one (bgc)
    for(Index jtcI=0; jtcI < bgc->GetNrJointTypes(); jtcI++)
    {
        //find corresponding jtcI in the original BG (i.e., in this)
        const vector<Index>& indTypes = bgc->JointToIndividualTypeIndices(jtcI);
        vector<Index> previousTCI(bgc->GetNrAgents(), 0 );
        for(Index agI=0; agI < bgc->GetNrAgents(); agI++)
        {
            Index tcI = indTypes.at(agI);
            TypeClusterList* tcl = bgc->_m_typeLists.at(agI);
            TypeCluster* tc = tcl->at(tcI);
            Index previousIndexOfThisTC = tc->GetIndex();
            previousTCI.at(agI) = previousIndexOfThisTC;
        }
        Index previousjtcI = this->IndividualToJointTypeIndices(
                previousTCI);
        //
        double p = this->GetProbability(previousjtcI);
        bgc->SetProbability( jtcI, p);
        ///make a deep copy of the joint beliefs!
        //bgc->_m_JBs.at(jtcI) = this->_m_JBs.at(previousjtcI);
        if(this->_m_JBs.at(previousjtcI))
        {
            JointBeliefInterface* jbcopy =  (this->_m_JBs.at(previousjtcI))->Clone();
            bgc->_m_JBs.at(jtcI) = jbcopy;
        }

        // only maintain jaohRep if we are actually going to use them
        if(!_m_qJB)
            bgc->_m_jaohReps.at(jtcI) = this->_m_jaohReps.at(previousjtcI);

        for(Index jaI=0; jaI < bgc->GetNrJointActions(); jaI++)
        {
            double u = this->GetUtility(previousjtcI, jaI);
            bgc->SetUtility(jtcI, jaI, u);
        }

    }

    ///renumber the typeclusters for each agent
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {

        TypeClusterList* tcl = bgc->_m_typeLists.at(agI);
        Index i = 0;
        TypeClusterList::iterator it = tcl->begin();
        TypeClusterList::iterator last = tcl->end();
        while(it != last)
        {
            TypeCluster* tc = *it;
            tc->SetIndex(i);
            i++;
            it++;
        }
    }
      
    bgc->SanityCheck();
    

    return bgc;
    //return this;
}
        
bool BayesianGameWithClusterInfo::TestExactEquivalence(Index agI, Index t1, Index t2) const
{
    //looping over all joint type profiles of the other agents
    //( over all type_{-i} ) is inconvenient.
    //Rather we loop over all joint types in this function
    
    //we start with the individual type vector corr. to joint type 0
    //(i.e., the vector with type index 0 for each agent).
    vector<Index> typeIndices;

    //first we compute the marginals P(t1) and P(t2)
    double p1, p2;
    p1 = p2 = 0.0;    
    typeIndices = vector<Index>(GetNrAgents(), 0 );
    bool finished = false;
    bool equalProb=true;

    while( !finished )
    {
        //only if agent agI's component is t1 we do stuff
        if(typeIndices.at(agI) == t1)
        {
            vector<Index> typeIndices2 = typeIndices;
            typeIndices2.at(agI) = t2;
            Index jtI1 = IndividualToJointTypeIndices( typeIndices);
            Index jtI2 = IndividualToJointTypeIndices( typeIndices2);
            p1 += GetProbability(jtI1);
            p2 += GetProbability(jtI2);
        }        
        finished = IndexTools::Increment(typeIndices, _m_nrTypes);
    }



    //now we compare P(type{-i} | t1) =? P(type{-i} | t2)
    typeIndices = vector<Index>(GetNrAgents(), 0 );
    finished = false;
    while( !finished && equalProb )
    {
        //only if agent agI's component is t1 we do stuff
        if(typeIndices.at(agI) == t1)
        {
            vector<Index> typeIndices2 = typeIndices;
            typeIndices2.at(agI) = t2;
            Index jtI1 = IndividualToJointTypeIndices( typeIndices);
            Index jtI2 = IndividualToJointTypeIndices( typeIndices2);
            //cout << "Comparing jtI1="<<jtI1<<"="
                //<<SoftPrintVector(typeIndices);
            //cout << "and jtI2="<<jtI2<<"="
                //<<SoftPrintVector(typeIndices2) << endl;
            double pjt1 =  GetProbability(jtI1);
            double pjt2 =  GetProbability(jtI2);
            double conditional_pjt1 = pjt1 / p1;
            double conditional_pjt2 = pjt2 / p2;
            //cout << "p1="<<p1
                //<<"\t,pjt1="<<pjt1
                //<<"\t,cpjt1="<<conditional_pjt1
                //<<"\np2="<<p2
                //<<"\t,pjt2="<<pjt2
                //<<"\t,cpjt2="<<conditional_pjt2
                //<<endl;
            if(! Globals::EqualProbability(conditional_pjt1,conditional_pjt2 ) )
            {
                equalProb=false;
            }
        }        
        finished = IndexTools::Increment(typeIndices, _m_nrTypes);
    }

#if !BGCLUSTER_OUTPUT_TESTEQUIVALENCE
    // if we want to check why two types are not equivalent, we also
    // need the second test, otherwise we can stop here
    if(!equalProb)
        return(false);
#endif

    //now we compare P(s, type{-i} | t1) =? P(s, type{-i} | t2)
    //or, actually we check that P(s |  type{-i}, t1) =? P(s | type{-i} , t2) 
    typeIndices = vector<Index>(GetNrAgents(), 0 );
    finished = false;
    bool equalJB=true;
    while( !finished && equalJB )
    {
        //only if agent agI's component is t1 we do stuff
        if(typeIndices.at(agI) == t1)
        {
            vector<Index> typeIndices2 = typeIndices;
            typeIndices2.at(agI) = t2;
            Index jtI1 = IndividualToJointTypeIndices( typeIndices);
            Index jtI2 = IndividualToJointTypeIndices( typeIndices2);
            JointBeliefInterface* jb1 = _m_JBs.at(jtI1);
            JointBeliefInterface* jb2 = _m_JBs.at(jtI2);
            // only do this test if the joint beliefs actually exist
            // (they may not in case the joint type has prob 0)
            if(jb1!=0 && jb2!=0 &&
               // If they exist, we still need to check whether can
               // actually occur!
               // In the old code (used for AAMAS09), this check was
               // missing, which is why the recycling problem
               // clustered to 9 JT there, instead of 4 now.
               !Globals::EqualProbability(GetProbability(jtI1),0.0) &&
               !Globals::EqualProbability(GetProbability(jtI2),0.0))
            {
                BeliefIteratorGeneric bit1 = jb1->GetIterator();
                BeliefIteratorGeneric bit2 = jb2->GetIterator();
                do
                {
                    // as the belief can be sparse, we also need to
                    // make sure the iterators point to the same state
                    // index, otherwise beliefs like [0 0 1 0] and [0
                    // 1 0 0] are considered equal
                    if(bit1.GetStateIndex() !=
                       bit2.GetStateIndex())
                        equalJB = false;
                    else
                    {
                        double p1 = bit1.GetProbability();
                        double p2 = bit2.GetProbability();
                        if(! Globals::EqualProbability(p1,p2) )
                            equalJB=false;
                    }
                }
                while(equalJB && bit1.Next() && bit2.Next());
            }
        }        
        finished = IndexTools::Increment(typeIndices, _m_nrTypes);
    }

#if BGCLUSTER_OUTPUT_TESTEQUIVALENCE
    if(!equalProb && !equalJB)
        cout << "TESTEQUIVALENCE " << GetStage() << " notEquivalent both" << endl;
    else if(!equalProb)
        cout << "TESTEQUIVALENCE " << GetStage() << " notEquivalent prob" << endl;
    else if(!equalJB)
        cout << "TESTEQUIVALENCE " << GetStage() << " notEquivalent belief" << endl;
    else
        cout << "TESTEQUIVALENCE " << GetStage() << " equivalent" << endl;
#endif

    if(equalProb && equalJB)
        return(true);
    else
        return(false);
}

size_t BayesianGameWithClusterInfo::ConstructClusteredIndividualTypes(
        Index agI, 
        TypeClusterList* newTypeList
        )
{
    size_t nrNewTypes = 0;
    //initialize the set of individual types of this agent
    //(with all types in it), we will incrementally remove
    //clusterable types
    list<Index> tIs;
    for(Index tI=0; tI < GetNrTypes(agI); tI++)
        tIs.push_back(tI);
    

    list<Index>::iterator it1, it2;
    it1 = tIs.begin();
    //Index loopIndex = 0;
    while(it1 != tIs.end())
    {
        // *it1 will become a type(cluster) in the clustered BG.

        Index t1 = *it1;
        
        ///check if this type has (marginal) prob. > 0
        double margProb = ComputeMarginalTypeProbability(agI,t1);
#if BGCLUSTER_REMOVE_ZEROPROB_TYPES
        if( Globals::EqualProbability(margProb, 0.0) )
        {
            it1++;
#if BGCLUSTER_OUTPUT_TESTEQUIVALENCE
            cout << "TESTEQUIVALENCE " << GetStage() << " zeroProb" << endl;
#endif
            continue;
        }
#endif
        //we make a (deep) copy and perform the clustering in there
        TypeCluster* tc1_inTheOriginal_BG =  _m_typeLists.at(agI)->at(t1);
        TypeCluster* tc1 = new TypeCluster(*tc1_inTheOriginal_BG);

        newTypeList->push_back(tc1);
        nrNewTypes++;
        it2 = it1;
        it2++;
        while(it2 != tIs.end())             
        {
            //compare the types pointed to by it1 and it2
            Index t2 = *it2;
            ///check if this type has (marginal) prob. > 0
            double margProb2 = ComputeMarginalTypeProbability(agI,t2);
#if BGCLUSTER_REMOVE_ZEROPROB_TYPES

            if( Globals::EqualProbability(margProb2, 0.0) )
            {
                it2++;
#if BGCLUSTER_OUTPUT_TESTEQUIVALENCE
            cout << "TESTEQUIVALENCE " << GetStage() << " zeroProb" << endl;
#endif
                continue;
            }
#endif
            bool equivalent=false;
            switch(_m_clusterAlgorithm)
            {
            case Lossless:
                equivalent=TestExactEquivalence(agI, t1, t2);
                break;
            default:
                throw(E("BayesianGameWithClusterInfo::ConstructClusteredIndividualTypes clustering algorithm not handled"));
            }

            if( equivalent )
            {
                //cluster stuff
                //cout << "About to cluster types " <<t1 <<"," <<t2<<
                    //" of agent "<<agI<<"!"<< endl;
                TypeCluster* tc2 = this->_m_typeLists.at(agI)->at(t2);
                tc1->Merge(tc2);
                tc2->clear();
                ShiftProbabilityAndUtility(agI, t1, t2);
                //remove Index from type list
                it2 = tIs.erase(it2);
            }
            else
            {
                //if we performed it2 = tIs.erase(it2);, then it2 is already
                //advanced.
                it2++;
            }
        }
        it1++;
        //loopIndex++;
    }
    return nrNewTypes;
} // now we should have constructed all the individual sets of types

void BayesianGameWithClusterInfo::ShiftProbabilityAndUtility(Index agI, Index t1, Index t2)
{
    

    //add the probability mass of the joint types in which
    //t2 participates to the corresponding joint types
    //of t1.
    vector<Index> typeIndices;
    //first we compute the marginals P(t1) and P(t2)
    double p1, p2 = 0.0;
    double u1, u2 = 0;
    typeIndices = vector<Index>(GetNrAgents(), 0 );
    bool finished = false;
    while( !finished )
    {
        //only if agent agI's component is t1 we do stuff
        if(typeIndices.at(agI) == t1)
        {
            vector<Index> typeIndices2 = typeIndices;
            typeIndices2.at(agI) = t2;
            Index jtI1 = IndividualToJointTypeIndices( 
                    typeIndices);
            Index jtI2 = IndividualToJointTypeIndices( 
                    typeIndices2);
            p1 = GetProbability(jtI1);
            p2 = GetProbability(jtI2);

            SetProbability(jtI1, p1+p2);
            SetProbability(jtI2, 0.0);
        }        
        finished = 
            IndexTools::Increment(typeIndices, _m_nrTypes);
    }
}



Index 
BayesianGameWithClusterInfo::FindTypeClusterIndex(
        Index agI, const TypeCluster* tc_previous, 
    Index aI, Index oI) const
{
    const TypeClusterList* tcl = _m_typeLists.at(agI);
    typeC_ci it = tcl->begin();
    typeC_ci last = tcl->end();

    Index tcI = 0; 
    Index foundtcI = 42;
    bool finished = false;
    while(it != last && !finished)
    {
        const TypeCluster* tc = *it;
        TypeCluster::type_ci it2 = tc->begin();
        TypeCluster::type_ci last2 = tc->end();
        while(it2 != last2  && !finished )
        {
            const Type* t1 = *it2;
            if(t1->GetSubClass() == Type::POINTERTUPLE)
            {
                const Type_PointerTuple* t = dynamic_cast
                    <const Type_PointerTuple*>(t1);
                if( tc_previous == t->GetPredecessor() &&
                    aI == t->GetAction() &&
                    oI == t->GetObservation() )
                {
                    finished = true;
                    foundtcI = tcI;
                }
            }
            it2++;
        }
        tcI++;
        it++;
    }
    if(!finished)
        throw E("BayesianGameWithClusterInfo::FindTypeClusterIndex - no matching type found!");

    return(foundtcI);
}


string BayesianGameWithClusterInfo::SoftPrint() const
{
    stringstream ss;
    ss << "Previous joint policy:"<<endl;
    if(_m_pBGJPol != 0 && _m_pBG != 0)
    {
        for(Index agI=0; agI < GetNrAgents(); agI++)
        {
            ss << "agI=" << agI <<endl;
            TypeClusterList* tl = _m_pBG->_m_typeLists.at(agI);
            {
                Index i=0;
                for(TypeClusterList::const_iterator it = tl->begin();
                        it != tl->end(); it++, i++)
                {
                    ss << "tI "<< i << " - " << (*it)->SoftPrint() 
                        << " Action: " << _m_pBGJPol->GetActionIndex(agI, i)
                        << endl;

                }
            }
        }
        ss << "that lead to the following BG:"<<endl;
    }
    else
        ss << "There is no previous BG or BG-policy"<<endl;



    ss << BayesianGameForDecPOMDPStage::SoftPrint(); 
    ss << "This BG is clustered/clusterable."<<endl;
    ss << "Joint beliefs assoc. with each joint type:"<<endl;
    {
        Index i=0;
        for(vector< JointBeliefInterface* >::const_iterator it=_m_JBs.begin(); 
                it != _m_JBs.end(); it++, i++)
        {
            ss << "jtI="<<i
                << "=" << PrintTools::SoftPrintVector(
                        JointToIndividualTypeIndices(i)
                        )
                << ", repres. jaohI="<< _m_jaohReps.at(i)
            //GetPUDecPOMDPDiscrete()->GetJointActionObservationHistoryTree(_m_jaohReps.at(i))->
                    //GetJointActionObservationHistory()->
                    //SoftPrintJointIndicesconst()
                <<", jb="<< (*it)->SoftPrint() 
            << endl;
        }
    }
    ss << "Type lists of agents:"<<endl;
    for(Index agI=0; agI < GetNrAgents(); agI++)
    {
        ss << "agI=" << agI << ", types:"<<endl;
        TypeClusterList* tl = _m_typeLists.at(agI);
        {
            Index i=0;
            for(TypeClusterList::const_iterator it = tl->begin();
                    it != tl->end(); it++, i++)
            {
                ss << "tI "<< i << " - " << (*it)->SoftPrint() << endl;
            }
        }
    }

    return(ss.str());
}

double 
BayesianGameWithClusterInfo::
ComputeMarginalTypeProbability(Index agI, Index typeI) const
{
    double p1;
    p1 =  0.0;    
    vector<Index> typeIndices (GetNrAgents(), 0 );
    bool finished = false;
    while( !finished )
    {
        //only if agent agI's component is t1 we do stuff
        if(typeIndices.at(agI) == typeI)
        {
            Index jtI1 = IndividualToJointTypeIndices( typeIndices);
            p1 += GetProbability(jtI1);
        }        
        finished = IndexTools::Increment(typeIndices, _m_nrTypes);
    }
    return p1;
}

string BayesianGameWithClusterInfo::SoftPrint(BGClusterAlgorithm clusterAlg)
{
    switch(clusterAlg)
    {
    case Lossless:
        return("Lossless");
    }

    throw(E("BayesianGameWithClusterInfo::SoftPrint BGClusterAlgorithm not handled"));
    return("");
}
