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

#include "JointPolicyPureVectorForClusteredBG.h"
#include "BayesianGameWithClusterInfo.h"
#include "BayesianGameIdenticalPayoffInterface.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "TypeCluster.h"
#include "Type_AOHIndex.h"

//#define DEBUG_JPOLASSIGN 0

using namespace std;

JointPolicyPureVectorForClusteredBG::JointPolicyPureVectorForClusteredBG(
        const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &pu,
        PolicyGlobals::PolicyDomainCategory idc,
        JPPVfCBG_constPtr prevJPolBG,
        double r)
    : 
    PartialJointPolicyPureVector( pu, PolicyGlobals::TYPE_INDEX, r ),
    _m_bg(),
    _m_prevJPolBG(prevJPolBG)
{
    if(idc!=PolicyGlobals::TYPE_INDEX)
        throw(E("JointPolicyPureVectorForClusteredBG should have TYPE_INDEX as domain category"));

    BGwCI_constPtr bg = boost::dynamic_pointer_cast<const BayesianGameWithClusterInfo>(pu);
    if(bg==0)
        throw(E("JointPolicyPureVectorForClusteredBG can only be constructed with a BayesianGameWithClusterInfo"));
    _m_bg=bg;
}

//copy constructor
JointPolicyPureVectorForClusteredBG::JointPolicyPureVectorForClusteredBG(
    const JointPolicyPureVectorForClusteredBG& a) : 
    PartialJointPolicyPureVector(a),
    _m_bg(a._m_bg),
    _m_prevJPolBG(a._m_prevJPolBG)
{
}


//Destructor
JointPolicyPureVectorForClusteredBG::~JointPolicyPureVectorForClusteredBG()
{
}

//Copy assignment operator
JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicyPureVectorForClusteredBG& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicyPureVectorForClusteredBG& o) called"<<endl;
#endif
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    _m_bg = o._m_bg;
    _m_prevJPolBG = o._m_prevJPolBG;
    
    //call base class assignment:
    PartialJointPolicyPureVector::operator= ( o );
    return *this;
}

JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const PartialJointPolicyPureVector& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const PartialJointPolicyPureVector& o)  called" << endl;
#endif
    const JointPolicyPureVectorForClusteredBG& o2 = dynamic_cast
        <const JointPolicyPureVectorForClusteredBG& >(o);
    return(this->operator=(o2)) ;
}

JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const PartialJointPolicyDiscretePure& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const PartialJointPolicyDiscretePure& o)  called" << endl;
#endif
    const JointPolicyPureVectorForClusteredBG& o2 = dynamic_cast
        <const JointPolicyPureVectorForClusteredBG& >(o);
    return(this->operator=(o2)) ;
}

JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicyDiscretePure& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicyDiscretePure& o)  called" << endl;
#endif
    const JointPolicyPureVectorForClusteredBG& o2 = dynamic_cast
        <const JointPolicyPureVectorForClusteredBG& >(o);
    return(this->operator=(o2)) ;
}
JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicyDiscrete& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicyDiscrete& o)  called" << endl;
#endif
    const JointPolicyPureVectorForClusteredBG& o2 = dynamic_cast
        <const JointPolicyPureVectorForClusteredBG& >(o);
    return(this->operator=(o2)) ;
}

JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicy& o)
{
#if DEBUG_JPOLASSIGN 
    cerr << "JointPolicyPureVectorForClusteredBG& JointPolicyPureVectorForClusteredBG::operator= (const JointPolicy& o)  called" << endl;
#endif
    const JointPolicyPureVectorForClusteredBG& o2 = dynamic_cast
        <const JointPolicyPureVectorForClusteredBG& >(o);
    return(this->operator=(o2)) ;
}



/* Pseudo code for converting to JointPolicyPureVector
 
//goto first stage
{
    bgQ = jpolQ = emptyQ
    bg = jpolBG.BG()
    jpolQ.push(jpolBG)
    bgQ.push(jpolBG)

    while bg.pastJPolBG != 0
        jpolBG = bg.pastJPolBG
        bg = jpolBG.BG()
        jpolQ.push(jpolBG)
        bgQ.push(jpolBG)
    end            
    if stage != 0
        can not fill entire JPPV

}
//now we are at the first stage (represented)
RecursivelyReconstructJointPolicyForStage( jpolJPPV, ts, jpolQ, bgQ)
or
StartRecursiveConstructionPerAgent( jpolJPPV, ts, jpolQ, bgQ)



RecursivelyReconstructJointPolicyForStage( jpolJPPV, ts, jpolQ, bgQ)
{
    jpolBG = jpolQ.pop()
    bg = bgQ.pop()
    for each agents
        for each type index tI
            if 'first' stage
                oaHI = tI.Get_oaHI
                aI = jpolBG.GetAction(tI);
                jpolJPPV.SetAction(oaHI, aI);
            else
                //this is shitty:
                for each oaHI corresponding to tI
                    aI = jpolBG.GetAction(tI);
                    jpolJPPV.SetAction(oaHI, aI);
                end
            end
        end
    end
    RecursivelyReconstructPolicyForStage( jpolJPPV, ts+1, jpolQ, bgQ)
}


or


StartRecursiveConstructionPerAgent( jpolJPPV, ts, jpolQ, bgQ)
{
    for each agent i
        for each type index tI
            oHI = tI.Get_oHI
            aI = jpolBG.GetAction(i, tI);
            jpolJPPV.SetAction(i, oHI, aI);
            RecursivelyFillPolicyForAgent(
                jpolJPPV, i, ts+1, oHI, tI, aI, jpolQ, bgQ)
        end
    end
}

RecursivelyFillPolicyForAgent(jpolJPPV, i, ts, oHI, tI, aI, jpolQ, bgQ)
{
    jpolBG = jpolQ[ts]
    bg = bgQ[ts]
    for each observation oI
        oHI = pu.getSuccessor(i, oHI, aI, oI);
        tI = bg.FindType(i,  <tI, aI, oI> );
        aI = jpolBG.GetAction(i, tI);
        jpolJPPV.SetAction(i, oHI, aI);
        RecursivelyFillPolicyForAgent(
                jpolJPPV, i, ts+1, oHI, tI, aI, jpolQ, bgQ)
    end
}
*/

#define DEBUG_TOJPPV 0

        
JPPV_sharedPtr 
JointPolicyPureVectorForClusteredBG::ToJointPolicyPureVector() const
{
#if DEBUG_TOJPPV
    cout << ">>>JointPolicyPureVectorForClusteredBG::ToJointPolicyPureVector() started"<<endl;
#endif
    const PlanningUnitDecPOMDPDiscrete* pu = _m_bg->GetPUDecPOMDPDiscrete();
    JPPV_sharedPtr jppv = 
        JPPV_sharedPtr(
            new JointPolicyPureVector(pu, PolicyGlobals::OHIST_INDEX ));

    size_t h = pu->GetHorizon();
    
    const JointPolicyPureVectorForClusteredBG* jpolBG = this;
    BGwCI_constPtr bg;
    vector<const JointPolicyDiscretePure* > jpolBGVec(h);
    vector<BGwCI_constPtr> bgVec(h);
    Index t=0;
    //for(t=h-1; t >= 0; t--) //<- this never terminates (t is unsigned)
    for(Index t_inv = 0; t_inv < h; t_inv++)
    {
        t = h - 1 - t_inv;
        bg =  jpolBG->GetBG();
#if DEBUG_TOJPPV
        cout << ">>>t="<<t<<" jpolBG: "<< endl<< jpolBG->SoftPrint() <<endl;
        cout << ">>>corresponding BG:" << endl << bg->SoftPrint() << endl;
#endif
        jpolBGVec.at(t) = jpolBG;
        bgVec.at(t) = bg;
        if( t > 0  )
        {
            jpolBG = jpolBG->_m_prevJPolBG.get();
            if(jpolBG == 0)
            {
#if DEBUG_TOJPPV
                cout << ">>>t=" << t
                     << "apparently for stages 0...t a different type of policy is used" << endl;
#endif
                //apparently for stages 0...t a different type of policy is used
                //we get it from the BG:
                boost::shared_ptr<const JointPolicyDiscretePure> jp = bg->GetPastJointPolicy();
                *jppv = *(jp->ToJointPolicyPureVector());
                break;
            }
        }
    }
    // 
#if DEBUG_TOJPPV
    cout << ">>>About to start StartRecursiveConstructionPerAgent... t="<<t<<endl;
#endif

    StartRecursiveConstructionPerAgent(pu, jppv, t, jpolBGVec, bgVec);
    return jppv;

}


void 
JointPolicyPureVectorForClusteredBG::StartRecursiveConstructionPerAgent( 
        const PlanningUnitDecPOMDPDiscrete* pu,
        JPPV_sharedPtr jpolJPPV,
        Index ts,
        const vector<const JointPolicyDiscretePure*> & jpolBGVec,
        const vector<boost::shared_ptr<const BayesianGameWithClusterInfo> >& bgVec
        ) const 
{
    const JointPolicyDiscretePure* jpolBG = jpolBGVec.at(ts);
#if DEBUG_TOJPPV
    cout << ">>>StartRecursiveConstructionPerAgent jpolBG is "
         << jpolBG->SoftPrint() << endl;
#endif
    BGwCI_constPtr bg = bgVec.at(ts);
    for(Index agI=0; agI < pu->GetNrAgents(); agI++)
    {
        for(Index tI=0; tI < bg->GetNrTypes(agI); tI++)
        {
            const TypeCluster* tc =  bg->GetTypeCluster(agI, tI);
            TypeCluster::type_ci it = tc->begin();
            TypeCluster::type_ci last = tc->end();
            while(it != last)
            {
                Type* t1 = *it;
                if(t1->GetSubClass() != Type::AOHINDEX)
                    throw E("JointPolicyPureVectorForClusteredBG::StartRecursiveConstructionPerAgent - expected an AOHINDEX type");
                Type_AOHIndex* t2 = dynamic_cast<Type_AOHIndex*>(t1);
                Index aohI = t2->GetAOHIndex();
                Index aIs[ts];
                Index oIs[ts];
                pu->GetActionObservationHistoryArrays (
                        agI, aohI, ts, aIs, oIs );
                vector<Index> observations;
                if(ts > 0)
                    observations = vector<Index>(oIs[0], oIs[ts-1]);
                Index ohI = pu->GetObservationHistoryIndex ( 
                        agI, ts, observations);
                Index aI = jpolBG->GetActionIndex(agI, tI);
                jpolJPPV->SetAction(agI, ohI, aI);
                
                if(ts+1 < pu->GetHorizon())
                    RecursivelyFillPolicyForAgent(
                        pu, jpolJPPV, agI, ts+1, aohI, ohI, tc, tI, aI, jpolBGVec, bgVec);
                it++;
            }
            
        }
    }

}
void
JointPolicyPureVectorForClusteredBG::RecursivelyFillPolicyForAgent(
            const PlanningUnitDecPOMDPDiscrete* pu,
            JPPV_sharedPtr jpolJPPV,
            Index agI,
            Index ts,
            Index aohI,
            Index ohI,
            const TypeCluster* tc,
            Index tI,
            Index aI,
            const std::vector<const JointPolicyDiscretePure* >& jpolBGVec,
            const std::vector<boost::shared_ptr<const BayesianGameWithClusterInfo> >& bgVec
    )const 
{    
    //const JointPolicyPureVectorForClusteredBG* jpolBG = jpolBGVec.at(ts);
    const JointPolicyDiscretePure *  jpolBG = jpolBGVec.at(ts);
    boost::shared_ptr<const BayesianGameWithClusterInfo> bg = bgVec.at(ts);

#if DEBUG_TOJPPV
    cout << ">>>Starting RecursivelyFillPolicyForAgent for " << endl;
    cout << "agI="<<agI<<endl;
    cout << "ts="<<ts<<endl;
    cout << "aohI="<<aohI<<endl;
    cout << "ohI="<<ohI<<endl;
    cout << "tI="<<tI<<endl;
    cout << "aI="<<aI<<endl;
    cout << "BG: " << endl << bg->SoftPrint();
    cout << endl;
#endif
    Index obsI = 0;
    for(obsI=0; obsI < pu->GetNrObservations(agI); obsI++)
    {
        Index n_aohI = pu->GetSuccessorAOHI(agI, aohI, aI, obsI);
        Index n_ohI = pu->GetSuccessorOHI(agI, ohI, obsI);
        ///NOTE: this can throw an error if there is no type 
        //for the history (e.g. if the histories prob. is 0)
        Index n_tI = 0;
        try{
            n_tI = bg->FindTypeClusterIndex(agI, tc, aI, obsI);
        } catch (E& e) {
            cout << "JointPolicyPureVectorForClusteredBG::RecursivelyFillPolicyForAgent implement catch?\n";
            continue;
        }
        
        const TypeCluster* n_tc =  bg->GetTypeCluster(agI, n_tI);
        Index n_aI = jpolBG->GetActionIndex(agI, n_tI);
        jpolJPPV->SetAction(agI, n_ohI, n_aI); //<- here something happens
        Index h = pu->GetHorizon();
        Index n_ts = ts+1;
        if(n_ts  <= h - 1) // ts + 1 <= h - 1
        {
            RecursivelyFillPolicyForAgent(
                pu, jpolJPPV, agI, n_ts, n_aohI, n_ohI, 
                n_tc, n_tI, n_aI, jpolBGVec, bgVec);
        }
        else
        {
#if DEBUG_TOJPPV
            cout << "this is last stage, no further RecursivelyFillPolicyForAgent"<<endl;
#endif
        }
    }
}

#define DEBUG_RECURSIVE_SPRINT 0

string JointPolicyPureVectorForClusteredBG::SoftPrint() const
{
    cout << "JointPolicyPureVectorForClusteredBG:SoftPrint"<<endl;
    size_t depth = GetDepth();
    stringstream ss;
    ss << "JointPolicyPureVectorForClusteredBG, depth " 
       << GetDepth() << endl;
    if(depth == 0)
    {
        ss << "empty policy!"<< endl;
        return(ss.str());
    }

    if(JointPolicyDiscretePure::GetInterfacePTPDiscretePureShared() == 0 &&
       JointPolicyDiscretePure::GetInterfacePTPDiscretePure() == 0)
        throw E("JointPolicyPureVectorForClusteredBG:SoftPrint - Error!!! PTPDP==0, but depth>0");
            
    ss << "Improve printing this (clustered) policy..., the below still takes exponential space... "<<endl;
/*    for(
            Index agentI = 0; 
            agentI <    static_cast< vector<PolicyPureVector*>::size_type> 
                            (PTPDP->GetNrAgents());
            agentI++
        )
    {
        ss << "Policy for agent " << agentI << " (index "
           << _m_policyPointerVector.at(agentI)->GetIndex() << "):" << endl
           << _m_policyPointerVector.at(agentI)->SoftPrint();
    }
*/    
    const PlanningUnitDecPOMDPDiscrete* pu = _m_bg->GetPUDecPOMDPDiscrete();
    size_t h = this->GetDepth();//pu->GetHorizon();
//     JPPVfCBG_constPtr jpolBG = 
//         JPPVfCBG_constPtr(new JointPolicyPureVectorForClusteredBG(*this));
    const JointPolicyPureVectorForClusteredBG *jpolBG=this;
    boost::shared_ptr<const BayesianGameWithClusterInfo> bg;
    vector<const JointPolicyDiscretePure* > jpolBGVec(h);
    vector<boost::shared_ptr<const BayesianGameWithClusterInfo> > bgVec(h);
    Index t=0;
    //for(t=h-1; t >= 0; t--) //<- this never terminates (t is unsigned)
    for(Index t_inv = 0; t_inv < h; t_inv++)
    {
        t = h - 1 - t_inv;
        bg =  jpolBG->GetBG();
        jpolBGVec.at(t) = jpolBG;
        bgVec.at(t) = bg;

        if( t > 0  )
        {
            jpolBG = jpolBG->GetPrevJPPVfCBG().get();
            if(jpolBG == 0)
            {
                //apparently for stages 0...t a different type of policy is used
                //we get it from the BG:
                boost::shared_ptr<const JointPolicyDiscretePure> jp = bg->GetPastJointPolicy();
                if(jp != 0)
                {
                    ss << "The following joint policy was used for stages 0..."
                        << t << " :"<< endl;
                    ss << jp->SoftPrint();
                }
                else
                {
                    cerr << "JointPolicyPureVectorForClusteredBG::SoftPrint"
                       <<" t="<<t<<", but no past joint policy? (jp ==0)"<<endl;

                }
                break;
            }

        }
    }
#if DEBUG_RECURSIVE_SPRINT
    cout << ">>>About to start StartRecursiveSoftPrintPerAgent..."<<endl;
#endif
    StartRecursiveSoftPrintPerAgent(pu, ss, t, h-1, jpolBGVec, bgVec);
    return(ss.str());
} 

void 
JointPolicyPureVectorForClusteredBG::StartRecursiveSoftPrintPerAgent( 
        const PlanningUnitDecPOMDPDiscrete* pu
        , stringstream& ss
        , Index ts
        , Index last_ts
        , const vector<const JointPolicyDiscretePure *>& jpolBGVec
        , vector<boost::shared_ptr<const BayesianGameWithClusterInfo> > bgVec
        )const 
{
    //const JointPolicyPureVectorForClusteredBG* jpolBG = jpolBGVec.at(ts);
    const JointPolicyDiscretePure*  jpolBG = jpolBGVec.at(ts);
    boost::shared_ptr<const BayesianGameWithClusterInfo> bg = bgVec.at(ts);
    for(Index agI=0; agI < pu->GetNrAgents(); agI++)
    {
        for(Index tI=0; tI < bg->GetNrTypes(agI); tI++)
        {
            const TypeCluster* tc =  bg->GetTypeCluster(agI, tI);
            TypeCluster::type_ci it = tc->begin();
            TypeCluster::type_ci last = tc->end();
            while(it != last)
            {
                Type* t1 = *it;
                if(t1->GetSubClass() != Type::AOHINDEX)
                {
                    //throw E("JointPolicyPureVectorForClusteredBG::StartRecursiveSoftPrintPerAgent - expected an AOHINDEX type");
                    it++;
                    continue;
                }
                Type_AOHIndex* t2 = dynamic_cast<Type_AOHIndex*>(t1);
                Index aohI = t2->GetAOHIndex();
                Index aIs[ts];
                Index oIs[ts];
                pu->GetActionObservationHistoryArrays (
                        agI, aohI, ts, aIs, oIs );
                vector<Index> observations;
                if(ts > 0)
                    observations = vector<Index>(oIs[0], oIs[ts-1]);
                Index ohI = pu->GetObservationHistoryIndex ( 
                        agI, ts, observations);
                Index aI = jpolBG->GetActionIndex(agI, tI);
                ss << pu->SoftPrintObservationHistory(agI, ohI) << " -> " <<
                    pu->SoftPrintAction(agI, aI) << endl;

                if(ts+1 <= last_ts)
                    RecursivelyPrintPolicyForAgent(
                        pu, ss, agI, ts+1, last_ts, aohI, 
                        ohI, tc, tI, aI, jpolBGVec, bgVec);
                it++;
            }
            
        }
    }

}
void
JointPolicyPureVectorForClusteredBG::RecursivelyPrintPolicyForAgent(
        const PlanningUnitDecPOMDPDiscrete* pu
        , stringstream& ss
        , Index agI
        , Index ts
        , Index last_ts
        , Index aohI    //the previous aohI
        , Index ohI    //the previous ohI
        , const TypeCluster* tc //the previous typecluster that represents aohI
        , Index tI //the index of tc (in the previous bg)
        , Index aI //the action taken for tI (i.e. for aohI)
        , const vector<const JointPolicyDiscretePure* >& jpolBGVec
        , vector<boost::shared_ptr<const BayesianGameWithClusterInfo> > bgVec
    )const 
{    
    //const JointPolicyPureVectorForClusteredBG* jpolBG = jpolBGVec.at(ts);
    const JointPolicyDiscretePure* jpolBG = jpolBGVec.at(ts);
    boost::shared_ptr<const BayesianGameWithClusterInfo> bg = bgVec.at(ts);

#if DEBUG_RECURSIVE_SPRINT
    cout << ">>>Starting RecurivelyPrintPolicyForAgent for " << endl;
    cout << "agI="<<agI<<endl;
    cout << "ts="<<ts<<endl;
    cout << "aohI="<<aohI<<endl;
    cout << "ohI="<<ohI<<endl;
    cout << "tI="<<tI<<endl;
    cout << "aI="<<aI<<endl;
    //cout << "BG: " << endl << bg->SoftPrint();
    cout << endl;
#endif
    Index obsI = 0;
    for(obsI=0; obsI < pu->GetNrObservations(agI); obsI++)
    {
        Index n_aohI = pu->GetSuccessorAOHI(agI, aohI, aI, obsI);
        Index n_ohI = pu->GetSuccessorOHI(agI, ohI, obsI);
        ///NOTE: this can throw an error if there is no type 
        //for the history (e.g. if the histories prob. is 0)
        Index n_tI = 0;
        try{
            n_tI = bg->FindTypeClusterIndex(agI, tc, aI, obsI);
        } catch (E& e) {
            ss << pu->SoftPrintObservationHistory(agI, n_ohI) << 
                " *CAN'T OCCUR* -> ANY "<< endl;
            continue;
        }
        
        const TypeCluster* n_tc =  bg->GetTypeCluster(agI, n_tI);
        Index n_aI = jpolBG->GetActionIndex(agI, n_tI);
        ss << pu->SoftPrintObservationHistory(agI, n_ohI) << " -> " <<
            pu->SoftPrintAction(agI, n_aI) << endl;

        Index n_ts = ts+1;
        if(n_ts  <= last_ts) // ts + 1 <= h - 1
        {
            RecursivelyPrintPolicyForAgent(
                pu, ss, agI, n_ts, last_ts, n_aohI, n_ohI, 
                n_tc, n_tI, n_aI, jpolBGVec, bgVec);
        }
        else
        {
#if DEBUG_RECURSIVE_SPRINT
            cout << "this is last stage, no further RecurivelyPrintPolicyForAgent"<<endl;
#endif
        }
    }
}

string JointPolicyPureVectorForClusteredBG::SoftPrintBrief() const
{
    if(JointPolicyDiscretePure::GetInterfacePTPDiscretePureShared() == 0 &&
       JointPolicyDiscretePure::GetInterfacePTPDiscretePure() == 0)
        throw E("JointPolicyPureVectorForClusteredBG:SoftPrintBrief - Error!!! PTPDP==0");
            
    stringstream ss;
    ss << "JPPV4CBG_d"<<GetDepth() << "_cur" << this << "_prev" << _m_prevJPolBG;
    return(ss.str());
}

JointPolicyPureVectorForClusteredBG* JointPolicyPureVectorForClusteredBG::Clone() const
{
    return new JointPolicyPureVectorForClusteredBG(*this);
}
