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

#include "BGforStageCreation.h"
#include "PlanningUnitMADPDiscrete.h"
#include "JointPolicyDiscretePure.h"

using namespace std;
/*
//Default constructor
BGforStageCreation::BGforStageCreation()
{
}
//Copy constructor.    
BGforStageCreation::BGforStageCreation(const BGforStageCreation& o) 
{
}
//Destructor
BGforStageCreation::~BGforStageCreation()
{
}
//Copy assignment operator
BGforStageCreation& BGforStageCreation::operator= (const BGforStageCreation& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/




void  BGforStageCreation::Fill_FirstOHtsI(
        const PlanningUnitMADPDiscrete* pu,
        Index ts, 
        vector<Index>& firstOHtsI
        )
{
    firstOHtsI.clear();
    //because the OHs are constructed breath-first, we know the OHs for agent i
    //for this time step are numbered:
    //firstOHtsGI[i]...firstOHtsGI[i]+nrOH[i]-1
    //
    //(read first-OH-for-time-step-ts its Global Index)
    //
    //i.e. ohGI = ohI + firstOHtsGI 

    for(Index agI=0; agI < pu->GetNrAgents(); agI++)
    {
        Index fI = CastLIndexToIndex(pu->GetFirstObservationHistoryIndex(agI, ts));
        firstOHtsI.push_back(fI); 
    }
}


void BGforStageCreation::Fill_joI_Array(
        const PlanningUnitMADPDiscrete* pu,
        const Index ts, 
        const vector<Index>& indTypes, 
        const vector<Index>& firstOHtsI, 
        Index* joI_arr)
{           
    //convert indiv type indices to ind. observation history indices:
    vector<Index> indOHI = vector<Index>(indTypes);
    // indivObservations[ti][agI]  will contain the observation agI received at tI+1
    vector< vector<Index> > indivObservations(ts,vector<Index>(
                pu->GetNrAgents() ) );
    for(Index agentI=0; agentI < pu->GetNrAgents(); agentI++)
    {
        indOHI[agentI] += firstOHtsI[agentI];
        Index obsArr[ts];
        pu->GetObservationHistoryArrays(agentI, indOHI[agentI], ts, obsArr);
        //now obsArr is filled and can be copied into indivObservations
        for(Index tI=0; tI < ts; tI++)
            indivObservations.at(tI).at(agentI) = obsArr[tI];
    }

    for(Index tI=0; tI < ts; tI++)
        joI_arr[tI] = pu->IndividualToJointObservationIndices(
                indivObservations[tI] );


}


//compute the joint actions taken bu jpolPrevTs when joIs is the true joint 
//observation history at stage ts.
void BGforStageCreation::Fill_jaI_Array(
        const PlanningUnitMADPDiscrete* pu,
        Index ts, 
        Index joIs[], //the array of joint observations issued 
        const boost::shared_ptr<const JointPolicyDiscretePure> &jpolPrevTs, 
        Index* jaI_arr
        )
{
    Index johI = 0;
    Index t = 0;
    while(t < ts)
    {
        Index ja = jpolPrevTs->GetJointActionIndex(johI);
        jaI_arr[t] = ja;

        Index next_joI = joIs[t];
        johI = pu->GetSuccessorJOHI(johI, next_joI);
        t++;
    }
}

