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

#include "AlphaVectorPOMDP.h"
#include "BeliefValue.h"
#include <float.h>
#include <fstream>
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "AlphaVector.h"

using namespace std;

//Default constructor
AlphaVectorPOMDP::AlphaVectorPOMDP(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu)
{
}
AlphaVectorPOMDP::AlphaVectorPOMDP(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu)
{
}

//Destructor
AlphaVectorPOMDP::~AlphaVectorPOMDP()
{
}

/**
 * Implements equation (3.18) of PhD thesis Matthijs.
 */
AlphaVector AlphaVectorPOMDP::BeliefBackup(const JointBeliefInterface &b,
                                           const GaoVectorSet &G) const
{
    int nrA=GetPU()->GetNrJointActions(),
        nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates();
    VectorSet *VS;
    // stores the index of the vector that maximizes b's value
    int maximizingVectorI;

    StartTimer("BeliefBackupPOMDP");

    VectorSet Gab(nrA,nrS);

    // compute the Gab vectors
    for(GaoVectorSetIndex a=0;a!=nrA;a++)
    {
        // initialize each Gab vector with the immediate reward
        for(int s=0;s!=nrS;s++)
            Gab(a,s)=GetPU()->GetReward(s,a);

        for(GaoVectorSetIndex o=0;o!=nrO;o++)
        {
            VS=G[a][o];
            maximizingVectorI=BeliefValue::GetMaximizingVectorIndex(b,*VS);

            // add discounted maximizing vector (3.16)
            for(int s=0;s!=nrS;s++)
                Gab(a,s)+=GetPU()->GetDiscount()*
                    (*VS)(maximizingVectorI,s);
        }
    }

    // find the maximizing Gab vector
    int maxA=BeliefValue::GetMaximizingVectorIndex(b,Gab);

    // create the vector for b
    AlphaVector newVector(nrS);
    newVector.SetAction(maxA);
    for(int s=0;s!=nrS;s++)
        newVector.SetValue(Gab(maxA,s),s);

    StopTimer("BeliefBackupPOMDP");

    return(newVector);
}

AlphaVector AlphaVectorPOMDP::BeliefBackup(const JointBeliefInterface &b,
                                           Index a,
                                           const GaoVectorSet &G) const
{
    int nrO=GetPU()->GetNrJointObservations(),
        nrS=GetPU()->GetNrStates();
    VectorSet *VS;
    // stores the index of the vector that maximizes b's value
    int maximizingVectorI;

    StartTimer("BeliefBackupPOMDP");

    vector<double> Gab(nrS);

    // initialize Gab vector with the immediate reward
    for(int s=0;s!=nrS;s++)
        Gab[s]=GetPU()->GetReward(s,a);
    
    for(GaoVectorSetIndex o=0;o!=nrO;o++)
    {
        VS=G[a][o];
        maximizingVectorI=BeliefValue::GetMaximizingVectorIndex(b,*VS);
        
        // add discounted maximizing vector (3.16)
        for(int s=0;s!=nrS;s++)
            Gab[s]+=GetPU()->GetDiscount()*
                (*VS)(maximizingVectorI,s);
    }

    // create the vector for b
    AlphaVector newVector(nrS);
    newVector.SetAction(a);
    for(int s=0;s!=nrS;s++)
        newVector.SetValue(Gab[s],s);

    StopTimer("BeliefBackupPOMDP");

    return(newVector);
}
