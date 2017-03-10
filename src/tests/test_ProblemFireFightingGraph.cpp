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
 * Bas Terwijn

 *
 * For contact information please see the included AUTHORS file.
 */



#include "JointPolicyPureVector.h"
#include "MADPParser.h"
#include "DecPOMDPDiscrete.h"
#include "NullPlanner.h"
#include "JointActionObservationHistoryTree.h"
#include "JointBeliefInterface.h"
#include "ProblemFireFightingGraph.h"

using namespace std;

int main()
{
    try {
    unsigned int h=2, depth=1;
    
    ProblemFireFightingGraph ff(2,3);
    ff.ConstructJointActions();
    ff.ConstructJointObservations();

//    ProblemFireFightingGraph_old ff("fireFightingGraph_2_3_3");
//    DecPOMDPDiscrete ff("","","../../problems/fireFightingGraph_2_3_3.dpomdp");
//    MADPParser parser(&ff);

    cout << ff.GetUnixName() << endl;

    ff.Print();

    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointBeliefs(false); // check without the cache
    params.SetUseSparseJointBeliefs(true);
    NullPlanner np(h,&ff,&params);

    JointPolicyPureVector *jpol=new JointPolicyPureVector(&np);
    
    // policy is <h1,h2,h4> -> <left,left,right>
    jpol->SetDepth(depth);
    jpol->SetAction(0,0,0);
    jpol->SetAction(1,0,0);
    if(ff.GetNrAgents()>2)
        jpol->SetAction(2,0,1);

//    jpol->Print();
//    ff.Print();

#if 0
    for(Index i=0;i<h;++i)
    {
        cout << np.GetFirstJointObservationHistoryIndex(i) << " "
             << np.GetFirstJointActionObservationHistoryIndex(i) << endl;            
    }
    cout << np.GetNrJointObservationHistories() << " "
         << np.GetNrJointActionObservationHistories() << endl;
#endif
    Index firstInTs=CastLIndexToIndex(np.GetFirstJointActionObservationHistoryIndex(depth)),
        lastInTs;
    if(depth==h-1)
        lastInTs=np.GetNrJointActionObservationHistories()-1;
    else
        lastInTs=CastLIndexToIndex(np.GetFirstJointActionObservationHistoryIndex(depth+1))-1;

    cout << "First index for timestep is " << firstInTs
         << ", last is " << lastInTs << endl;
    
#if 0
    double p,pJpol;
    for(Index i=firstInTs;i<=lastInTs;++i)
    {
        p=np.GetJAOHProb(i,0,0);
        pJpol=np.GetJAOHProb(i,0,0,jpol);
        cout << "jaohI " << i << " p " << p << " p with jpol " << pJpol << endl;
        cout << np.GetJointActionObservationHistoryTree(i)->
            GetJointActionObservationHistory()->SoftPrint() << endl << endl;
    }
#endif

    cout.precision(2);
//     cout.width(4);
//    cout.setf(ios::showpoint | ios::showpos | ios::scientific);
    vector<Index> JaohI;
    vector<double> pJaohI;
    vector<JointBeliefInterface*> bJaohI;
    double p;

//    cout << "Probabilities and joint beliefs for JAOHistories:" << endl;
    for(Index i=firstInTs;i<=lastInTs;++i)
    {
        JointBeliefInterface *b=np.GetNewJointBeliefInterface();
        p=np.GetJAOHProbs(b,i,0,0,jpol);
        if(p>0)
        {
#if 0
            cout << "jaohI " << i << " p " << p << " jb " << b->SoftPrint()
                 << endl;
#endif
            cout << "jaohI " << i << " = " 
                 << np.GetJointActionObservationHistoryTree(i)->
                GetJointActionObservationHistory()->SoftPrint() << endl;
            JaohI.push_back(i);
            pJaohI.push_back(p);
            bJaohI.push_back(b);
        }
    }

    cout << "Immediate reward for JAOHistories (r is from centralized model, "
         << "rE is reward per edge, and sum_rE should be equal to r):"
         << endl;
    double r,sum_rE,rE;
    for(Index i=0;i!=JaohI.size();++i)
    {
        for(Index a=0;a!=np.GetNrJointActions();++a)
        {
            r=0;
            for(Index s=0;s!=np.GetNrStates();++s)
            {
                p=bJaohI[i]->Get(s);
                if(p>0)
                    r+=p*np.GetReward(s,a); 
            } 
#if 0
            Index lastJoI=np.GetJointActionObservationHistoryTree(JaohI[i])->
                GetJointActionObservationHistory()->GetJointObservationIndex();
            cout << "jaohI " << JaohI[i] << " last jo "
                 << m.GetJointObservation(lastJoI)->SoftPrintBrief()
#else
            cout << "jaohI " << JaohI[i]
#endif
                 << "\tja " 
                 << ff.GetJointAction(a)->SoftPrintBrief()
                 << "\tr " << r << " rE <";
            sum_rE=0;
            for(Index e=0;e!=ff.GetNrLRFs();++e)
            {
                //old interface gave expected reward for belief
                // rE=ff.GetLRFReward(e,*bJaohI[i],a);
                //now we need to compute it here        
                rE = 0;
                for(Index s=0;s!=np.GetNrStates();++s)
                {
                    p=bJaohI[i]->Get(s);
                    if(p>0)
                        rE+=p*ff.GetLRFRewardFlat(e,s,a); 
                } 
                sum_rE+=rE;
                cout << " " << rE;
            }
            cout << " > sum_rE " << sum_rE;
            if(abs(r-sum_rE)<1e-8)
                cout << " equal";
            cout << endl;
        }
    }

    } catch(E& e){ e.Print(); }

    return(0);
}
