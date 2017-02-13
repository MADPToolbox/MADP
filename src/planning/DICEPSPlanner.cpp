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

#include "DICEPSPlanner.h"
#include "ValueFunctionDecPOMDPDiscrete.h"
#include "JPPVValuePair.h"
#include "SimulationDecPOMDPDiscrete.h"
#include "SimulationResult.h"
#include <float.h>

using namespace std;

#define DEBUG_DICEPSPlanner 0
#define DEBUG_DICEPSPlannerTIMINGS 1

DICEPSPlanner::DICEPSPlanner(
    size_t horizon,
    DecPOMDPDiscreteInterface* p,
    size_t nrRestarts,
    size_t nrIterations,
    size_t nrSamples,
    size_t nrSamplesForUpdate,
    bool use_hard_threshold, //(gamma in CE papers)
    double CEalpha, //the learning rate
    size_t nrEvalRuns, // value approximation runs (set 0 for exact eval)
    const PlanningUnitMADPDiscreteParameters * params,
    bool convergenceStats,
    ofstream * convergenceStatsFile,
    int verbose
    ) :
    PlanningUnitDecPOMDPDiscrete(horizon, p, params)
    ,_m_foundPolicy()
{
    _m_nrRestarts = nrRestarts; 
    _m_nrIterations = nrIterations;
    _m_nrSampledJointPolicies = nrSamples;
    _m_nrJointPoliciesForUpdate = nrSamplesForUpdate;
    _m_use_gamma = use_hard_threshold;
    _m_alpha = CEalpha;
    _m_nrEvalRuns = nrEvalRuns;
    _m_outputConvergenceStatistics = convergenceStats;
    _m_outputConvergenceFile = convergenceStatsFile;
    _m_verbose = verbose;
}


void DICEPSPlanner::Plan()
{
    StartTimer("DICEPS::Plan()");
    double v_best = -DBL_MAX;
    // Index jpolI_best = 0;
    JointPolicyPureVector jpol_best( this, OHIST_INDEX ); //temporary?

    //the algorithm has the following form
    //create initial joint policy distribution
    //for number of restarts
        //while improving
            //sample joint policies
            //evaluate and rank the sampled policies
            //update the probability distribution

    //get some vars:
    size_t nrAgents = GetDPOMDPD()->GetNrAgents();
    StartTimer("DICEPS::create-Xi");

    //create initial joint policy distribution
    //Xi is the parameter 'vector' for the joint probability distribution 
    //with the following form:
    //Xi[agentI][ohistI][actionI] (= Pr(actionI | ohistI, agentI) )
    vector< vector< vector<double> > > Xi (nrAgents);
    for(Index agentI=0; agentI < nrAgents; agentI++)
    {
         vector< vector<double> >& ta_vec = Xi.at(agentI);
         size_t nrOHists = GetNrObservationHistories(agentI);
         size_t nrAcs = GetDPOMDPD()->GetNrActions(agentI);
         ta_vec = vector< vector<double> >(nrOHists, 
                vector<double>(nrAcs, 1.0 / nrAcs) ); //uniform dist. at start
    }
    StopTimer("DICEPS::create-Xi");

    //for number of restarts
    for(Index restart=0; restart < _m_nrRestarts; restart++)
    {
        StartTimer("DICEPS::run(restart)");
        //for now, we use a fixed number of iterations:
        for(Index iter=0; iter < _m_nrIterations; iter++)
        {
#if DEBUG_DICEPSPlannerTIMINGS
            StartTimer("DICEPS::(CE)iteration");
#endif
            list<JPPVValuePair*> best_samples;
            double v_xth_best = -DBL_MAX;
            double v_gamma = -DBL_MAX;
            for(Index sample=0; sample < _m_nrSampledJointPolicies; sample++)
            {
#if DEBUG_DICEPSPlannerTIMINGS
                StartTimer("DICEPS::(CE)sample");
#endif
                JPPV_sharedPtr p_jpol =
                    JPPV_sharedPtr(
                        new JointPolicyPureVector( this ) );

                //sample next joint policy
                // by sampling individual policy for each agent
                vector< PolicyPureVector* > & BGpolicies = p_jpol->
                    GetIndividualPolicies();
                for(Index agentI=0; agentI < nrAgents; agentI++) {
                    SampleIndividualPolicy(*(BGpolicies.at(agentI)),Xi[agentI]);
                }

#if DEBUG_DICEPSPlanner
                {
                    cout << "sampled new policy: "<<
                        p_jpol->GetIndex();
                }
#endif                
                
                //evaluate the JointPolicy
                double v;

#if DEBUG_DICEPSPlannerTIMINGS
                StartTimer("DICEPS::(CE)sample evaluation");
#endif
                if (_m_nrEvalRuns > 0)
                { 
                    // use approximate evaluation:
                    // larger _m_nrEvalRuns yields better approximations
                    
                    v = ApproximateEvaluate(*p_jpol, _m_nrEvalRuns);
#if DEBUG_CEPOMDP
                    cout << ", approx. value= " << approxV << endl;
#endif                
                } 
                else 
                {
                    // use exact evaluation
                    
                    ValueFunctionDecPOMDPDiscrete vf(*this, *p_jpol);
                    v = vf.CalculateV<true>();
#if DEBUG_CEPOMDP
                    cout << ", value="<<v<<endl;
#endif
                }
#if DEBUG_DICEPSPlannerTIMINGS
                StopTimer("DICEPS::(CE)sample evaluation");
#endif
                

                //retain it if it ranks among the best...
                //we maintain an ordered list with contains the x best
                //policies (pol-val pairs). (x = _m_nrJointPoliciesForUpdate)
                //front() is the highest ranked policy and
                //back() the lowest ranked one.
                
                if(
                            //either we have not sampled x policies
                        (   best_samples.size() <  _m_nrJointPoliciesForUpdate
                            || 
                            //or the value of this policy is better 
                            (v > v_xth_best)
                        )
                        &&  ( (!_m_use_gamma) || (v > v_gamma)  )
                  )
                {
                    if (best_samples.size() ==  _m_nrJointPoliciesForUpdate)
                    {
#if DEBUG_DICEPSPlanner
                    cout << "best_samples full: making space...";
#endif
                        delete best_samples.back();
                        best_samples.pop_back(); //make room
                    }
                    JPPVValuePair* polval = new JPPVValuePair(p_jpol,v);
                    OrderedInsertJPPVValuePair(polval, best_samples);
                    JPPVValuePair* back = best_samples.back();
                    v_xth_best = back->GetValue();
#if DEBUG_DICEPSPlanner
                    cout << "inserted pol (v="<<v<<") - v_xth_best now:"
                        << v_xth_best << endl;
                    cout <<"best_samples contains the following pol/val pairs:"
                        << endl;
                    PrintBestSamples(best_samples);
                    cout << endl;
#endif
                }
                else
                {
//                    delete p_jpol;
                }
#if DEBUG_DICEPSPlannerTIMINGS
                StopTimer("DICEPS::(CE)sample");
#endif
            } //end for samples

            if(_m_use_gamma) //update the gamma
            {
                JPPVValuePair* back = best_samples.back();
                v_gamma = back->GetValue();
                if(_m_verbose >= 2)
                    cout << "new v_gamma="<<v_gamma<<endl;
            }

            //retain the very best sample:
            double v_best_this_iter = best_samples.front()->GetValue();
            if(v_best_this_iter > v_best)
            {
                if(_m_verbose >= 2)
                    cout << "new absolute best="<<v_best_this_iter <<
                        " (old="<< v_best <<")"<<endl;
                v_best = v_best_this_iter;
                //LIndex too short...
                //jpolI_best = best_samples.front()->GetJPPV()->GetIndex();
                jpol_best =  *(best_samples.front()->GetJPPV());
            }
                
            //update the probability distribution
            UpdateCEProbDistribution(Xi, best_samples);

#if DEBUG_DICEPSPlanner
            for(Index agentI=0; agentI < nrAgents; agentI++)
            {
                cout << "updated parameter vector for agent "<<agentI<<":";
                PrintVectorCout(Xi[agentI]);
                cout << endl;
            }
#endif
            if(_m_outputConvergenceStatistics)
            {
                size_t nrCREvals = 2000; //CR for convergence run
                double r_total = 0.0;
                JointPolicyPureVector* p_jpol=new JointPolicyPureVector(this);
                for(Index cr=0; cr < nrCREvals; cr++)
                {
                    //sample a policy from the new distribution
                    vector< PolicyPureVector* > & BGpolicies = p_jpol->
                        GetIndividualPolicies();
                    for(Index agentI=0; agentI < nrAgents; agentI++) 
                        SampleIndividualPolicy(
                                *(BGpolicies.at(agentI)), Xi[agentI]  );
                    //evaluate 1 run of this policy
                    r_total += ApproximateEvaluate(*p_jpol, 1);
                }
                delete p_jpol;
                r_total /= (double)nrCREvals;
                cout << "iteration " << iter << " ended, V(Xi)="<< r_total;
                cout << endl;
                char mean[20];
                sprintf(mean, "%.6f", r_total);
                (*_m_outputConvergenceFile) << mean <<"\t";

            }
            
            //delete list
            while(!best_samples.empty())
            {
                delete best_samples.front();
                best_samples.pop_front();
            }

#if DEBUG_DICEPSPlannerTIMINGS
            StopTimer("DICEPS::(CE)iteration");
#endif
        } //end for iterations
        StopTimer("DICEPS::run(restart)");

        if(_m_outputConvergenceStatistics)
            (*_m_outputConvergenceFile) << endl;
    } // end for restarts




    // store best found joint policy
    _m_foundPolicy = JPPV_sharedPtr(new JointPolicyPureVector(jpol_best));

    if (_m_nrEvalRuns > 0) {
        // the expected reward for the found policy is an approximation
        //so we now determine its value more accurately
        size_t Ventries = GetNrStates() * GetNrJointObservationHistories();
        //cout << "Value function entries=" << Ventries << 
            //" (S="<<GetNrStates() <<
            //", JOH="<<GetNrJointObservationHistories() <<")"<<endl;
        if(Ventries > 20000)
        {
            StartTimer("DICEPS::FoundJPolAccurateEvaluation()");
            _m_expectedRewardFoundPolicy = 
                ApproximateEvaluate(jpol_best, 20000);
            StopTimer("DICEPS::FoundJPolAccurateEvaluation()");
        }
        else
        {
            StartTimer("DICEPS::FoundJPolExactEvaluation()");
            // so it is needed to evaluate it once using the exact method
            ValueFunctionDecPOMDPDiscrete vf(*this, jpol_best);
            _m_expectedRewardFoundPolicy = vf.CalculateV<true>();
            StopTimer("DICEPS::FoundJPolExactEvaluation()");
        }
    } else { 
        // the expected reward for the found policy is already given, use it
        _m_expectedRewardFoundPolicy = v_best;
    }

    if(_m_verbose>=1)
        cout << "DICE best value after " << _m_nrRestarts << " restarts: "
             << v_best << endl;

    StopTimer("DICEPS::Plan()");
}

void DICEPSPlanner::SampleIndividualPolicy(PolicyPureVector& pol, 
    const vector< vector<double> >&  ohistActionProbs )
{
    vector< vector<double> >::const_iterator it = ohistActionProbs.begin();
    vector< vector<double> >::const_iterator last = ohistActionProbs.end();
    Index ohistI = 0;
    while(it != last)
    {
        //the action probabilities for the type pointed to by *it
        const vector<double> & action_probs = *it;
        double r = ((double)rand()) / RAND_MAX;
        double cumulativeActionProb = 0.0;

        Index aI = 0; //the action index
        vector<double>::const_iterator a_it = action_probs.begin();
        vector<double>::const_iterator a_last = action_probs.end();
        while(a_it != a_last)
        {
            double prob_aI = *a_it;
            cumulativeActionProb += prob_aI;
            if(cumulativeActionProb >= r) //action aI is sampled
                break;
            aI++;
            a_it++;
        }
        pol.SetAction(ohistI, aI);

        ohistI++;
        it++;
    }

}

void DICEPSPlanner::OrderedInsertJPPVValuePair( JPPVValuePair* pv, 
            list< JPPVValuePair*>& l)
{
    double v_pv = pv->GetValue();
    list<JPPVValuePair*>::iterator it = l.begin(); //=front - highest values
    list<JPPVValuePair*>::iterator last = l.end(); //=back - lowest values
    while(it != last)
    {
        JPPVValuePair* temp = *it;
        double val = temp->GetValue();
        if( v_pv < val )
        {
            it++;
        }
        else
        {
            l.insert(it, pv);
            return;
        }
    }
    //this should only happen when size=0
    l.insert(last, pv);
}


void DICEPSPlanner::PrintBestSamples( const list< JPPVValuePair*>& l)
{
    list<JPPVValuePair*>::const_iterator it = l.begin(); //=front - highest values
    list<JPPVValuePair*>::const_iterator last = l.end(); //=back - lowest values
    while(it != last)
    {
        JPPVValuePair* temp = *it;
        double val = temp->GetValue();
        cout << ""<<val<<", ";
        //LIndex i = temp->GetJPol()->GetIndex();
        //cout << "<p"<<i<<", v"<<val<<">,";
        it++;
    }
}




    
void DICEPSPlanner::UpdateCEProbDistribution(
            vector< vector< vector<double> > >& Xi, 
            const list<JPPVValuePair* >& best_samples)
{
    size_t nrAgents = Xi.size();
    size_t nrSamples = best_samples.size();
    //get counts
    vector< vector< vector< unsigned int > > > counts;
    list<JPPVValuePair* >::const_iterator it = best_samples.begin();
    list<JPPVValuePair* >::const_iterator last = best_samples.end();
    for(Index agI=0; agI < nrAgents; agI++)    
    {
        size_t nrH = GetNrObservationHistories(agI);
        counts.push_back( 
                vector< vector< unsigned int > >( 
                    nrH, 
                    vector<  unsigned int >(GetDPOMDPD()->GetNrActions(agI))
                )
        );
    }

    while(it != last)
    {
        vector< PolicyPureVector* > & policies = 
            (*it)->GetJPPV()->GetIndividualPolicies();
        for(Index agI=0; agI < nrAgents; agI++)
        {
            size_t nrH = GetNrObservationHistories(agI);
            for(Index ohistI=0; ohistI < nrH; ohistI++)
            {
                Index acI = policies[agI]->GetActionIndex(ohistI);
                counts[agI][ohistI][acI]++;
            }
        }
        it++;
    }
    // update
    
    for(Index agI=0; agI < nrAgents; agI++) {
        size_t nrH = GetNrObservationHistories(agI);
        for(Index ohistI=0; ohistI < nrH; ohistI++) { 
            for(Index acI=0; acI < GetDPOMDPD()->GetNrActions(agI); acI++)
            {
                double new_prob = ((double)counts[agI][ohistI][acI])/nrSamples;
                Xi.at(agI).at(ohistI).at(acI) = 
                    (1 - _m_alpha)  * Xi.at(agI).at(ohistI).at(acI) +
                    _m_alpha * new_prob;
            }
        }
    }

}

double DICEPSPlanner::ApproximateEvaluate(JointPolicyDiscrete &jpol, int nrRuns)
{
    // perform approximate evaluation of a joint policy
    // by evaluating it's value nrRuns times randomly.

    SimulationDecPOMDPDiscrete simulator(*this, nrRuns);
#if DEBUG_DICEPSPlannerTIMINGS
    StartTimer("DICEPS::(CE)sample evaluation: simulator.RunSimulations()");
#endif    
    SimulationResult simres = simulator.RunSimulations(&jpol);
#if DEBUG_DICEPSPlannerTIMINGS
    StopTimer("DICEPS::(CE)sample evaluation: simulator.RunSimulations()");
#endif    

    return simres.GetAvgReward();
}

