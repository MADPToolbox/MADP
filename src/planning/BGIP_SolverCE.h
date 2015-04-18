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

/* Only include this header file once. */
#ifndef _BGIP_SOLVERCE_H_
#define _BGIP_SOLVERCE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "TimeTools.h"
#include "JPPVValuePair.h"
#include "EDeadline.h"

#include <float.h>
#include <list>

#define DEBUG_BGIP_SOLVER_CE 0
//#define DEBUG_EXHBR 0
//#define DEBUG_EXJESP 0

#define DEBUG_BGIP_SOLVER_CE_PRINTOUTPROGRESS 0

/** 
 * BGIP_SolverCE is a class that performs Cross Entropy optimization
 * for identical payoff Bayesian Games.
 */
// Matthijs: I tried to templatize this class, so that it works on
// JointPolicyPureVectorForClusteredBG as well, but I ran into
// problems because it uses JPPVValuePair
//template<class JP>
class BGIP_SolverCE : public BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector>
{
private:    
    size_t _m_nrRestarts;
    size_t _m_nrIterations;
    size_t _m_nrSampledJointPolicies;
    size_t _m_nrJointPoliciesForUpdate;
    bool _m_use_gamma;
    double _m_alpha;

protected:

    static void SampleIndividualPolicy(PolicyPureVector& pol, 
                                       const std::vector<std::vector<double> >&
                                       typeActionProbs)
        {
            std::vector< std::vector<double> >::const_iterator it = typeActionProbs.begin();
            std::vector< std::vector<double> >::const_iterator last = typeActionProbs.end();
            Index typeI = 0;
            while(it != last)
            {
                //the action probabilities for the type pointed to by *it
                const std::vector<double> & action_probs = *it;
                double r = ((double)rand()) / RAND_MAX;
                double cumulativeActionProb = 0.0;
                
                Index aI = 0; //the action index
                std::vector<double>::const_iterator a_it = action_probs.begin();
                std::vector<double>::const_iterator a_last = action_probs.end();
                while(a_it != a_last)
                {
                    double prob_aI = *a_it;
                    cumulativeActionProb += prob_aI;
                    if(cumulativeActionProb >= r) //action aI is sampled
                        break;
                    aI++;
                    a_it++;
                }
                pol.SetAction(typeI, aI);
                
                typeI++;
                it++;
            }
        }


    static void OrderedInsertJPPVValuePair(JPPVValuePair* pv, 
                                           std::list<JPPVValuePair*>& l)
        {
            double v_pv = pv->GetValue();
            std::list<JPPVValuePair*>::iterator it = l.begin(); //=front - highest values
            std::list<JPPVValuePair*>::iterator last = l.end(); //=back - lowest values
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

    static void PrintBestSamples(const std::list<JPPVValuePair*>& l)
        {
            std::list<JPPVValuePair*>::const_iterator it = l.begin(); //=front - highest values
            std::list<JPPVValuePair*>::const_iterator last = l.end(); //=back - lowest values
            while(it != last)
            {
                JPPVValuePair* temp = *it;
                double val = temp->GetValue();
                //LIndex i = temp->GetJPPV()->GetIndex();
                std::cout << ""<<val<<", ";
                //std::cout << "<p"<<i<<", v"<<val<<">,";
                it++;
            }
        }

    
    void UpdateCEProbDistribution(
        std::vector< std::vector< std::vector<double> > >& Xi, 
        const std::list<JPPVValuePair* >& best_samples)
        {
            size_t nrAgents = Xi.size();
            size_t nrSamples = best_samples.size();
            //get counts
            std::vector< std::vector< std::vector< unsigned int > > > counts;
            std::list<JPPVValuePair* >::const_iterator it = best_samples.begin();
            std::list<JPPVValuePair* >::const_iterator last = best_samples.end();
            for(Index agI=0; agI < nrAgents; agI++)    
            {
                size_t nrT =  this->GetBGIPI()->
                    GetNrPolicyDomainElements(agI, TYPE_INDEX);
                counts.push_back( 
                    std::vector< std::vector< unsigned int > >( 
                        nrT, 
                        std::vector<  unsigned int >(this->GetBGIPI()->GetNrActions(agI))
                        )
                    );
            }
            
            while(it != last)
            {
                boost::shared_ptr<JointPolicyPureVector> p_jpol= (*it)->GetJPPV();
                std::vector< PolicyPureVector* > & policies = 
                    p_jpol->GetIndividualPolicies();
                for(Index agI=0; agI < nrAgents; agI++)
                {
                    for(Index typeI=0; typeI < this->GetBGIPI()->
                            GetNrPolicyDomainElements(agI, TYPE_INDEX); typeI++)
                    {
                        Index acI = policies[agI]->GetActionIndex(typeI);
                        counts[agI][typeI][acI]++;
                    }
                }
                it++;
            }
            // update
    
            for(Index agI=0; agI < nrAgents; agI++)
                for(Index typeI=0; typeI < this->GetBGIPI()->
                        GetNrPolicyDomainElements(agI, TYPE_INDEX); typeI++)
                    for(Index acI=0; acI < this->GetBGIPI()->GetNrActions(agI); acI++)
                    {
                        double new_prob = ((double)counts[agI][typeI][acI])/nrSamples;
                        Xi.at(agI).at(typeI).at(acI) = 
                            (1 - _m_alpha)  * Xi.at(agI).at(typeI).at(acI) +
                            _m_alpha * new_prob;
                    }
        }

public:
    // Constructor, destructor and copy assignment.
    // (default) Constructor
    //BGIP_SolverCE();
    /**Constructor. Directly Associates a problem with the planner
     * Information regarding the problem is used to construct a joint policy
     * of the proper shape.*/
    BGIP_SolverCE(const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
        size_t nrCERestarts = 10,
        size_t nrIterations = 30,
        size_t nrSamples = 40,
        size_t nrSamplesForUpdate =15,
        bool use_hard_threshold = true, //(gamma in CE papers)
        double CEalpha = 0.3 //the learning rate
        ) :
        BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector>(bg)
        {
            _m_nrRestarts = nrCERestarts;
            _m_nrIterations = nrIterations;
            _m_nrSampledJointPolicies = nrSamples;
            _m_nrJointPoliciesForUpdate = nrSamplesForUpdate;
            _m_use_gamma = use_hard_threshold;
            _m_alpha = CEalpha;
        }

    double Solve()
        {
            InitDeadline();

            double v_best = -DBL_MAX;
            //Index jpolI_best = 0;
            JointPolicyPureVector jpol_best(this->GetBGIPI(), TYPE_INDEX); //temporary?
            
            struct timeval start_time, cur_time;
            if(gettimeofday(&start_time, NULL) != 0)
                throw "Error with gettimeofday";
            
        //the algorithm has the following form
        //create initial joint policy distribution
        //for number of restarts
            //while improving
                //sample joint policies
                //evaluate and rank the sampled policies
                //update the probability distribution

            //get some vars:
            size_t nrAgents = this->GetBGIPI()->GetNrAgents();
            size_t nrJT = this->GetBGIPI()->GetNrJointTypes();

            //create initial joint policy distribution
            //Xi is the parameter 'std::vector' for the joint probability distribution 
            //with the following form:
            //Xi[agentI][typeI][actionI] (= Pr(actionI | typeI, agentI) )
            std::vector< std::vector< std::vector<double> > > Xi (nrAgents);
            for(Index agentI=0; agentI < nrAgents; agentI++)
            {
                size_t nrTypes = 
                    this->GetBGIPI()->GetNrPolicyDomainElements(agentI, 
                                                             TYPE_INDEX);
                size_t nrAcs = this->GetBGIPI()->GetNrActions(agentI);
                std::vector< std::vector<double> > ta_vec =
                    std::vector< std::vector<double> >(nrTypes, 
                                             std::vector<double>(nrAcs, 1.0 / nrAcs) ); //uniform dist. at start
                Xi.at(agentI)=ta_vec;
            }

            //for number of restarts
            for(Index restart=0; restart < _m_nrRestarts; restart++)
            {
                //for now, we use a fixed number of iterations:
                for(Index iter=0; iter < _m_nrIterations; iter++)
                {
                    CheckDeadline("CE deadline exceeded");

                    std::list<JPPVValuePair*> best_samples;
                    double v_xth_best = -DBL_MAX;
                    double v_gamma = -DBL_MAX;
                    for(Index sample=0; sample < _m_nrSampledJointPolicies; sample++)
                    {
                        boost::shared_ptr<JointPolicyPureVector> p_jpol =
                            boost::shared_ptr<JointPolicyPureVector>(new JointPolicyPureVector( this->GetBGIPI() ));
                        //sample next joint policy
                        std::vector< PolicyPureVector* > & BGpolicies = p_jpol->
                            GetIndividualPolicies();
                        for(Index agentI=0; agentI < nrAgents; agentI++)
                            //sample individual policy for next agent
                            SampleIndividualPolicy(*(BGpolicies.at(agentI)),Xi[agentI]);
#if DEBUG_BGIP_SOLVER_CE
                        {
                            std::cout << "sampled new policy: "<<
                                p_jpol->GetIndex();
                        }
#endif                
                        //evaluate 
                        double v = 0.0;        

                        const BayesianGameIdenticalPayoffInterface* bgipRawPtr=this->GetBGIPI().get();
                        for(Index jt = 0; jt < nrJT; jt++)
                        {
                            if(jt % 10000 == 0)
                                CheckDeadline("CE deadline exceeded");

                            double P_jt = bgipRawPtr->GetProbability(jt);
                            if(P_jt>0)
                            {
                                const std::vector<Index>& indTypes = bgipRawPtr->
                                    JointToIndividualTypeIndices(jt);
                                std::vector<Index> indAcs(nrAgents, 0);
                                //determine action of agentI 
                                for(Index agentI=0; agentI < nrAgents; agentI++)
                                    indAcs.at(agentI) = BGpolicies.at(agentI)->
                                        GetActionIndex(indTypes.at(agentI));
                                
                                Index ja = bgipRawPtr->IndividualToJointActionIndices(indAcs);
                                v += P_jt * bgipRawPtr->GetUtility(jt, ja);
                            }
                        }
#if DEBUG_BGIP_SOLVER_CE
                        std::cout << ", value="<<v<<std::endl;
#endif                
                        //retain it if it ranks among the best...
                        //we maintain an ordered std::list with contains the x best
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
#if DEBUG_BGIP_SOLVER_CE
                                std::cout << "best_samples full: making space...";
#endif
                                delete best_samples.back();
                                best_samples.pop_back(); //make room
                            }
                            JPPVValuePair* polval = new JPPVValuePair(p_jpol,v);
                            OrderedInsertJPPVValuePair(polval, best_samples);
                            JPPVValuePair* back = best_samples.back();
                            v_xth_best = back->GetValue();
#if DEBUG_BGIP_SOLVER_CE
                            std::cout << "inserted pol (v="<<v<<") - v_xth_best now:"
                                 << v_xth_best << std::endl;
                            std::cout <<"best_samples contains the following pol/val pairs:"
                                 << std::endl;
                            PrintBestSamples(best_samples);
                            std::cout << std::endl;
#endif
                        }
                        else
                        {
//                            delete p_jpol;
                        }
                    } //end for samples
                    if(_m_use_gamma) //update the gamma
                    {
                        JPPVValuePair* back = best_samples.back();
                        v_gamma = back->GetValue();
#if DEBUG_BGIP_SOLVER_CE
                        std::cout << "new v_gamma="<<v_gamma<<std::endl;
#endif
                    }
                    //retain the very best sample:
                    double v_best_this_iter = best_samples.front()->GetValue();
                    if(v_best_this_iter > v_best)
                    {
#if DEBUG_BGIP_SOLVER_CE
                        std::cout << "new absolute best="<<v_best_this_iter <<
                            " (old="<< v_best <<")"<<std::endl;
#endif
                        v_best = v_best_this_iter;
                        if(this->GetWriteAnyTimeResults()){
                            double delta  = TimeTools::GetDeltaTimeDouble(start_time, cur_time);

                            (*this->GetResultsOFStream()) << v_best << "\t";
                            (*this->GetTimingsOFStream()) << delta << "\t";
                            
                        }
                        //LIndex too short...
                        //jpolI_best = best_samples.front()->GetJPPV()->GetIndex();
                        jpol_best =  *(best_samples.front()->GetJPPV());
                    }
                
                    //update the probability distribution
                    UpdateCEProbDistribution(Xi, best_samples);
#if DEBUG_BGIP_SOLVER_CE
                    for(Index agentI=0; agentI < nrAgents; agentI++)
                    {
                        std::cout << "updated parameter std::vector for agent "<<agentI<<":";
                        PrintVectorCout(Xi[agentI]);
                        std::cout << std::endl;
                    }
#endif
                    //delete std::list
                    while(!best_samples.empty())
                    {
                        delete best_samples.front();
                        best_samples.pop_front();
                    }
                } //end iteration
            } // end restart
            //end the line in the results file
            if(this->GetWriteAnyTimeResults()){
                (*this->GetResultsOFStream()) << std::endl;
                (*this->GetTimingsOFStream()) << std::endl;
            } 

            //JointPolicyPureVector best_found(*this->GetBGIPI());
            //best_found.SetIndex(jpolI_best);
            //_m_solution.SetPolicy(best_found);

            this->AddSolution(jpol_best,v_best);
            
            return(v_best);
        }

    bool IsExactSolver() const { return(false); }
        
    void SetCBGupperBound(double upperbound){}// _m_CBGupperbound=upperbound; }
    void SetCBGlowerBound(double lb){}
};


#endif /* !_BGIP_SOLVERCE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
