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
#ifndef _BGIP_SOLVERALTERNATINGMAXIMIZATION_H_
#define _BGIP_SOLVERALTERNATINGMAXIMIZATION_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "TimeTools.h"
#include <float.h>
#include "Referrer.h"

#include "BGIP_IncrementalSolverInterface_T.h"
#include "JPPVValuePair.h"
#include "PartialJPDPValuePair.h"
class JointPolicyPureVector;


#define DEBUG_BGIP_SOLVER_AM_SOLVE 0
#define DEBUG_BGIP_SOLVER_AM_SOLVE_EXTRAVERBOSE 0
#define CHECK_NEW_BR 0

/**\brief BGIP_SolverAlternatingMaximization implements an approximate
 * solver for identical payoff Bayesian games, based on alternating
 * maximization. 
 * 
 * The template argument JP represents the joint policy class the
 * solver should return.
 */
template<class JP>
class BGIP_SolverAlternatingMaximization
    : public BGIP_IncrementalSolverInterface_T<JP>
{
private:    
    
    unsigned int _m_nrRestarts;
    int _m_verbose;
    double _m_deadlineInSeconds;
    bool _m_solved;

#define DEBUG_CBR 0
    double ComputeBestResponse(JP & jpolBG, 
            Index optimizingAgentI)
        {
            if(DEBUG_CBR)
            {
                std::cout << "ComputeBestResponse called - current jpolBG:"<<std::endl;
                jpolBG.Print();
                std::cout << "value="<<this->Evaluate(jpolBG)<<std::endl;
            }
            boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bgip=
                this->GetBGIPI();
            const BayesianGameIdenticalPayoffInterface* bgipRawPtr=bgip.get();
            Index agI = optimizingAgentI;
            std::vector< std::vector<double> > v (bgip->GetNrTypes(agI), std::vector<double>(
                                            bgipRawPtr->GetNrActions(agI), 0.0) );
    
            if(DEBUG_CBR)
                std::cout << "ComputeBestResponse: compute all type-action values for agent agI..."<<std::endl;
            //compute all type-action values for agent agI
            for(Index jt = 0; jt < bgipRawPtr->GetNrJointTypes(); jt++)
            {
                std::vector<Index> types = bgipRawPtr->JointToIndividualTypeIndices(jt);
                Index type = types.at(agI);
                double jtProb = bgipRawPtr->GetProbability(jt);
                Index jaI = jpolBG.GetJointActionIndex(jt);
                std::vector<Index> avec = bgipRawPtr->JointToIndividualActionIndices(jaI);
                for(Index acI =0 ; acI < bgipRawPtr->GetNrActions(agI); acI++)
                {
                    avec.at(agI) = acI;
                    Index jaInew = bgipRawPtr->IndividualToJointActionIndices(avec);
                    double u =  bgipRawPtr->GetUtility(jt, jaInew);
                    v[type][acI] +=  jtProb * u;
                    if(DEBUG_CBR)
                        std::cout << "updated v[type][acI]="<<v[type][acI]<< 
                            "(jtProb=" << jtProb << ", u=" << u <<")" <<std::endl;
                }
            }
            if(DEBUG_CBR)
                std::cout << "ComputeBestResponse: compute the best response and value..."<<std::endl;
            //compute the best response and value
            double v_best = 0.0;
            for(Index type = 0; type < bgipRawPtr->GetNrTypes(agI); type++)
            {
                double v_t = -DBL_MAX;
                Index best_a = 0;
                for(Index acI = 0; acI < bgipRawPtr->GetNrActions(agI); acI++)
                {
                    double v_t_a = v[type][acI];
                    if(v_t_a > v_t)
                    {
                        best_a = acI;
                        v_t = v_t_a;
                    }
                }
                v_best += v_t;
                jpolBG.SetAction(agI, type, best_a);
            }
            return v_best;
        }

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverAlternatingMaximization(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
            unsigned int nrRestarts=10, 
            int verbose=0,
            size_t nrSolutions=1) :
        BGIP_IncrementalSolverInterface_T<JP>(bg, nrSolutions),
        _m_nrRestarts(nrRestarts),
        _m_verbose(verbose),
        _m_solved(false)
        {
        }

    /// Destructor.
    ~BGIP_SolverAlternatingMaximization(){};
    double Solve()
        {
            this->InitDeadline();

            if(_m_verbose >= 1)
                std::cout << "BGIP_SolverAlternatingMaximization::Solve() started"<<std::endl;
            boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bgip=
                this->GetBGIPI();
    
            struct timeval start_time, cur_time;
            if(gettimeofday(&start_time, NULL) != 0)
                throw "Error with gettimeofday";

            boost::shared_ptr<JP> temp =  boost::dynamic_pointer_cast<JP>( this->GetNewJpol() );
            JP jpolBG(*temp);
//            delete temp;
            double vmax = -DBL_MAX;
            
            for(Index r=0; r < _m_nrRestarts; ++r)
            {
                this->CheckDeadline("AM deadline exceeded");

                if(_m_verbose >= 1)
                    std::cout << "AM: new restart"<<std::endl;
                double v_thisAMrun = -DBL_MAX;
                jpolBG.RandomInitialization();

                //eval the expected payoff of this jpolBG
                double v = this->Evaluate(jpolBG);
                
                v_thisAMrun = v;
                Index optimizingAgentI = 0;
#if DEBUG_BGIP_SOLVER_AM_SOLVE
                {
                    std::cout<<"\n>new AM restart - initialized with v="<<
                        v_thisAMrun;
#if DEBUG_BGIP_SOLVER_AM_SOLVE_EXTRAVERBOSE
                    std::cout << " - pol:"<<std::endl;
                    jpolBG.Print();
#endif
                    std::cout << std::endl;
                }
#endif
                
                bool improving = true;
                size_t nr_non_improving_agents = 0;

                while(nr_non_improving_agents <  bgip->GetNrAgents() ) 
                {
                    this->CheckDeadline("AM deadline exceeded");

                    if(_m_verbose >= 2 )
                        std::cout << "AM: computing new best response...";
                    improving = false;//unless improvement is found
                    //find best response
                    v = ComputeBestResponse(jpolBG, optimizingAgentI);
                    if(_m_verbose >= 2)
                        std::cout << "v="<<v<<std::endl;
#if CHECK_NEW_BR            
                    JP checkjpolBG = jpolBG;
                    double vcheck = 
                        ComputeBestResponseOld(checkjpolBG, optimizingAgentI);
                    if(jpolBG.GetIndex() != checkjpolBG.GetIndex())
                    {
                        std::cout << "Error best response computation failed!!!"<<std::endl;
                        std::cout << "v="<<v<<", vcheck="<<vcheck<<std::endl;
                        std::cout << "jpolBG="<< jpolBG.SoftPrint();
                        std::cout << "checkjpolBG="<< jpolBG.SoftPrint();
                    }
#endif            
#if DEBUG_BGIP_SOLVER_AM_SOLVE            
                    {
                        std::cout<<"best response achieves v="<< v ;
#if DEBUG_BGIP_SOLVER_AM_SOLVE_EXTRAVERBOSE
                        std::cout << " - pol:"<<std::endl;
                        jpolBG.Print();
#endif
                        std::cout << std::endl;
                    }
#endif

                    if(v > v_thisAMrun  + 1e-9)
                    {
#if DEBUG_BGIP_SOLVER_AM_SOLVE            
                        std::cout << "(improving)" <<std::endl;
#endif
                        v_thisAMrun = v;
                        improving = true;                
                    }

                    //next agent
                    if(improving)
                        nr_non_improving_agents = 0;
                    else
                        nr_non_improving_agents++;

                    optimizingAgentI = (optimizingAgentI+1) % bgip->GetNrAgents();
                } // end of this alternating maximization run        
                if(DEBUG_BGIP_SOLVER_AM_SOLVE)
                    std::cout << "AM run ended" <<std::endl;

                size_t oldNrSolutionsFound=
                    this->GetNrFoundSolutions();

                //store the solution
                this->AddSolution( jpolBG, v_thisAMrun );

                if(_m_verbose >= 1)
                    if(this->GetNrFoundSolutions()
                       != (oldNrSolutionsFound+1))
                        std::cout << "AM: computed policy not added to BGIPSolution, probably a duplicate" << std::endl;
                
                if(v_thisAMrun > vmax)
                {
                    vmax = v_thisAMrun;
                    if(DEBUG_BGIP_SOLVER_AM_SOLVE)
                        std::cout << "(best solution)" <<std::endl;
                    
                    if(this->GetWriteAnyTimeResults()){
                        double delta  = TimeTools::GetDeltaTimeDouble(start_time, cur_time);
                        (*this->GetResultsOFStream()) << vmax << "\t";
                        (*this->GetTimingsOFStream()) << delta << "\t";
                    }
                }
            }
            //end the line in the results file
            if(this->GetWriteAnyTimeResults()){
                (*this->GetResultsOFStream()) << std::endl;
                (*this->GetTimingsOFStream()) << std::endl;
            } 
            
#if 0
            //check the solution
            std::cout << "BGIP_SolverAlternatingMaximization finished, first solution: " << 
                _m_solution.GetNextSolution()->SoftPrint() << std::endl;
#endif
            return(vmax);
        }

    bool IsExactSolver() const { return(false); }

    void SetCBGupperBound(double upperbound){}// _m_CBGupperbound=upperbound; }
    void SetCBGlowerBound(double lb){}

    bool GetNextJointPolicyAndValueSpecific(boost::shared_ptr<JointPolicyDiscretePure> &jpol, double &value)
    {
        if(!_m_solved)
            Solve();

        // since BFS computes all solutions. we just need to get
        // the next one from the solution and return it

        bool foundSolution = false;
        value = 0;

        // this requires some magic to make sure we return from
        // the correct pool, which depends on the JP type
        if(!this->IsEmptyJPPV())
        {                
            const boost::shared_ptr<JPPVValuePair> jppv=
                this->GetNextSolutionJPPV();
            value = jppv->GetValue();
        }
        else if(!this->IsEmptyPJPDP())
        {
            //alright, if I understand it correctly, it tries to convert the policy returned
            //to a "JointPolicyPureVectorForClusteredBG"

            const boost::shared_ptr<PartialJPDPValuePair> jppv=
                this->GetNextSolutionPJPDP();
            value = jppv->GetValue();
        }

        if(!foundSolution)
        {
//                std::cout << "value " << value << " " <<  this->_m_nrSolutionsReturned <<  " "
//                          << this->GetNrDesiredSolutions() << " " 
//                          << this->GetBGIPI()->GetNrJointPolicies() << std::endl;
            jpol.reset();
            value = 0;
        }

        return(foundSolution);
    }

};


#endif /* !_BGIP_SOLVERALTERNATINGMAXIMIZATION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
