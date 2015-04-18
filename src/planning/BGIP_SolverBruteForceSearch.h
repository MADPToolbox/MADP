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
#ifndef _BGIP_SOLVERBRUTEFORCESEARCH_H_
#define _BGIP_SOLVERBRUTEFORCESEARCH_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BGIP_IncrementalSolverInterface_T.h"
#include "JointPolicyPureVector.h"
#include <float.h>
#include "TimeTools.h"
#include "Referrer.h"
#include "EDeadline.h"
#include <sys/times.h>
#include "JPPVValuePair.h"
#include "PartialJPDPValuePair.h"

#define DEBUG_BGIP_SOLVER_BFS 0
#define DEBUG_BGIP_SOLVER_BFS_PRINTOUTPROGRESS 0

#include <typeinfo>

/**\brief BGIP_SolverBruteForceSearch is a class that performs Brute
 * force search for identical payoff Bayesian Games.
 *
 * The template argument JP represents the joint policy class the
 * solver should return.
 */
template<class JP>
class BGIP_SolverBruteForceSearch : public BGIP_IncrementalSolverInterface_T<JP>
{
private:    
    ///stores the MaxPlus parameter for the verbosity level
    size_t _m_verbosity;
    /// Stores the maximum execution time.
    /** If the solver expects it will take more time, it aborts an
     * expection. */
    size_t _m_deadlineInSeconds;

    /// Keeps track whether Solve() has been called.
    bool _m_solved;

    double _m_CBGlowerBound;
    double _m_CBGupperBound;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    // (default) Constructor
    //BGIP_SolverBruteForceSearch();
    /**Constructor. Directly Associates a problem with the planner
     * Information regarding the problem is used to construct a joint policy
     * of the proper shape.*/
    BGIP_SolverBruteForceSearch(const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
                                size_t verbose = 0, size_t nrDesiredSolutions = INT_MAX,
                                size_t deadlineInSeconds = 0) :
        BGIP_IncrementalSolverInterface_T<JP>(bg,nrDesiredSolutions),
        _m_verbosity(verbose),
        _m_deadlineInSeconds(deadlineInSeconds),
        _m_solved(false),
        _m_CBGupperBound(DBL_MAX)
        {}

    double Solve()
    {    
        _m_solved = true;
        struct timeval start_time, cur_time;
        if(gettimeofday(&start_time, NULL) != 0)
            throw "Error with gettimeofday";

        tms ts_before, ts_after;
        times(&ts_before);
        LIndex checkPointForDeadline = 1000000;

        bool round = false;

        boost::shared_ptr<JP> jpol = 
            boost::dynamic_pointer_cast<JP>(this->GetNewJpol());
        boost::shared_ptr<JP> best = 
            boost::dynamic_pointer_cast<JP>(this->GetNewJpol());

        double v_best = -DBL_MAX;
        double v = 0.0;

        int i = 0;
        if(DEBUG_BGIP_SOLVER_BFS)
            std::cout<<"Starting Bruteforce search - v_best is init to "<<v_best<<std::endl;
#if DEBUG_BGIP_SOLVER_BFS_PRINTOUTPROGRESS
        LIndex nrJPols = this->GetBGIPI()->GetNrJointPolicies();
#endif

        size_t nrJT = this->GetBGIPI()->
            GetNrJointTypes();
        size_t nrDesiredSolutions = this->GetNrDesiredSolutions();
        
        while(!round)
        {
            if(DEBUG_BGIP_SOLVER_BFS){std::cout << "Jpol#"<< i << " - ";}
#if DEBUG_BGIP_SOLVER_BFS_PRINTOUTPROGRESS
            if(i % 1000 == 0&& i > 1000)
            {
                std::cout << "Jpol #"<< i << " of " << nrJPols<< " - ";
                printf("%.4f", ((double)i / nrJPols) * 100 );
                std::cout << "%" << std::endl;
            }
#endif
            //this checks whether BFS has used more time then allowed 
            //(the deadline) and throws an exception if it has.
            if(_m_deadlineInSeconds && (i % checkPointForDeadline) == 0)
            {
                times(&ts_after);
                clock_t utime = ts_after.tms_utime -
                    ts_before.tms_utime;
                double timeSpentInS = static_cast<double>(utime) / sysconf(_SC_CLK_TCK);

                // we don't want to base our estimate on too few
                // time, so make it run at least 2s
                if(timeSpentInS < 2)
                    checkPointForDeadline *= 2;
                else
                {
                    LIndex nrJPols = this->
                        GetBGIPI()->GetNrJointPolicies();
                    double expectedTimeNeeded=
                        (CastLIndexToDouble(nrJPols)/
                         CastLIndexToDouble(checkPointForDeadline))*
                        timeSpentInS;
                    if(expectedTimeNeeded > (1.5*_m_deadlineInSeconds))
                    {
                        std::stringstream ss;
                        ss << "BGIP_SolverBruteForceSearch::Solve after " 
                           << checkPointForDeadline
                           << " we spent " << timeSpentInS << "s, for all " << nrJPols
                           << " jpols we expect to take " << expectedTimeNeeded
                           << "s, which is above the deadline (>1.5*" 
                           << _m_deadlineInSeconds << "s), bailing out";
                        EDeadline e(ss.str(),expectedTimeNeeded);
                        throw(e);
                    }
                }
            }
            ++i;
            
            v = 0.0;

            JP *jpolRawPtr = jpol.get();
            boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> 
                bgip= this->GetBGIPI();
            const BayesianGameIdenticalPayoffInterface *bgipRawPtr=
                bgip.get();
            for(Index jt = 0; jt < nrJT; ++jt)
            {
                double P_jt = bgipRawPtr->GetProbability(jt);
                Index ja = jpolRawPtr->GetJointActionIndex(jt);  
                v += P_jt * bgipRawPtr->GetUtility(jt, ja);
            }

            if(DEBUG_BGIP_SOLVER_BFS) std::cout << "Expected value = "<< v;
            if(v > v_best)
            {
                if(DEBUG_BGIP_SOLVER_BFS) std::cout << " -> new best policy!!!";
                v_best = v;
                *best = *jpol; 
                if(this->GetWriteAnyTimeResults()){
                    double delta  = TimeTools::GetDeltaTimeDouble(start_time, cur_time);
                    (*this->GetResultsOFStream()) << v_best << "\t";
                    (*this->GetTimingsOFStream()) << delta << "\t";
                }
            }

            if(nrDesiredSolutions == 1 &&
               (v >= (_m_CBGupperBound-PROB_PRECISION)))
            {
                std::cout << "Hit CBG upperbound" << std::endl;
                this->AddSolution( *jpol, v );
                return(v);
                break;
            }

            // if we want more than just the single best solution, try to add all to the solution
            if(nrDesiredSolutions>1)
            {
                this->AddSolution( *jpol, v );
//                      std::cout << "added solution with value " << v << ", best is "
//                                << this->GetPayoff() << std::endl;
//                      std::cout << this->GetSolution().SoftPrint();
            }
            if(DEBUG_BGIP_SOLVER_BFS) {    
                std::cout <<  std::endl << "Incrementing joint policy..."<<std::endl; 
            }    
            round = ++(*jpol);
        }
        //end the line in the results file
        if(this->GetWriteAnyTimeResults()){
            (*this->GetResultsOFStream()) << std::endl;
            (*this->GetTimingsOFStream()) << std::endl;
        } 

        // if _m_nrSolutions>1 then we already added this to the queue
        if(nrDesiredSolutions == 1)
            this->AddSolution( *best, v_best );

        return(v_best);
    }

    bool IsExactSolver() const { return(true); }


#define DGNJPAV 0
    // I guess this gives a reference to the pointer, such that chaning the pointer here, will update
    // the pointer in the calling function ? (cryptic...)
    bool GetNextJointPolicyAndValueSpecific(boost::shared_ptr<JointPolicyDiscretePure> &jpol, double &value)
    {
#if DGNJPAV
        std::cout << "DEBUG: GetNextJointPolicyAndValue called" <<std::endl <<
            "\texpecting a pointer to a "<< typeid(jpol).name() << std::endl;

#endif

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
#if DGNJPAV
            std::cout << "DEBUG: GetNextJointPolicyAndValue - selecting from JPPV queue" <<std::endl;
#endif
            const boost::shared_ptr<JPPVValuePair> jppv=
                this->GetNextSolutionJPPV();
            value = jppv->GetValue();
            if(value >= _m_CBGlowerBound)
            {
                boost::shared_ptr<JP> test = boost::dynamic_pointer_cast<JP>(jppv->GetJPPV());
                if(test)
                {
                    foundSolution = true;
                    jpol = boost::static_pointer_cast<JointPolicyDiscretePure>(test);
                    this->PopNextSolutionJPPV();
                }
            }
        }
        else if(!this->IsEmptyPJPDP())
        {
            //alright, if I understand it correctly, it tries to convert the policy returned
            //to a "JointPolicyPureVectorForClusteredBG"
            //
#if DGNJPAV
            std::cout << "DEBUG: GetNextJointPolicyAndValue - selecting from PJPDP queue" <<std::endl;
#endif
            const boost::shared_ptr<PartialJPDPValuePair> jppv=
                this->GetNextSolutionPJPDP();
            value = jppv->GetValue();
            if(value >= _m_CBGlowerBound)
            {
                
                boost::shared_ptr<PartialJointPolicyDiscretePure> pjpdp = jppv->GetJPol();
                boost::shared_ptr<JP> test = boost::dynamic_pointer_cast<JP>(pjpdp);
#if DGNJPAV
                std::cout << "\tconversion requested from "<<  typeid( jppv->GetJPol() ).name() 
                          << " to " <<              typeid( test ).name()
                          << std::endl
                          << "\tthe former is a pointer to " <<  typeid( *(jppv->GetJPol()) ).name() 
                          << " ." << std::endl;
#endif
                if(test)
                {
#if DGNJPAV
                    std::cout << "DEBUG: GetNextJointPolicyAndValue - PJPDP conversion success" <<std::endl;
#endif
                    foundSolution = true;
                    //jpol = test;
                    jpol = boost::static_pointer_cast<JointPolicyDiscretePure>(test);
                    this->PopNextSolutionPJPDP();
                }
                else
                {
#if DGNJPAV
                    std::cout << "DEBUG: GetNextJointPolicyAndValue - PJPDP conversion failed!" <<std::endl;
#endif
                    throw E("DEBUG: GetNextJointPolicyAndValue - PJPDP conversion failed!");
                }
            }
        }
        else
        {
#if DGNJPAV
            std::cout << "DEBUG: GetNextJointPolicyAndValue - selecting ERROR - both queues empty?!?" <<std::endl;
#endif
        }

        if(!foundSolution)
        {
//                std::cout << "value " << value << " " <<  this->_m_nrSolutionsReturned <<  " "
//                          << this->GetNrDesiredSolutions() << " " 
//                          << this->GetBGIPI()->GetNrJointPolicies() << std::endl;
            jpol.reset();
            value = 0;
        }
//             std::cout << "found jpol " << jpol << " value " << value << " and " 
//                       << this->GetNrFoundSolutions()
//                       << " solutions left" 
//                       << std::endl;

        return(foundSolution);
    }

    void SetCBGlowerBound(double lowerbound) { _m_CBGlowerBound = lowerbound; }

    void SetCBGupperBound(double upperbound) { _m_CBGupperBound = upperbound; }
};


#endif /* !_BGIP_SOLVERBRUTEFORCESEARCH_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
