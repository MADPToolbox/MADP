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
#ifndef _BAYESIANGAMEIDENTICALPAYOFFSOLVER_T_H_
#define _BAYESIANGAMEIDENTICALPAYOFFSOLVER_T_H_ 1

/* the include directives */
#include <iostream>
#include <fstream>
#include "Globals.h"

#include "Referrer.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BayesianGameIdenticalPayoffSolver.h"
#include "JointPolicy.h"
#include "JointPolicyPureVector.h"
#include "BGIPSolution.h"
#include "BayesianGameWithClusterInfo.h"
#include "EDeadline.h"
#include <sys/times.h>
#include "boost/shared_ptr.hpp"
#include "boost/pointer_cast.hpp"

//needed for timing results
#include <sys/time.h>
#include <time.h>

/**\brief BayesianGameIdenticalPayoffSolver_T is an interface for
 * solvers for Bayesian games with identical payoff.
 *
 * The template argument JP represents the joint policy class the
 * solver should return.
 */
template<class JP>
class BayesianGameIdenticalPayoffSolver_T 
    : public BayesianGameIdenticalPayoffSolver
{
public:
    /// (default) Constructor 
    /**takes a reference to the BG to be solved. nrDesiredSolutions is the number
     * of solutions that the solver should return. I.e., if set higher than
     * 1, it returns the k best found joint policies.
     */
    BayesianGameIdenticalPayoffSolver_T(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
            size_t nrDesiredSolutions=1) :
        BayesianGameIdenticalPayoffSolver(bg, nrDesiredSolutions)
    {};

    /// this gives a implementation of GetNewJpol (specified in BayesianGameIdenticalPayoffSolver)
    virtual boost::shared_ptr<JointPolicyDiscretePure> GetNewJpol() const
    //boost::shared_ptr<JP> GetNewJpol() const
    {
        boost::shared_ptr<JP> jpol = boost::shared_ptr<JP>(new JP(this->GetBGIPI()));
        // check whether it is a
        // JointPolicyPureVectorForClusteredBG, because then we
        // need to store more things
        boost::shared_ptr<JointPolicyPureVectorForClusteredBG> JPPVfCBG=
            boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(jpol);
        if(JPPVfCBG)
        {
            boost::shared_ptr<const JointPolicyDiscretePure> pastJpol =
                JPPVfCBG->GetBG()->GetPastJointPolicyPVFCBG();
            JPPVfCBG->SetPrevJPPVfCBG(
                boost::dynamic_pointer_cast<const JointPolicyPureVectorForClusteredBG>(pastJpol));
            JPPVfCBG->SetDepth(JPPVfCBG->GetBG()->GetStage());
        }
        return(jpol);
    }
#if 0 
// Frans 20110923, stuff that is done by BayesianGameIdenticalPayoffSolver     
private:    

    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> _m_referredBG;
    
    ///Stores the solution found by the solver
    BGIPSolution _m_solution;
    ///This variable gives the number of solutions to return (k).

    ///boolean that indicates whether anytime results should be written
    bool _m_writeAnyTimeResults;

    ///the file to which writes the results are written
    std::ofstream* _m_results_f;
    ///the file to which writes the timings of the results are written
    std::ofstream* _m_timings_f;

    /// To limit the amount of time the solver uses.
    double _m_deadlineInSeconds;
    tms _m_timeAtStartOfSolving;

protected:

    /// Should be called at the beginning of Solve().
    virtual void InitDeadline() { times(&_m_timeAtStartOfSolving); }

    /// Checks whether the deadline has expired. Throws EDeadline.
    virtual void CheckDeadline(const std::string &errorMessage) const
        {
            // make sure we don't use too much time
            if(_m_deadlineInSeconds>0)
            {
                tms ts_now;
                times(&ts_now);
                clock_t utime = ts_now.tms_utime -
                    _m_timeAtStartOfSolving.tms_utime;
                clock_t stime = ts_now.tms_stime -
                    _m_timeAtStartOfSolving.tms_stime;
                double timeSpentInS=static_cast<double>(utime+stime) / sysconf(_SC_CLK_TCK);
                if(timeSpentInS>_m_deadlineInSeconds)
                    throw(EDeadline(errorMessage));
            }
        };

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor 
    /**takes a reference to the BG to be solved. nrDesiredSolutions is the number
     * of solutions that the solver should return. I.e., if set higher than
     * 1, it returns the k best found joint policies.
     */
    BayesianGameIdenticalPayoffSolver_T(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
            size_t nrDesiredSolutions=1) :
        BayesianGameIdenticalPayoffSolver(bg, nrDesiredSolutions),
        _m_referredBG(bg),
        _m_solution(bg, nrDesiredSolutions),
        _m_writeAnyTimeResults(false),
        _m_deadlineInSeconds(0)
        {
        };

    /// Destructor.
    virtual ~BayesianGameIdenticalPayoffSolver_T(){};
    
    /**\brief The methods that performs the planning. Returns the
     * expected reward.*/
    virtual double Solve() = 0;
        
    /**\brief Methods should indicated whether they compute exact
    * (optimal) solutions or not. */
    virtual bool IsExactSolver() const = 0;

    /// To limit the amount of time the solver uses.
    virtual void SetDeadline(double deadlineInSeconds) { _m_deadlineInSeconds=deadlineInSeconds; }

    const boost::shared_ptr<JointPolicy> GetJointPolicy() const 
        { return(_m_solution.GetJointPolicy()); }
    const JointPolicyPureVector& GetJointPolicyPureVector() const 
        { return(_m_solution.GetJointPolicyPureVector()); }
    double GetExpectedReward() const 
        { return(_m_solution.GetPayoff()); }
    std::string SoftPrintSolution() const
        { return(_m_solution.SoftPrint()); }
    void SaveSolution(const std::string &filename) const
        { _m_solution.Save(filename); }

    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> GetReferred() const
        {
            return(_m_referredBG);
        }
    ///Turns Anytime results on and of 
    /**When turning on, valid ofstream pointers must be provided for the 
     * results and timings file.
     */
    void SetAnyTimeResults(bool turn_on, std::ofstream* results, 
            std::ofstream* timings)
        {
            _m_writeAnyTimeResults = turn_on;
            if(turn_on)
            {
                _m_results_f = results;
                _m_timings_f = timings;         
            }
            else
            {
                _m_results_f = NULL;
                _m_timings_f = NULL;
            }
        }
    
    double Evaluate(const JP & jpolBG) const
        {
            boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> bgip=GetReferred();
            const BayesianGameIdenticalPayoffInterface *bgipRawPtr=bgip.get();
            double v = 0.0;
            for(Index jt = 0; jt < bgipRawPtr->GetNrJointTypes(); jt++)
            {
                Index jaI = jpolBG.GetJointActionIndex(jt);  
                double p = bgipRawPtr->GetProbability(jt);
                double u = bgipRawPtr->GetUtility(jt, jaI);
                v += p * u;    
            }
            return v;
        }

    virtual boost::shared_ptr<JointPolicyDiscretePure> GetNewJpol() const
    //boost::shared_ptr<JP> GetNewJpol() const
    {
        boost::shared_ptr<JP> jpol = boost::shared_ptr<JP>(new JP(this->GetBGIPI()));
        // check whether it is a
        // JointPolicyPureVectorForClusteredBG, because then we
        // need to store more things
        boost::shared_ptr<JointPolicyPureVectorForClusteredBG> JPPVfCBG=
            boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(jpol);
        if(JPPVfCBG)
        {
            boost::shared_ptr<const JointPolicyDiscretePure> pastJpol =
                JPPVfCBG->GetBG()->GetPastJointPolicyPVFCBG();
            JPPVfCBG->SetPrevJPPVfCBG(
                boost::dynamic_pointer_cast<const JointPolicyPureVectorForClusteredBG>(pastJpol));
            JPPVfCBG->SetDepth(JPPVfCBG->GetBG()->GetStage());
        }
        return(jpol);
    }

    ///Gets the desired number of solutions to be returned
    void SetNrDesiredSolutions(size_t n) 
    {_m_solution.SetNrDesiredSolutions(n);};
    size_t GetNrDesiredSolutions() const 
    {return _m_solution.GetNrDesiredSolutions();};
    ///Gets the found number of solutions
    size_t GetNrFoundSolutions() const 
    {return _m_solution.GetNrFoundSolutions();};

    bool GetWriteAnyTimeResults() const 
        { return(_m_writeAnyTimeResults); }

    std::ofstream* GetResultsOFStream() const
        {  return(_m_results_f); }

    std::ofstream* GetTimingsOFStream() const
        {  return(_m_timings_f); }

    double GetPayoff() const { return(_m_solution.GetPayoff()); }

    void AddSolution(const JointPolicyPureVector &jp, double value )
        { _m_solution.AddSolution(jp,value); }
    void AddSolution(const JointPolicyPureVectorForClusteredBG &jp, double value)
        { _m_solution.AddSolution(jp,value); }
    void AddSolution(LIndex jpolIndex, double value )
        { _m_solution.AddSolution(jpolIndex,value); }

    boost::shared_ptr<JPPVValuePair> GetNextSolutionJPPV() const
        { return(_m_solution.GetNextSolutionJPPV()); }
    void PopNextSolutionJPPV() { _m_solution.PopNextSolutionJPPV(); }
    bool IsEmptyJPPV() const { return(_m_solution.IsEmptyJPPV()); }

    boost::shared_ptr<PartialJPDPValuePair> GetNextSolutionPJPDP() const
        {return(_m_solution.GetNextSolutionPJPDP()); }
    void PopNextSolutionPJPDP() { _m_solution.PopNextSolutionPJPDP(); }
    bool IsEmptyPJPDP() const { return(_m_solution.IsEmptyPJPDP()); }

    virtual void SetCBGupperBound(double ub) = 0;
    virtual void SetCBGlowerBound(double lowerbound) = 0;
#endif

};
#endif /* !_BAYESIANGAMEIDENTICALPAYOFFSOLVER_T_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
