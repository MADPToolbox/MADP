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
#ifndef _BAYESIANGAMEIDENTICALPAYOFFSOLVER_H_
#define _BAYESIANGAMEIDENTICALPAYOFFSOLVER_H_ 1

/* the include directives */
#include <iostream>
#include <fstream>
#include "Globals.h"
#include <sys/times.h>
#include "boost/shared_ptr.hpp"
#include "boost/pointer_cast.hpp"
//needed for timing results
#include <sys/time.h>
#include <time.h>

#include "BGIPSolution.h"
class BayesianGameIdenticalPayoffInterface;

/**\brief BayesianGameIdenticalPayoffSolver is an interface for
 * solvers for Bayesian games with identical payoff.
 *
 * This is the non-templated version, hence the two versions of Evaluate().
 */

class BayesianGameIdenticalPayoffSolver
{
private:    
    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> _m_referredBG;
    
    ///Stores the solution found by the solver
    BGIPSolution _m_solution;
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
    virtual void CheckDeadline(const std::string &errorMessage) const;

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor 
    /**takes a reference to the BG to be solved. nrDesiredSolutions is the number
     * of solutions that the solver should return. I.e., if set higher than
     * 1, it returns the k best found joint policies.
     */
    BayesianGameIdenticalPayoffSolver(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg,
            size_t nrDesiredSolutions=1);
    /// Destructor.
    virtual ~BayesianGameIdenticalPayoffSolver(){};
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

    boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> GetBGIPI() const
    {return(_m_referredBG);}

    ///Turns Anytime results on and of 
    /**When turning on, valid ofstream pointers must be provided for the 
     * results and timings file.
     */
    void SetAnyTimeResults(bool turn_on, std::ofstream* results, 
            std::ofstream* timings);
    
    double Evaluate(const JointPolicyPureVector & jpolBG) const;
    double Evaluate(const JointPolicyPureVectorForClusteredBG & jpolBG) const;

    /// returns a new policy to be used with this BayesianGameIdenticalPayoffSolver
    virtual boost::shared_ptr<JointPolicyDiscretePure> GetNewJpol() const;
    
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

};


#endif /* !_BAYESIANGAMEIDENTICALPAYOFFSOLVER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
