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
#ifndef _BGIPSOLUTION_H_
#define _BGIPSOLUTION_H_ 1

/* the include directives */
#include <iostream>
#include <set>
#include <queue>
#include "boost/shared_ptr.hpp"
#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "JointPolicyPureVectorForClusteredBG.h"
#include "FixedCapacityPriorityQueue.h"

class JPPVValuePair;
class PartialJPDPValuePair;

/**\brief BGIPSolution represents a solution for BayesianGameIdenticalPayoff.*/
class BGIPSolution 
{
private:    

    const Interface_ProblemToPolicyDiscretePure* _m_pu;
    I_PtPDpure_constPtr _m_puShared;

    ///This variable gives the number of solutions to return (k).
    size_t _m_nrDesiredSolutions;

    ///whether we use the fixed capacity priority queue
    bool _m_useFixedCapacityPriorityQueue;

    /**if we want to return the best k solutions, we store them in a priority 
     *queue.
     */
    FixedCapacityPriorityQueue<boost::shared_ptr<JPPVValuePair> > _m_qFixedK;
    FixedCapacityPriorityQueue<boost::shared_ptr<PartialJPDPValuePair> > _m_qpFixedK;

    /* These are the queues use when there is no limit on how many
     * solutions we store. 
     *
     * They have to be pointers otherwise there
     * are problems (classes are not properly defined) with not including JPPVValuePair.h and
     * PartialJPDPValuePair.h
     * */
    std::priority_queue<boost::shared_ptr<JPPVValuePair> > *_m_qInfSize;
    std::priority_queue<boost::shared_ptr<PartialJPDPValuePair> > *_m_qpInfSize;
    
    //The set of Indices of the policies added to the queue
    //(to avoid adding duplicates).
    std::set<LIndex> _m_jpolIndices;

    bool _m_issuedOverFlowWarning;

protected:

    void GiveOverFlowWarning() 
    {
        if (_m_issuedOverFlowWarning)
            return;
        std::cerr << "BGIPSolution: Warning, joint policy indices are"
             << " overflowing, cannot detect duplicates" << std::endl;
        _m_issuedOverFlowWarning = true;
    }
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIPSolution(const Interface_ProblemToPolicyDiscretePure* pu,
                 size_t nrDesiredSolutions=1);
    BGIPSolution(const I_PtPDpure_constPtr &pu,
                 size_t nrDesiredSolutions=1);
    /// Destructor.
    ~BGIPSolution();

    double GetPayoff() const;

    const boost::shared_ptr<JointPolicy> GetJointPolicy() const;
    const JointPolicyPureVector& GetJointPolicyPureVector() const;
    const JointPolicyPureVectorForClusteredBG&
    GetJointPolicyPureVectorForClusteredBG() const;

    void Save(const std::string &filename) const;
    void Load(const std::string &filename);

    void Print() const { std::cout << SoftPrint() << std::endl; }
    std::string SoftPrint() const;

    size_t GetNrDesiredSolutions() const {return _m_nrDesiredSolutions; }
    void SetNrDesiredSolutions(size_t n);

    size_t GetNrFoundSolutions() const;

    /**\brief Adds a JPPVValuePair to the priority queue that maintains the best 
     * _m_nrSolutions solutions.
     *
     * Any old solutions that are no longer needed are automatically deleted.
     */
    void AddSolution(const JointPolicyPureVector &jp, double value );
    void AddSolution(const JointPolicyPureVectorForClusteredBG &jp, double value);
    void AddSolution(LIndex jpolIndex, double value);

    boost::shared_ptr<JPPVValuePair> GetNextSolutionJPPV() const;
    void PopNextSolutionJPPV();
    bool IsEmptyJPPV() const;

    boost::shared_ptr<PartialJPDPValuePair> GetNextSolutionPJPDP() const;
    void PopNextSolutionPJPDP();
    bool IsEmptyPJPDP() const;
};


#endif /* !_BGIPSOLUTION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
