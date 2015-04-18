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
#ifndef _DICEPSPlannerPLANNER_H_
#define _DICEPSPlannerPLANNER_H_ 1

/* the include directives */
#include <iostream>
#include <vector>
#include <list>
#include <fstream>

#include "Globals.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "ValueFunctionDecPOMDPDiscrete.h"
#include "JointPolicyPureVector.h"
#include "JointPolicy.h"
#include "TimedAlgorithm.h"


class JPPVValuePair;

using std::vector;
using std::list;

/** \brief DICEPSPlanner implements the Direct Cross-Entropy Policy
 * Search method.
 *
 * The algorithm is described in #refDICEPS (see DOC-References.h).
 */
class DICEPSPlanner : 
    public PlanningUnitDecPOMDPDiscrete,
    public TimedAlgorithm
{
    
private:    
    //output settings:
    bool _m_outputConvergenceStatistics;
    std::ofstream* _m_outputConvergenceFile;
    int _m_verbose;

    // CE Settings
    size_t _m_nrRestarts;
    size_t _m_nrIterations;
    size_t _m_nrSampledJointPolicies;
    size_t _m_nrJointPoliciesForUpdate;
    bool _m_use_gamma;
    double _m_alpha;
    size_t _m_nrEvalRuns;

    //the best found policy
    JPPV_sharedPtr _m_foundPolicy;
    //the expected reward of the best found policy
    double _m_expectedRewardFoundPolicy;

protected:

    static void SampleIndividualPolicy(PolicyPureVector& pol, 
            const vector< vector<double> >&  ohistActionProbs );
    static void OrderedInsertJPPVValuePair( JPPVValuePair* pv, 
            list< JPPVValuePair*>& l );
    static void PrintBestSamples( const list< JPPVValuePair*>& l );
    
    void UpdateCEProbDistribution(
            vector< vector< vector<double> > >& Xi, 
            const list<JPPVValuePair* >& best_samples);
    double ApproximateEvaluate(JointPolicyDiscrete &jpol, int nrRuns);

    public:
        
        // Constructor, destructor and copy assignment.
        // (default) Constructor
        //DICEPSPlanner();
        DICEPSPlanner(
            const PlanningUnitMADPDiscreteParameters &params,
            DecPOMDPDiscreteInterface* p,
            size_t horizon,
            size_t nrRestarts,
            size_t nrIterations,
            size_t nrSamples,
            size_t nrSamplesForUpdate,
            bool use_hard_threshold, //(gamma in CE papers)
            double CEalpha, //the learning rate
            size_t nrEvalRuns, // policy evaluation runs (set 0 for exact eval)
            int verbose = 0
            );
        DICEPSPlanner(
            DecPOMDPDiscreteInterface* p,
            int horizon,
            size_t nrRestarts,
            size_t nrIterations,
            size_t nrSamples,
            size_t nrSamplesForUpdate,
            bool use_hard_threshold, //(gamma in CE papers)
            double CEalpha, //the learning rate
            size_t nrEvalRuns, // policy evaluation runs (set 0 for exact eval)
            int verbose = 0
            );
        DICEPSPlanner(
            const PlanningUnitMADPDiscreteParameters &params,
            DecPOMDPDiscreteInterface* p,
            size_t horizon,
            size_t nrRestarts,
            size_t nrIterations,
            size_t nrSamples,
            size_t nrSamplesForUpdate,
            bool use_hard_threshold, //(gamma in CE papers)
            double CEalpha, //the learning rate
            size_t nrEvalRuns, // policy evaluation runs (set 0 for exact eval)
            bool convergenceStats,
            std::ofstream & convergenceStatsFile,
            int verbose = 0
            );
        DICEPSPlanner(
            DecPOMDPDiscreteInterface* p,
            int horizon,
            size_t nrRestarts,
            size_t nrIterations,
            size_t nrSamples,
            size_t nrSamplesForUpdate,
            bool use_hard_threshold, //(gamma in CE papers)
            double CEalpha, //the learning rate
            size_t nrEvalRuns, // policy evaluation runs (set 0 for exact eval)
            bool convergenceStats,
            std::ofstream & convergenceStatsFile,
            int verbose = 0
            );

        //operators:

        //data manipulation (set) functions:
        /**
         * The methods that performs the planning according to 
         * the CE for Dec-POMDP algorithm. */
        void Plan();

        //get (data) functions:
        boost::shared_ptr<JointPolicy> GetJointPolicy()
            { return(_m_foundPolicy); }
        boost::shared_ptr<JointPolicyDiscrete> GetJointPolicyDiscrete()
            { return(_m_foundPolicy); }
        JPPV_sharedPtr GetJointPolicyPureVector()
            { return(_m_foundPolicy); }
        double GetExpectedReward(void) const
            { return(_m_expectedRewardFoundPolicy); }

};


#endif /* !_DICEPSPlannerPLANNER_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
