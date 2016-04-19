/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Philipp Robbel 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "FactoredMMDPDiscrete.h"
#include "CPDKroneckerDelta.h"

using namespace std;

string FactoredMMDPDiscrete::SoftPrint() const {
    stringstream ss;
    ss << "Fully-observable MMDP" << endl;
    ss << FactoredDecPOMDPDiscrete::SoftPrint();
    return ss.str();
}

// Don't support flattening of observations by default
void FactoredMMDPDiscrete::CacheFlatModels(bool sparse) {
#if MADP_DFS_WARNINGS
    cout << "FactoredMMDPDiscrete::CacheFlatModels() does not flatten observation model" << endl;
#endif                 
    ConstructJointActions();
  
    CacheFlatTransitionModel(sparse);
    CacheFlatRewardModel(sparse);
}

/// Initialize a fully-observable transition and observation DBN.
void FactoredMMDPDiscrete::Initialize2DBN()
{
    // construct observations
    ConstructObservations();
    SetObservationsInitialized(true); // Note: joint indices are likely to break
    
    MultiAgentDecisionProcessDiscreteFactoredStates::Initialize2DBN(boost::bind( &FactoredMMDPDiscrete::SetScopes, this),
                                                                    boost::bind( &FactoredMMDPDiscrete::ComputeTransitionProb, this, _1,_2,_3,_4,_5),
                                                                    EmptyComputeObservationProb); //fully-observable scenario

    // above calls SetOScopes and initializes CPD vector for observation variables
    Initialize2DBNObservations(); // set actual CPDs
}

void FactoredMMDPDiscrete::ConstructObservations() {
    size_t nrAgents = GetNrAgents();
    size_t nrStateFactors = GetNrStateFactors();
    if(nrAgents == 0 || nrStateFactors == 0)
        throw(E("FactoredMMDPDiscrete::ConstructObservations() no agents specified or state space empty"));

    size_t nrStates = GetNrStates();
    for(Index i=0; i<nrAgents; i++) {
        // create nameless observations for this agent
        SetNrObservations(i, nrStates);
    }
}

void FactoredMMDPDiscrete::SetOScopes() {
    size_t nrAgents = GetNrAgents();
    size_t nrStateFactors = GetNrStateFactors();
    if(nrAgents == 0 || nrStateFactors == 0)
        throw(E("FactoredMMDPDiscrete::SetOScopes() no agents specified or state space empty"));

    const Scope& asfS = GetAllStateFactorScope();
    for(Index oI=0; oI<nrAgents; oI++) {
        SetSoI_O( oI, Scope("<>"), asfS, Scope("<>") );
    }
}

void FactoredMMDPDiscrete::Initialize2DBNObservations() {
    size_t nrAgents = GetNrAgents();
    if(nrAgents == 0)
        throw(E("FactoredMMDPDiscrete::Initialize2DBNObservations() no agents specified"));
    
    for(Index i=0; i<nrAgents; i++) {
        // attach identity function to 2BDN (fully-observable scenario)
        Get2DBN()->SetCPD_O(i, new CPDKroneckerDelta());
    }
}
