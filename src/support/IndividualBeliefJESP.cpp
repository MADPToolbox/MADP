/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "IndividualBeliefJESP.h"

#include <float.h>
#include <string>
//Necessary as header file contains a forward declaration:
#include "PlanningUnitMADPDiscrete.h" 
#include "JointPolicyPureVector.h" 

using namespace std;

#define IndividualBeliefJESP_doSanityCheckAfterEveryUpdate 0

IndividualBeliefJESP::IndividualBeliefJESP(Index agentI, Index stage,
    const PlanningUnitMADPDiscrete& pu) 
    :
    Belief(0),
    _m_pumadp(&pu),
    _m_stage(stage),
    _m_agentI(agentI)
{
    //compute size of  this belief:
    size_t nrS = _m_pumadp->GetNrStates();
    _m_nrAgents= _m_pumadp->GetNrAgents();
    size_t nrJOH_others = 1;
    for(Index j=0; j < _m_nrAgents; j++)
    {
        if(j == agentI)
            continue;
        //else:
        _m_others.push_back(j);
        size_t nrOH_j = _m_pumadp->GetNrObservationHistories(j, stage);
        nrJOH_others *=  nrOH_j;
        _m_nrOH_others.push_back(nrOH_j);
    }
    size_t size = nrS * nrJOH_others;
    _m_b = vector<double>(size,0.0);
    _m_sizeVec.push_back(nrS);
    _m_sizeVec.push_back(nrJOH_others);
    _m_stepsizeSJOH = IndexTools::CalculateStepSize(_m_sizeVec);

    //we will use 
    // IndexTools::JointToIndividualIndicesStepSize(Index jointI, 
    //      const vector<size_t> &step_size, size_t vec_size )
    //and
    // IndexTools::IndividualToJointIndicesStepSize (const std::vector< Index >
    //      &indices, const std::vector< size_t > &step_size)
    //
    // to do the conversions between JOHI_j <-> <OHI_1,...,OHI_nrA >
    //
    // so we cah the step_size
    _m_stepsizeJOHOH = IndexTools::CalculateStepSize(_m_nrOH_others);
}

//Destructor
IndividualBeliefJESP::~IndividualBeliefJESP()
{
    delete [] _m_stepsizeJOHOH;
    delete [] _m_stepsizeSJOH;
}

IndividualBeliefJESP& 
IndividualBeliefJESP::operator= (const IndividualBeliefJESP& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment

    throw E("IndividualBeliefJESP assignment operator not implemented yet");
    Belief::operator=(o);

    return(*this);
}

vector<Index>
IndividualBeliefJESP::GetOthersObservationHistIndex(Index eI) const
{
    //get <sI, JOHI_others>
    vector<Index> v1 = 
        IndexTools::JointToIndividualIndicesStepSize(eI, _m_stepsizeSJOH, 2);
    //get <individual observation history indices of others:
    vector<Index> withinStageOHIndices_o = 
        IndexTools::JointToIndividualIndicesStepSize(
            v1.at(1), _m_stepsizeJOHOH, _m_nrAgents-1 );
    //add the offset for the stage to the indices:
    for(Index j=0; j < withinStageOHIndices_o.size(); j++)
        withinStageOHIndices_o[j] += CastLIndexToIndex(_m_pumadp->
                                                       GetFirstObservationHistoryIndex( _m_others[j], _m_stage)); 
    return(withinStageOHIndices_o);
}
Index 
IndividualBeliefJESP::GetAugmentedStateIndex(Index sI, 
        const vector<Index>& oHist_others) const
{
    //get indices without the offset for the stage:
    vector<Index> withinStageOHIndices_o = vector<Index>(oHist_others.size());
    for(Index j=0; j < withinStageOHIndices_o.size(); j++)
        withinStageOHIndices_o[j] = oHist_others[j] -
            CastLIndexToIndex(_m_pumadp->GetFirstObservationHistoryIndex( _m_others[j], _m_stage)); 

    vector<Index> v;
    v.push_back(sI);
    v.push_back(
        IndexTools::IndividualToJointIndicesStepSize(withinStageOHIndices_o, 
            _m_stepsizeJOHOH ) );
    return( IndexTools::IndividualToJointIndicesStepSize( v, 
            _m_stepsizeSJOH) );
}

double IndividualBeliefJESP::Update(
        const IndividualBeliefJESP& b_prev, 
           Index lastAI, Index newOI, const JointPolicyPureVector* jpol)
{
    //set all probs of this belief to 0
    _m_b = vector<double>(_m_b.size(), 0.0);

    double Po_ba = 0.0; // P(o|b,a) with o=newJO
    vector<double> newJB_unnorm;

    size_t nrJO_others = 1;
    Scope otherAgentIndices;
    vector<size_t> nrO_others; 
    for(Index j=0; j < _m_pumadp->GetNrAgents(); j++)
    {
        if(j == _m_agentI)
            continue;
        //else
        Index nrO_j =  _m_pumadp->GetNrObservations(j);
        nrJO_others *= nrO_j;
        nrO_others.push_back(nrO_j);
        otherAgentIndices.push_back(j);
    }

    for(Index prev_eI=0; prev_eI < b_prev.Size(); prev_eI++)
    {
        double probPrev_eI=b_prev.Get(prev_eI);
        if(probPrev_eI>0)// only need to do this if this previous eI is possible
        {
            Index prev_sI = b_prev.GetStateIndex(prev_eI);
            vector<Index> prev_oHist_others = b_prev.
                GetOthersObservationHistIndex(prev_eI);
            vector<Index> actions(_m_nrAgents);
            actions.at(_m_agentI) = lastAI;
            for(Index j=0; j < otherAgentIndices.size(); j++)
            {
                Index ag_j = otherAgentIndices[j];
                Index prev_oHist_j = prev_oHist_others[j];//not ag_j!!!
                Index act_j = jpol->GetActionIndex(ag_j, prev_oHist_j);
                actions.at(ag_j) = act_j;
            }
            Index jaI = _m_pumadp->IndividualToJointActionIndices(actions);
            
            for(Index next_sI=0; next_sI < _m_pumadp->GetNrStates(); next_sI++)
            {
                //note we do not loop over all possible next_eI, because *a lot*
                //of transitions will be 0 ( if next_oHistJ != (prev_oHistJ, oJ) )
                //
                //rather we now loop over all possible oJ (observations of others)
                double Ps_as = _m_pumadp->
                    GetTransitionProbability(prev_sI, jaI, next_sI);
                
                if(Ps_as>0) // only need to do this if this transition can occur
                {
                    for(Index JO_o=0; JO_o < nrJO_others; JO_o++)
                    {
                        vector<Index> oIs(_m_nrAgents);
                        oIs.at(_m_agentI) = newOI; // `our' observation is fixed
                        vector<Index> oIs_others = IndexTools::JointToIndividualIndices(
                            JO_o, nrO_others);
                        for(Index j=0; j < otherAgentIndices.size(); j++)
                            oIs.at( otherAgentIndices.at(j) ) = oIs_others.at(j);
                        
                        Index joI = _m_pumadp->IndividualToJointObservationIndices(oIs);
                        //compute P(joI | jaI,s')
                        double Po_as = _m_pumadp->GetObservationProbability(
                            jaI, next_sI, joI);
                        
                        //prob of next_eI = <next_sI, (oHist_others, oIs_others)>
                        //AND prev_sI can now be computed.
                        //
                        //first, however, lets find the index, next_eI, for 
                        // <next_sI, (oHist_others, oIs_others)>
                    
                        // first find the next_oHist_others indices.
                        vector<Index> next_oHist_others;
                        for(Index j=0; j < otherAgentIndices.size(); j++)
                        {
                            Index next_oHist_j = _m_pumadp->GetSuccessorOHI(otherAgentIndices.at(j), 
                                                                            prev_oHist_others[j],
                                                                            oIs_others[j] );
                            next_oHist_others.push_back(next_oHist_j);
                        }
                        Index next_eI = GetAugmentedStateIndex(next_sI, 
                                                               next_oHist_others);

                        //p += P(oi | ai, <s',oH'>) * P(<s',oH'>|<s,oH>,ai) * b(<s,oH>)
                        //      = P(oi, <s',oH'>|<s,oH'>,ai) * b(<s,oH>)
                        //      = P(s', jo | s, ja) b(<s,oH>)  //ja=<ai, aj>,aj=pol(oHj)
                        //      = P(jo|ja,s')*P(s'|s,ja)
                        double Pso_sa =  Po_as * Ps_as * probPrev_eI;
                        _m_b.at(next_eI) += Pso_sa;
                        Po_ba += Pso_sa; //running sum of P(oi|b,ai)
                    }
                }
            }
        }
    }
    for(Index eI=0; eI < this->Size(); eI++)
        _m_b.at(eI) = _m_b.at(eI) / Po_ba;

    return(Po_ba);

}
string IndividualBeliefJESP::SoftPrint() const
{
    stringstream ss;
    for(Index eI=0; eI < Size(); eI++)
    {
        ss << "eI="<<eI<<",[sI="<<GetStateIndex(eI) << ", " <<
            SoftPrintVector(GetOthersObservationHistIndex(eI)) << " ] - p=" <<
            Get(eI) << endl;
    }
    return (ss.str());
}
