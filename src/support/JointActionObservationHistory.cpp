/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JointActionObservationHistory.h"
#include "JointAction.h"
#include "JointObservation.h"
#include "PlanningUnitMADPDiscrete.h"
#include "JointActionObservationHistoryTree.h"
#include "ActionObservationHistoryTree.h"
#include "IndexTools.h"

using namespace std;

#define DEBUG_JAOH 0

//Default constructor
JointActionObservationHistory::JointActionObservationHistory(PlanningUnitMADPDiscrete& pu) :
    _m_planningUnitMADPDiscrete(&pu)
{    
    SetLength(0);
    _m_pred = 0;
    _m_jaI = 0; //HARDCODED
    _m_joI = 0; 
    Index individualInitialAOHIndex = 0;//HARDCODED 
    _m_individualActionObservationHistories = vector<Index>(
            _m_planningUnitMADPDiscrete->GetNrAgents(), individualInitialAOHIndex);

}
JointActionObservationHistory::JointActionObservationHistory(Index jaI, Index 
        joI, JointActionObservationHistory* pred) :
    _m_planningUnitMADPDiscrete(pred->_m_planningUnitMADPDiscrete)
{
    SetLength(pred->GetLength() + 1);
    _m_pred = pred;
    
    _m_jaI = jaI;
    _m_joI = joI; 

    //calculate the new individual ActionObservationHistory indices...
    vector<Index> iA = _m_planningUnitMADPDiscrete->JointToIndividualActionIndices(jaI);
    vector<Index> iO = _m_planningUnitMADPDiscrete->JointToIndividualObservationIndices(joI);
    vector<Index> pred_iAOH= pred->GetIndividualActionObservationHistoryIndices();
    for(Index agentI = 0; agentI < _m_planningUnitMADPDiscrete->GetNrAgents(); agentI++)
    {
        Index next_indivIndex = _m_planningUnitMADPDiscrete->GetSuccessorAOHI(agentI, 
                    pred_iAOH[agentI], iA[agentI], iO[agentI] ) ;
        //Index next_indivIndex = 
            //GetReferred() //=PlanningUnitMADPDiscrete
            //->GetActionObservationHistoryTree(agentI, pred_iAOH[agentI])//=aoht
            //->GetSuccessor( iA[agentI], iO[agentI] ) //=aoht'
            //->GetIndex(); //=aohI'
        _m_individualActionObservationHistories.push_back(next_indivIndex);
    }
            
}

string JointActionObservationHistory::SoftPrint() const
{
    stringstream ss;
    if(_m_pred != 0)
    {
        if(_m_length >= 1)
        {
            ss << _m_pred->SoftPrint();
            ss << ", ";
        }
        else
        {
            ss << "JointActionObservationHistory:Print() -Warning:_m_pred != "
                << "null, but lenght <= 1 !"<<endl;
            throw E(ss);
        }
    }

    if (_m_length >= 1) 
    {
        ss << "< ";    
        ss << _m_planningUnitMADPDiscrete->GetJointAction(_m_jaI)->SoftPrintBrief();
        ss << ", ";    
        ss << _m_planningUnitMADPDiscrete->GetJointObservation(_m_joI)->SoftPrintBrief();
        ss << " >";    
    }
    else
        ss << "<EMPTY>";


    return(ss.str());

}

string JointActionObservationHistory::SoftPrintJointIndices() const
{
    stringstream ss;
    if(_m_pred != 0)
    {
        if(_m_length >= 1)
        {
            ss << _m_pred->SoftPrintJointIndices();
            ss << ", ";
        }
        else
        {
            ss << "JointActionObservationHistory:Print() -Warning:_m_pred != "
                << "null, but lenght <= 1 !"<<endl;
            throw E(ss);
        }
    }

    if (_m_length >= 1) 
    {
        ss << "jaoI=" <<
            IndexTools::ActionAndObservation_to_ActionObservationIndex(
            _m_jaI, _m_joI, 
            _m_planningUnitMADPDiscrete->GetNrJointActions(), 
            _m_planningUnitMADPDiscrete->GetNrJointObservations() );
        ss << "(jaI=";    
        ss << _m_jaI;
        ss << ",joI=";    
        ss << _m_joI;
        ss << ")";    
    }
    else
        ss << "<>";


    return(ss.str());

}

void 
JointActionObservationHistory::GetJointActionObservationHistoryVectors(
//    JointActionObservationHistory *joah,
    vector<Index> &jaIs, vector<Index> &joIs)
{
    vector<Index> jaIsReversed;
    vector<Index> joIsReversed;

    GetJointActionObservationHistoryVectorsRecursive( jaIsReversed,
            joIsReversed);

    // reverse both vectors
    for(vector<Index>::reverse_iterator it=jaIsReversed.rbegin();
        it!=jaIsReversed.rend(); ++it)
        jaIs.push_back(*it);

    for(vector<Index>::reverse_iterator it=joIsReversed.rbegin();
        it!=joIsReversed.rend(); ++it)
        joIs.push_back(*it);

#if DEBUG_JAOH
    cout << SoftPrint() << " " << SoftPrintVector(jaIsReversed) << " "
         << SoftPrintVector(joIsReversed) << endl;
#endif
}

void 
JointActionObservationHistory::GetJointActionObservationHistoryVectorsRecursive(
    vector<Index> &jaIs, vector<Index> &joIs)
{
    if(_m_length >= 1) // for t=0, length=0 (and we don't want to include the
        //_m_jaI, _m_joI for that stage (they are def'd to be 0) )
    {
        jaIs.push_back(_m_jaI);
        joIs.push_back(_m_joI);
    }
    if(_m_pred!=NULL)
        _m_pred->GetJointActionObservationHistoryVectorsRecursive(jaIs,joIs);
}

/* old - this seems strange...
void 
JointActionObservationHistory::GetJointActionObservationHistoryVectors(
    JointActionObservationHistory *joah,
    vector<Index> &jaIs, vector<Index> &joIs)
{
    vector<Index> jaIsReversed;
    vector<Index> joIsReversed;

    GetJointActionObservationHistoryVectorsRecursive(joah,
                                                     jaIsReversed,
                                                     joIsReversed);

    // reverse both vectors
    for(vector<Index>::reverse_iterator it=jaIsReversed.rbegin();
        it!=jaIsReversed.rend(); ++it)
        jaIs.push_back(*it);

    for(vector<Index>::reverse_iterator it=joIsReversed.rbegin();
        it!=joIsReversed.rend(); ++it)
        joIs.push_back(*it);

}

void 
JointActionObservationHistory::GetJointActionObservationHistoryVectorsRecursive(
    JointActionObservationHistory *joah,
    vector<Index> &jaIs, vector<Index> &joIs)
{
    if(joah->_m_length > 1)
    {
        jaIs.push_back(joah->_m_jaI);
        joIs.push_back(joah->_m_joI);
    }
    if(joah->_m_pred!=NULL)
        GetJointActionObservationHistoryVectorsRecursive(joah->_m_pred,
                                                         jaIs,joIs);
}

*/
