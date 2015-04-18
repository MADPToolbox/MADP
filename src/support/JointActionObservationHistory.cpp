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
     Referrer<PlanningUnitMADPDiscrete>(pu)
{    
    SetLength(0);
    _m_pred = 0;
    _m_jaI = 0; //HARDCODED
    _m_joI = 0; 
    Index individualInitialAOHIndex = 0;//HARDCODED 
    _m_individualActionObservationHistories = vector<Index>(
            GetReferred()->GetNrAgents(), individualInitialAOHIndex);

}
JointActionObservationHistory::JointActionObservationHistory(Index jaI, Index 
        joI, JointActionObservationHistory* pred) :
    Referrer<PlanningUnitMADPDiscrete>(pred->GetReferred())
{
    SetLength(pred->GetLength() + 1);
    _m_pred = pred;
    
    _m_jaI = jaI;
    _m_joI = joI; 

    //calculate the new individual ActionObservationHistory indices...
    vector<Index> iA = GetReferred()->JointToIndividualActionIndices(jaI);
    vector<Index> iO = GetReferred()->JointToIndividualObservationIndices(joI);
    vector<Index> pred_iAOH= pred->GetIndividualActionObservationHistoryIndices();
    for(Index agentI = 0; agentI < GetReferred()->GetNrAgents(); agentI++)
    {
        Index next_indivIndex = 
            GetReferred() //=PlanningUnitMADPDiscrete
            -> GetSuccessorAOHI(agentI, 
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
        ss << GetReferred()->GetJointAction(_m_jaI)->SoftPrintBrief();
        ss << ", ";    
        ss << GetReferred()->GetJointObservation(_m_joI)->SoftPrintBrief();
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
            GetReferred()->GetNrJointActions(), 
            GetReferred()->GetNrJointObservations() );
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
