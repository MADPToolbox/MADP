/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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

#include "ActionObservationHistory.h"
#include "Action.h"
#include "Observation.h"
#include "PlanningUnitMADPDiscrete.h"
#include "ActionHistory.h"
#include "ObservationHistoryTree.h"

using namespace std;

#define DEBUG_AOH 0

//Default constructor
ActionObservationHistory::ActionObservationHistory(PlanningUnitMADPDiscrete& pu, Index agentI) :
    IndividualHistory(agentI),
    _m_planningUnitMADPDiscrete(&pu)
{    
    SetLength(0);
    _m_pred = 0;
    _m_ahI = 0; //HARDCODED
    _m_ohI = 0; //HARDCODED
}
ActionObservationHistory::ActionObservationHistory(Index aI, Index oI,ActionObservationHistory* pred) :
    IndividualHistory(pred->_m_agentI),
    _m_planningUnitMADPDiscrete(pred->_m_planningUnitMADPDiscrete)
{
    SetLength(pred->GetLength() + 1);
    _m_pred = pred;
    _m_ahI = 
        CastLIndexToIndex(_m_planningUnitMADPDiscrete->GetActionHistoryTree(_m_agentI, pred->_m_ahI) //=aht
                          ->GetSuccessor(aI) //=aht'
                          ->GetIndex()); //=ahI'
    _m_ohI = 
       CastLIndexToIndex(_m_planningUnitMADPDiscrete->GetObservationHistoryTree(_m_agentI, pred->_m_ohI) //=oht
                         ->GetSuccessor(oI) //=oht'
                         ->GetIndex()); //=ohI'
}

string ActionObservationHistory::SoftPrint() const
{
    stringstream ss;
    if(_m_pred != 0)
    {
        if(_m_length >= 1)
            ss << _m_pred->SoftPrint();
        else
        {
            ss << "ObservationHistory:Print() - Warning:_m_pred != "<<
            "null, but lenght < 1 !"<<endl;
            throw E(ss);
        }
    }

    ActionHistory* ah = _m_planningUnitMADPDiscrete->GetActionHistoryTree(
            _m_agentI, _m_ahI) //=aht
        ->GetContainedElement(); //=ah
    if (!ah->IsEmpty()) // don't print the empty observation
    {
        ss << ", ";
        Index aI = ah->GetLastActionIndex(); // aI
        ss << _m_planningUnitMADPDiscrete->GetAction(_m_agentI, aI)->SoftPrintBrief();
    }
    else
        ss << "EMPTY_AH";

    ss << ", ";

    ObservationHistory* oh = _m_planningUnitMADPDiscrete->GetObservationHistoryTree(
            _m_agentI, _m_ohI) //=oht
        ->GetContainedElement(); //=oh
    if (!oh->ContainsEmptyOI()) // don't print the empty observation
    {
        ss << ", ";
        Index oI = oh->GetLastObservationIndex(); // oI
        ss << _m_planningUnitMADPDiscrete->GetObservation(_m_agentI, oI)->SoftPrintBrief();
    }
    else
        ss << "Oempty";

    return(ss.str());
}
