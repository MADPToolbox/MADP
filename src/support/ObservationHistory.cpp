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

#include "Observation.h"
#include "ObservationHistory.h"
//Necessary as header file contains a forward declaration:
#include "PlanningUnitMADPDiscrete.h" 

using namespace std;

#define DEBUG_OH 0

//Default constructor
ObservationHistory::ObservationHistory(PlanningUnitMADPDiscrete& pu,Index agentI) :
    IndividualHistory(agentI),
    _m_PlanningUnitMADPDiscrete(&pu)
{
    SetLength(0);
    _m_observationI = 0;
    _m_containsEmptyOI = true;
    _m_pred = 0;
}

ObservationHistory::ObservationHistory(PlanningUnitMADPDiscrete& pu, Index agentI, Index obsI) :
    IndividualHistory(agentI)
{
    throw E("ObservationHistory::ObservationHistory(Index obsI) not yet implemented - non-empty initial observations not yet supported.");
}

ObservationHistory::ObservationHistory(Index obsI, ObservationHistory* pred):
    IndividualHistory(pred->_m_agentI),
    _m_PlanningUnitMADPDiscrete(pred->_m_PlanningUnitMADPDiscrete)
{
    SetLength(pred->GetLength() + 1);
    _m_observationI = obsI;
    _m_containsEmptyOI = false;
    _m_pred = pred;
}

//Destructor
ObservationHistory::~ObservationHistory()
{
if(DEBUG_OH){    cout << "Deleting observation history: ";
    Print();  cout << endl;}
}

string ObservationHistory::SoftPrint() const
{
    stringstream ss;
    if(_m_pred != 0)
    {
        if(_m_length >= 1)
            ss << _m_pred->SoftPrint();
        else
            throw E("ObservationHistory:SoftPrint() - Warning:_m_pred != null,\
                    but lenght <= 1 !");
    }   
    if (!_m_containsEmptyOI) // don't print the empty observation
    {
//        ss << GetReferred()->GetObservationDiscrete(_m_agentI,
//        let's see if this works...
        ss << _m_PlanningUnitMADPDiscrete->GetObservation(_m_agentI,
            _m_observationI)->SoftPrintBrief() << ", ";
    }
    else
        ss << "Oempty, ";

    return(ss.str());
}
