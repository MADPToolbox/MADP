/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ActionHistory.h"
#include "Action.h"
//Necessary as header file contains a forward declaration:
#include "PlanningUnitMADPDiscrete.h" 

using namespace std;

#define DEBUG_AH 0

//Default constructor
ActionHistory::ActionHistory(PlanningUnitMADPDiscrete& pu, 
        Index agentI) :
    Referrer<PlanningUnitMADPDiscrete>(pu),
    IndividualHistory(agentI)
{
    SetLength(0);
    _m_actionI = 0;
    _m_isEmpty = true;
    _m_pred = 0;
}

ActionHistory::ActionHistory(Index aI, ActionHistory* pred):
    Referrer<PlanningUnitMADPDiscrete>( pred->GetReferred() ),
    IndividualHistory(pred->_m_agentI)
{
    SetLength(pred->GetLength() + 1);
    _m_actionI = aI;
    _m_isEmpty = false;
    _m_pred = pred;
}

//Destructor
ActionHistory::~ActionHistory()
{
if(DEBUG_AH){    cout << "Deleting action history: ";
    Print();  cout << endl;}
}

string ActionHistory::SoftPrint() const
{
    stringstream ss; 
    if(_m_pred != 0)
    {
        //if the predec. is the empty action sequence (with length 0) 
        //we don't want to print this. (and this is the case when 
        //this action history has length 1)
        if(_m_length >= 1)
            ss << _m_pred->SoftPrint();
        else if (_m_length < 1)
        {
            ss << "ActionHistory:Print() - Warning:_m_pred != "<<
            "null, but lenght < 1 !"<<endl;
            throw E(ss);
        }
        //else length == 1 and we simply skip
    }
   
    if (!_m_isEmpty) // don't print the empty action
    {
        ss << (GetReferred()->GetAction(_m_agentI,
            _m_actionI))->SoftPrintBrief();
        ss << ", ";
    }
    else
        ss << "EMPTY_AH";

    return(ss.str());

}
