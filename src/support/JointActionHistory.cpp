/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JointActionHistory.h"
#include "JointAction.h"
//Necessary as header file contains a forward declaration:
#include "PlanningUnitMADPDiscrete.h" 

using namespace std;

#define DEBUG_JOH 0

//Default constructor
JointActionHistory::JointActionHistory(PlanningUnitMADPDiscrete& pu) :
     Referrer<PlanningUnitMADPDiscrete>(pu)
{
    SetLength(0);
    _m_jointActionI = 0;
    _m_isEmpty = true;
    _m_pred = 0;
    size_t nrAgents = pu.GetNrAgents();
    _m_individualActionHistories = vector<Index>(nrAgents,0);
}
JointActionHistory::JointActionHistory(PlanningUnitMADPDiscrete& pu,
        Index jObsI)
{
    throw E("JointActionHistory::JointActionHistory(Index obsI) not yet implemented - non-empty initial actions not yet supported.");
}

JointActionHistory::JointActionHistory(Index jaI,
           JointActionHistory* pred) :
     Referrer<PlanningUnitMADPDiscrete>( pred->GetReferred() )
{
    SetLength(pred->GetLength() + 1);
    _m_jointActionI = jaI;
    _m_isEmpty = false;
    _m_pred = pred;
    size_t nrAgents = pred->_m_individualActionHistories.size();
    
    vector<Index> indivActionIndices = GetReferred()->
        JointToIndividualActionIndices(jaI);

    for(Index aI=0; aI < nrAgents; aI++)
    {
        //get previous history index for agent ai 
        Index predOHindex_ai = pred->_m_individualActionHistories[aI];
        //get the corresponding previous history
        //ActionHistoryTree* oht_ai = GetReferred()->
            //GetActionHistoryTree(aI, predOHindex_ai);
        //get the successor for the action taken by ai
        //Index sucOHindex_ai = oht_ai->GetSuccessor(indivActionIndices[aI])->
            //GetIndex();
        //and store it...
        Index sucOHindex_ai = 
            GetReferred() //=PlanningUnitMADPDiscrete
            -> GetSuccessorAHI(aI, 
                    predOHindex_ai, indivActionIndices[aI] ) ;

        _m_individualActionHistories.push_back(sucOHindex_ai);
    }
    
}

//Copy assignment constructor.    
JointActionHistory::JointActionHistory(const JointActionHistory& o) 
{
if(DEBUG_JOH){ cout << "Cloning JointActionHistory: ";
    Print(); cout <<endl;}
}
//Destructor
JointActionHistory::~JointActionHistory()
{
if(DEBUG_JOH){ cout << "Deleting JointActionHistory: ";
    Print(); cout <<endl;}
    
//    _m_individualActionHistories.clear();
}

const vector<Index>& JointActionHistory::GetIndividualActionHistoryIndices() const
{
    return(_m_individualActionHistories);
}

string JointActionHistory::SoftPrint() const
{
    stringstream ss;
    if(_m_pred != 0)
    {
        //if the predec. is the empty action sequence (with length 0) 
        //we don't want to print this. (and this is the case when 
        //this action history has length 1)
        if(_m_length >= 1) // action histories of length 1 have a pred.
            ss << _m_pred->SoftPrint();
        else if (_m_length < 1)
        {
            ss << "JointActionHistory:Print() - Warning:_m_pred != "<<
            "null, but lenght < 1 !"<<endl;
            throw E(ss);
        }
        //else length == 1 and we simply skip
    }
        
    if (!_m_isEmpty) // don't print the empty action
    {
//        GetReferred()->GetJointActionDiscrete(
        ss << GetReferred()->GetJointAction(
            _m_jointActionI)->SoftPrintBrief();
    }
    else
        ss << "EMPTY_JAH";
    return(ss.str());
}
