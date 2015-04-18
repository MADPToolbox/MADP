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

#include "JointObservationHistory.h"
#include "JointObservation.h"
//Necessary as header file contains a forward declaration:
#include "PlanningUnitMADPDiscrete.h" 
#include "ObservationHistoryTree.h"

using namespace std;

#define DEBUG_JOH 0

//Default constructor
JointObservationHistory::JointObservationHistory(PlanningUnitMADPDiscrete& pu) :
    Referrer<PlanningUnitMADPDiscrete>(pu)
{
    SetLength(0);
    _m_jointObservationI = 0;
    _m_containsEmptyJOI = true;
    _m_pred = 0;
    size_t nrAgents = pu.GetNrAgents();
    _m_individualObservationHistories = vector<Index>(nrAgents,0);
}


/*Creates a observation history specifying jObsI for the last joint
  * observation and pred as the preceeding JointObservationHistory.*/
JointObservationHistory::JointObservationHistory(Index joI,
           JointObservationHistory* pred) :
     Referrer<PlanningUnitMADPDiscrete>( pred->GetReferred() )
{
    SetLength(pred->GetLength() + 1);
    _m_jointObservationI = joI;
    _m_containsEmptyJOI = false;
    _m_pred = pred;
    size_t nrAgents = pred->_m_individualObservationHistories.size();
    
    const vector<Index>& indivOIndices = GetReferred()->
        JointToIndividualObservationIndices(joI);

    //we also maintain the indices of the individual observation histories
    //represented by this joint observation history
    for(Index aI=0; aI < nrAgents; aI++)
    {
        //get the individual observation history for the predecessor
        Index predOHindex_ai = pred->_m_individualObservationHistories[aI];
        ObservationHistoryTree* oht_ai = GetReferred()->
            GetObservationHistoryTree(aI, predOHindex_ai);
        Index ai_last_observation = indivOIndices[aI];
        Index sucOHindex_ai = 
            CastLIndexToIndex(oht_ai->GetSuccessor(ai_last_observation)->
                              GetIndex());
        _m_individualObservationHistories.push_back(sucOHindex_ai);
    }
}

//Copy assignment constructor.    
JointObservationHistory::JointObservationHistory(const JointObservationHistory& o) 
{
if(DEBUG_JOH){ cout << "Cloning JointObservationHistory: ";
    Print(); cout <<endl;}
}
//Destructor
JointObservationHistory::~JointObservationHistory()
{
if(DEBUG_JOH){ cout << "Deleting JointObservationHistory: ";
    Print(); cout <<endl;}
    
    _m_individualObservationHistories.clear();
}
const vector<Index>& JointObservationHistory::GetIndividualObservationHistoryIndices() const
{
    return(_m_individualObservationHistories);
}

string JointObservationHistory::SoftPrint() const
{
    stringstream ss;
    if(_m_pred != 0)
    {
        if(_m_length >= 1)
            ss << _m_pred->SoftPrint();
        else
        {
            ss << "JointObservationHistory:Print() - Warning:_m_pred != "<<
            "null, but lenght <= 1 !"<<endl;
            throw E(ss);
        }
    }
   
    if (!_m_containsEmptyJOI) // don't print the empty observation
    {
        ss << GetReferred()->GetJointObservation(
            _m_jointObservationI)->SoftPrintBrief() << ", ";
    }
    else
        ss << "JOnull, ";

    return(ss.str());
}
