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

#include "ActionObservationHistoryTree.h"

#define DEBUG_AOHT 0

using namespace std;

/// Create a joint observation history tree for joh
ActionObservationHistoryTree::ActionObservationHistoryTree(
        ActionObservationHistory * const oh, aoh_t nt) :    
    TreeNode<ActionObservationHistory> (oh)
{
    _m_nodeType = nt; 
}
//Copy constructor.    
ActionObservationHistoryTree::
ActionObservationHistoryTree(const ActionObservationHistoryTree& o) :
    TreeNode<ActionObservationHistory>(o)
{
}

void ActionObservationHistoryTree::SetIndex(Index i)
{
    if (_m_nodeType == O_SUC)
        throw E("Trying ActionObservationHistoryTree::SetIndex on O_SUC node");

    this->TreeNode<ActionObservationHistory>::SetIndex(i);
};

void ActionObservationHistoryTree::SetSuccessor(Index aI, Index oI,
        ActionObservationHistoryTree* suc)
{
    if (_m_nodeType == O_SUC)
        throw E("Trying ActionObservationHistoryTree::SetSuccessor(Index aI, Index oI, ActionObservationHistoryTree* suc)  on O_SUC node");

    //first see if necessary to create a new  intermediate node
    ActionObservationHistoryTree* oNode; 
    if(this->TreeNode<ActionObservationHistory>::ExistsSuccessor(aI))
        oNode = (ActionObservationHistoryTree*)
            this->TreeNode<ActionObservationHistory>::GetSuccessor(aI);
    else //we need to create an intermediate node
    {
        oNode = new ActionObservationHistoryTree(O_SUC);

        //which is the aI'th successor of this node
        this->TreeNode<ActionObservationHistory>::SetSuccessor(aI, oNode);
    }

    //let oNode point to the actual successor.
    //Note: SetSuccessor with 1 index should give no problem for O_SUC nodes
    oNode->TreeNode<ActionObservationHistory>::SetSuccessor(oI, suc);
}

ActionObservationHistoryTree* ActionObservationHistoryTree::GetSuccessor(Index 
        aI, Index oI)
{
    if (_m_nodeType == O_SUC)
        throw E("Trying ActionObservationHistoryTree::SetSuccessor(Index aI, Index oI, ActionObservationHistoryTree* suc)  on O_SUC node");

    ActionObservationHistoryTree* suc;
    try {
        suc = (ActionObservationHistoryTree*)
        //we only put ActionObservationHistoryTree*'s in here so this cast
        //should be allowed.
        this->TreeNode<ActionObservationHistory>::GetSuccessor(aI) //the oNode
        ->TreeNode<ActionObservationHistory>::GetSuccessor(oI);
    } catch(EInvalidIndex& e) {
        // successor does not exist, so we should create it, see
        // JointActionObservationHistoryTree::GetSuccessor for
        // inspiration
        throw(E("ActionObservationHistoryTree::GetSuccessor generation of successors on the fly not yet implemented"));
    }

    return(suc);
}

ActionObservationHistory* ActionObservationHistoryTree::
    GetActionObservationHistory() const
{
    if (_m_nodeType == O_SUC)
        throw E("Trying ActionObservationHistoryTree::GetActionObservationHistory() on O_SUC node");

    return GetContainedElement();
}
    
void ActionObservationHistoryTree::Print() const
{
    if(_m_nodeType == A_SUC)
    {
        if(_m_containedElem == 0)
            return;    

        cout << "index: ";
        if(_m_indexValid)
            cout<< _m_index;
        else
            cout << "INVALID";
        cout << " - ";
        _m_containedElem->Print();
        cout <<endl;
    }
    //else //if (_m_nodeType == O_SUC)
    
    map< LIndex, TreeNode<ActionObservationHistory>*>::const_iterator
        it = _m_successor.begin();
    while(it != _m_successor.end())
    {        
        pair< LIndex, TreeNode<ActionObservationHistory>*> p = *it;
        TreeNode<ActionObservationHistory>* suc_tn = p.second;
        //we only put ActionObservationHistoryTree's as successors, so
        //we can do this:
        ActionObservationHistoryTree* suc_aoht =
            static_cast<ActionObservationHistoryTree*>(suc_tn);
        if(suc_aoht != 0)
            suc_aoht->Print();
        else
            throw E("ActionObservationHistoryTree::Print() - encountered"\
"a successor of this ActionObservationHistoryTree that is 0 (NULL) ");

        it++;
    }

}

