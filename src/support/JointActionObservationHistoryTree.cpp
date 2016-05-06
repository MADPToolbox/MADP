/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JointActionObservationHistoryTree.h"
#include "PlanningUnitMADPDiscrete.h"

#define DEBUG_JAOHT 0

using namespace std;

/// Create a joint observation history tree for joh
JointActionObservationHistoryTree::JointActionObservationHistoryTree(
        JointActionObservationHistory * const oh, aoh_t nt) :    
    TreeNode<JointActionObservationHistory> (oh)
{
    _m_nodeType = nt; 
}
//Default constructor
/*
JointActionObservationHistoryTree::JointActionObservationHistoryTree()
{
}

//Copy constructor.    
JointActionObservationHistoryTree::JointActionObservationHistoryTree(const JointActionObservationHistoryTree& o) 
{
}
//Destructor
JointActionObservationHistoryTree::~JointActionObservationHistoryTree()
{
}
//Copy assignment operator
JointActionObservationHistoryTree& JointActionObservationHistoryTree::operator= (const JointActionObservationHistoryTree& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/
void JointActionObservationHistoryTree::SetIndex(LIndex i)
{
    if (_m_nodeType == O_SUC)
        throw E("Trying JointActionObservationHistoryTree::SetIndex on O_SUC node");

    this->TreeNode<JointActionObservationHistory>::SetIndex(i);
/*    _m_index = i; 
    _m_indexValid = true; */
};

void JointActionObservationHistoryTree::SetSuccessor(Index aI, Index oI,
        JointActionObservationHistoryTree* suc)
{
    if (_m_nodeType == O_SUC)
        throw E("Trying JointActionObservationHistoryTree::SetSuccessor(Index aI, Index oI, JointActionObservationHistoryTree* suc)  on O_SUC node");

    //first see if necessary to create a new  intermediate node
    JointActionObservationHistoryTree* oNode; 
    if (this->TreeNode<JointActionObservationHistory>::ExistsSuccessor(aI))
        oNode = (JointActionObservationHistoryTree*)
            this->TreeNode<JointActionObservationHistory>::GetSuccessor(aI);
    else //we need to create an intermediate node
    {
        oNode = new JointActionObservationHistoryTree(O_SUC);
        //which is the aI'th successor of this node
        this->TreeNode<JointActionObservationHistory>::SetSuccessor(aI, oNode);
    }

    //let oNode point to the actual successor.
    //Note: SetSuccessor with 1 index should give no problem for O_SUC nodes
    oNode->TreeNode<JointActionObservationHistory>::SetSuccessor(oI, suc);
}

/** Generates a successor if it does not exist yet (in case we did not
 * generate cache the whole tree). */
JointActionObservationHistoryTree* JointActionObservationHistoryTree::GetSuccessor(Index aI, Index oI)
{
    if (_m_nodeType == O_SUC)
        throw E("Trying JointActionObservationHistoryTree::SetSuccessor(Index aI, Index oI, JointActionObservationHistoryTree* suc)  on O_SUC node");

    JointActionObservationHistoryTree* suc;
    if(this->TreeNode<JointActionObservationHistory>::ExistsSuccessor(aI) &&
       this->TreeNode<JointActionObservationHistory>::GetSuccessor(aI)->
       TreeNode<JointActionObservationHistory>::ExistsSuccessor(oI))
        // we only put JointActionObservationHistoryTree*'s in here so
        // this cast should be allowed.
        suc = (JointActionObservationHistoryTree*)
            this->TreeNode<JointActionObservationHistory>::GetSuccessor(aI)//oNode
            ->TreeNode<JointActionObservationHistory>::GetSuccessor(oI);
    else
    {
        // successor does not exist, so let's create it, as we would
        // in
        // PlanningUnitMADPDiscrete::CreateJointActionObservationHistories
        
        JointActionObservationHistory* jaoh = this->
            GetJointActionObservationHistory();
        JointActionObservationHistory* next_jaoh =
            new JointActionObservationHistory(aI, oI, jaoh);
        
        JointActionObservationHistoryTree* next_jaoht =
            new JointActionObservationHistoryTree(next_jaoh);
        
        this->SetSuccessor(aI, oI, next_jaoht);

        next_jaoht->SetIndex(next_jaoh->GetPlanningUnitMADPDiscrete()->
                             GetSuccessorJAOHI(this->GetIndex(),aI,oI));
        // stores the pointer in the big reference vector
        next_jaoh->GetPlanningUnitMADPDiscrete()->
            RegisterJointActionObservationHistoryTree(next_jaoht);

        suc=next_jaoht;
    }

    return(suc);
}

JointActionObservationHistoryTree* JointActionObservationHistoryTree::GetPredecessor() const
{
    if (GetLength()==0) //ts 0 - length 0 NO INITIAL OBSERVATIONS
        throw E("Trying JointActionObservationHistoryTree::SetPredeccessor() on root node. (no predecessor)");
    if (_m_nodeType == O_SUC)
        throw E("Trying JointActionObservationHistoryTree::GetPredecessor()  on O_SUC node");

    JointActionObservationHistoryTree* pred =
        //we only put JointActionObservationHistoryTree*'s in here so this cast
        //should be allowed.
        (JointActionObservationHistoryTree*)
        this->TreeNode<JointActionObservationHistory>::GetPredecessor()//=oNode
        ->TreeNode<JointActionObservationHistory>::GetPredecessor();//pred.aNode

    return(pred);
}


JointActionObservationHistory* JointActionObservationHistoryTree::
    GetJointActionObservationHistory() const
{
    if (_m_nodeType == O_SUC)
        throw E("Trying JointActionObservationHistoryTree::GetJointActionObservationHistory() on O_SUC node");

    return GetContainedElement();
}
    
void JointActionObservationHistoryTree::Print() const
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
    //in both cases (also if _m_nodeType == O_SUC)
    
    map< LIndex, TreeNode<JointActionObservationHistory>*>::const_iterator
        it = _m_successor.begin();
    while(it != _m_successor.end())
    {
        pair< LIndex, TreeNode<JointActionObservationHistory>*> p = *it;
        TreeNode<JointActionObservationHistory>* suc_tn = p.second;
        //we only put JointActionObservationHistoryTree's as successors, so
        //we can do this:
        JointActionObservationHistoryTree* suc_jaoht =
            static_cast<JointActionObservationHistoryTree*>(suc_tn);
        if(suc_jaoht != 0)
            suc_jaoht->Print();
        else
            throw E("JointActionObservationHistoryTree::Print() - encountered"\
"a successor of this JointActionObservationHistoryTree that is 0 (NULL) ");

        it++;
    }
}
