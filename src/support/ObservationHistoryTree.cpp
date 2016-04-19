/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */


#define DEBUG_OHT 0

/*
//Default constructor
ObservationHistoryTree::ObservationHistoryTree()
{
    _m_index = 0;
    _m_indexValid = false;
    _m_obsHist = 0;
}
ObservationHistoryTree::ObservationHistoryTree(ObservationHistory *const oh)
{
    _m_index = 0;
    _m_indexValid = false;
    _m_obsHist = oh;
}
//Copy assignment constructor.    
ObservationHistoryTree::ObservationHistoryTree(const ObservationHistoryTree& o) 
{
if(DEBUG_OHT){     cout << "Cloning ObservationHistoryTree. This node ";
    PrintThisNode();cout << endl;}
}

//Destructor -- deletes this node, the contained ObservationHistory and all
//successors
ObservationHistoryTree::~ObservationHistoryTree()
{
if(DEBUG_OHT){    cout << "Deleting ObservationHistoryTree. This node ";
    PrintThisNode();cout << endl;}
    
    delete(_m_obsHist);
    vector<ObservationHistoryTree*>::iterator it = _m_successor.begin();
    vector<ObservationHistoryTree*>::iterator last = _m_successor.end();
    
    while(it != last)
    {
        delete(*it);
        it++;
    }
    _m_successor.clear();
}
void ObservationHistoryTree::SetSuccessor(Index observationI, 
    ObservationHistoryTree* suc)
{
    size_t cursize = _m_successor.size();
    if(observationI == cursize)
        _m_successor.push_back(suc);
    else if(observationI < 0 || observationI > _m_successor.size() )
        cout << "ObservationHistoryTree::SetSuccessor ERROR index out of"
        << " bounds! (perhaps setting _m_successor["
        << observationI<< "] before _m_successor[" 
        << observationI-1 << "] ?? )";
    else
    {
        cout << "_m_successor["<< observationI<< "] already set: overwriting!";
        _m_successor[observationI] = suc;
    }
}
ObservationHistoryTree* ObservationHistoryTree::GetSuccessor(Index observationI) const
{
    if(observationI < 0 || observationI >= _m_successor.size() )
        throw E("ObservationHistoryTree::GetSuccessor index out of bounds!");
    else
        return(_m_successor[observationI]);
}
void ObservationHistoryTree::PrintThisNode() const
{
    if(_m_obsHist != 0)
    {
        cout << "index: "<<_m_index<<" - ";
        _m_obsHist->Print();
    }
}
void ObservationHistoryTree::Print() const
{
    if(_m_obsHist != 0)
    {
        cout << "index: "<<_m_index<<" - ";
        _m_obsHist->Print();
        cout <<endl;
        vector<ObservationHistoryTree*>::const_iterator it = _m_successor.begin();
        vector<ObservationHistoryTree*>::const_iterator last = _m_successor.end();
        while(it != last)
        {
            if(*it != 0) (*it)->Print();
            it++;
        }
    }
}
*/
