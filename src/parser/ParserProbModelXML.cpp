/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Joao Messias 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "ParserProbModelXML.h"

using namespace std;

namespace{

#ifdef HAVE_LIBXML2

void SetScopes(ParserProbModelXML* parser, FactoredDecPOMDPDiscrete* decpomdp)
{
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Links/Link[@directed='true']";
    xmlXPathObjectPtr link_nodes = parser->GetNodesMatchingExpression(xpath);

    if(link_nodes == NULL)
        throw EParse("No links were found in target file.");

    xmlNodeSetPtr link_nodeset = link_nodes->nodesetval;
    vector<xmlNodePtr> reward_links;
    for(int i = 0; i < link_nodeset->nodeNr; i++)
    {
        xmlNodePtr parent_node = xmlFirstElementChild(link_nodeset->nodeTab[i]);
        xmlNodePtr child_node = xmlLastElementChild(link_nodeset->nodeTab[i]);
        string parent_name = parser->GetVariableName(parent_node);
        int timeslice = parser->GetVariableTimeslice(parent_node);
        string child_name = parser->GetVariableName(child_node);
        std::pair<ParserProbModelXML::elm_type, Index> parent_elm = parser->GetParsedElement(parent_name);
        std::pair<ParserProbModelXML::elm_type, Index> child_elm = parser->GetParsedElement(child_name);

        Index cidx = child_elm.second, pidx = parent_elm.second;
        Scope ASoI;
        Scope ASoILocal;
        Scope XSoI;  //State factor SoI at t
        Scope YSoI;  //State factor SoI at t+1
        switch(child_elm.first) //child type
        {
            case ParserProbModelXML::STATE:
                ASoI = decpomdp->GetASoI_Y(cidx);
                XSoI = decpomdp->GetXSoI_Y(cidx);
                YSoI = decpomdp->GetYSoI_Y(cidx);
                break;
            case ParserProbModelXML::OBSERVATION:
                XSoI = decpomdp->GetXSoI_O(cidx);
                ASoI = decpomdp->GetASoI_O(cidx);
                YSoI = decpomdp->GetYSoI_O(cidx);
                break;
            case ParserProbModelXML::REWARD:
                //Reward links have to be handled differently, since they may depend on either 1st or 2nd time-slice factors,
                //and we have to backup the scopes in the latter case.
                reward_links.push_back(link_nodeset->nodeTab[i]);
                continue;
            default:
                continue;  //links to Actions can be ignored, since they imply observations.
        }

        switch(parent_elm.first) //parent type
        {
            case ParserProbModelXML::STATE:
                if(timeslice == 0)
                {
                    if(!XSoI.Contains(pidx))
                    {
                        XSoI.Insert(pidx);
                    }
                }
                else
                {
                    if(!YSoI.Contains(pidx))
                    {
                        YSoI.Insert(pidx);
                    }
                }
                break;
            case ParserProbModelXML::ACTION:
                if(parser->IsAsynchronous()){
                    ASoILocal.Insert(pidx);
                    pidx = 0;
                }
                if(!ASoI.Contains(pidx))
                {
                    ASoI.Insert(pidx);
                }
                break;
            default:
                if(DEBUG_PARSE)
                      cout << "Parent is an observation. Ignoring." << endl;
                break; //links from Observations are ignored for now.
        }

        switch(child_elm.first) //child type
        {
            case ParserProbModelXML::STATE:
                decpomdp->SetSoI_Y(cidx,
                                   XSoI,              //const Scope & XSoI
                                   ASoI,              //const Scope & ASoI
                                   YSoI);             //const Scope & YSoI 
                
                if(parser->IsAsynchronous() && !ASoILocal.empty())
                    parser->AddToLocalSFActionScope(cidx,ASoILocal);
                
                break;
            case ParserProbModelXML::OBSERVATION:
                decpomdp->SetSoI_O(cidx,
                                   XSoI,               //const Scope & XSoI
                                   ASoI,               //const Scope & ASoI
                                   YSoI,               //const Scope & YSoI 
                                   Scope("<>"));       //const Scope & OSoI

                if(parser->IsAsynchronous() && !ASoILocal.empty())
                    parser->AddToLocalObsActionScope(cidx,ASoILocal);
                
                break;
            default:
                continue;  //links to Actions can be ignored, since they imply observations.
        }

        if(DEBUG_PARSE)
            cout << "found a link from " << parent_name << " ["<< timeslice <<"] to " << child_name << endl;
    }
    for(size_t i = 0; i < reward_links.size(); i++)
    {
        //now for the rewards
        xmlNodePtr parent_node = xmlFirstElementChild(reward_links[i]);
        xmlNodePtr child_node = xmlLastElementChild(reward_links[i]);
        string parent_name = parser->GetVariableName(parent_node);
        string child_name = parser->GetVariableName(child_node);
        std::pair<ParserProbModelXML::elm_type, Index> parent_elm = parser->GetParsedElement(parent_name);
        std::pair<ParserProbModelXML::elm_type, Index> child_elm = parser->GetParsedElement(child_name);

        Index LRFidx = child_elm.second, pidx = parent_elm.second;
        Scope ASoI = decpomdp->GetAgentScopeForLRF(LRFidx);
        Scope ASoILocal;
        Scope XSoI = decpomdp->GetStateFactorScopeForLRF(LRFidx);
        Scope YSoI;
        Scope OSoI;
        int timeslice = parser->GetVariableTimeslice(parent_node);

        if(timeslice == 0)
        {
            //the usual case
            switch(parent_elm.first) //parent type
            {
                case ParserProbModelXML::STATE:
                    if(!XSoI.Contains(pidx))
                    {
                        XSoI.Insert(pidx);
                    }
                    break;
                case ParserProbModelXML::ACTION:
                    if(parser->IsAsynchronous()){
                        ASoILocal.Insert(pidx);
                        pidx = 0;
                    }
                    if(!ASoI.Contains(pidx))
                    {
                        ASoI.Insert(pidx);
                    }
                    break;
                default:
                    continue; //links from Observations are ignored for now.
            }
        }
        else
        {
            switch(parent_elm.first) //parent type
            {
                case ParserProbModelXML::STATE:
                    YSoI.Insert(pidx);
                    break;
                case ParserProbModelXML::OBSERVATION:
                    OSoI.Insert(pidx);
                    break;
                default:
                    continue; //links from NS actions are invalid.
            }
        }
        decpomdp->SetScopeForLRF(LRFidx, XSoI, ASoI, YSoI, OSoI);
        
        if(parser->IsAsynchronous() && !ASoILocal.empty())
            parser->AddToLocalLRFActionScope(LRFidx,ASoILocal);
    }
    xmlXPathFreeObject (link_nodes);
}

bool CompareParsedDependencies(pair<Index,Index> p1, pair<Index,Index> p2)
{
    return p1.first < p2.first;
}

bool CompareParsedActions(pair<Index, xmlNodePtr> p1, pair<Index, xmlNodePtr > p2)
{
    return p1.first < p2.first;
}

double ComputeTransitionProb(Index y, 
                             Index yVal,
                             const std::vector< Index>& Xs,
                             const std::vector< Index>& As,
                             const std::vector< Index>& Ys,
                             ParserProbModelXML* parser,
                             const FactoredDecPOMDPDiscrete* decpomdp)
{
    Scope XSoI = decpomdp->GetXSoI_Y(y);
    Scope ASoI;
    const std::vector< Index>* p_As = 0;
    std::vector< Index> AsLocal;
    if(!parser->IsAsynchronous()){
        ASoI = decpomdp->GetASoI_Y(y);
        p_As = &As;
    }
    else if(As.size() > 0)
    {
        parser->ConvertToLocalSFActionScope(y,As[0],AsLocal,ASoI); 
        p_As = &AsLocal;
    }
    Scope YSoI = decpomdp->GetYSoI_Y(y);

    map<string, pair<Index, Index> > t0_deps;
    map<string, pair<Index, Index> > t1_deps;

    for(size_t i = 0; i < Xs.size(); i++)
    {
        Index xi = XSoI[i];
        string factor_name = decpomdp->GetStateFactorDiscrete(xi)->GetName();
        t0_deps[factor_name] = make_pair(Xs[i], decpomdp->GetNrValuesForFactor(xi));
    }
    
    if(p_As != 0)
    {
        for(size_t i = 0; i < p_As->size(); i++)
        {
            //Decision node names are not stored in the decpomdp: only the names of their values.
            Index ai = ASoI[i];
            pair<ParserProbModelXML::elm_type, Index> elm = make_pair(ParserProbModelXML::ACTION, ai);
            string factor_name = parser->GetParsedElementName(elm);
            size_t nrActions;
            if(!parser->IsAsynchronous())
                nrActions = decpomdp->GetNrActions(ai);
            else
                nrActions = parser->GetLocalNrActions(ai);
            
            t0_deps[factor_name] = make_pair(p_As->at(i), nrActions);
        }
    }
    
    for(size_t i = 0; i < Ys.size(); i++)
    {
        Index yi = YSoI[i];
        string factor_name = decpomdp->GetStateFactorDiscrete(yi)->GetName();
        t1_deps[factor_name] = make_pair(Ys[i], decpomdp->GetNrValuesForFactor(yi));
    }
    const StateFactorDiscrete* sf = decpomdp->GetStateFactorDiscrete(y);
    string sf_name = sf->GetName();
    t1_deps[sf_name] = make_pair(yVal, decpomdp->GetNrValuesForFactor(y));
    double_t p = parser->GetPotential(sf_name, t0_deps, t1_deps);
    if(DEBUG_PARSE)
    {
        cout << "Xs: ";
        for(size_t i = 0; i < Xs.size(); i++)
            cout << Xs[i] << " ";
        cout << "| As: ";
        if(!parser->IsAsynchronous()){
            for(size_t i = 0; i < p_As->size(); i++)
                cout << p_As->at(i) << " ";
        }else{
            for(size_t i = 0; i < AsLocal.size(); i++)
                cout << AsLocal[i] << " ";
        }
        cout << "| Ys: ";
        for(size_t i = 0; i < Ys.size(); i++)
            cout << Ys[i] << " ";
        cout << "| y=" << y << " | yVal=" << yVal << " | p = " << p << endl;
    }
    return p;
}

double ComputeObservationProb(Index o,
                              Index oVal,
                              const std::vector< Index>& Xs,
                              const std::vector< Index>& As,
                              const std::vector< Index>& Ys,
                              const std::vector< Index>& Os,
                              ParserProbModelXML* parser,
                              const FactoredDecPOMDPDiscrete* decpomdp)
{    
    Scope XSoI = decpomdp->GetXSoI_O(o);
    Scope ASoI;
    const std::vector< Index>* p_As = 0;
    std::vector< Index> AsLocal;
    if(!parser->IsAsynchronous()){
        ASoI = decpomdp->GetASoI_O(o);
        p_As = &As;
    }
    else if(As.size() > 0)
    {
        parser->ConvertToLocalObsActionScope(o,As[0],AsLocal,ASoI); 
        p_As = &AsLocal;
    }
    Scope YSoI = decpomdp->GetYSoI_O(o);
    Scope OSoI = decpomdp->GetOSoI_O(o);

    map<string, pair<Index, Index> > t0_deps;
    map<string, pair<Index, Index> > t1_deps;

    if(p_As != 0)
    {
        for(size_t i = 0; i < p_As->size(); i++)
        {
            //Decision node names are not stored in the decpomdp: only the names of their values.
            Index ai = ASoI[i];
            pair<ParserProbModelXML::elm_type, Index> elm = make_pair(ParserProbModelXML::ACTION, ai);
            string factor_name = parser->GetParsedElementName(elm);
            size_t nrActions;
            if(!parser->IsAsynchronous())
                nrActions = decpomdp->GetNrActions(ai);
            else
                nrActions = parser->GetLocalNrActions(ai);
            
            t0_deps[factor_name] = make_pair(p_As->at(i), nrActions);
        }
    }
    for(size_t i = 0; i < Xs.size(); i++)
    {
        Index xi = XSoI[i];
        string factor_name = decpomdp->GetStateFactorDiscrete(xi)->GetName();
        t0_deps[factor_name] = make_pair(Xs[i], decpomdp->GetNrValuesForFactor(xi));
    }
    for(size_t i = 0; i < Ys.size(); i++)
    {
        Index yi = YSoI[i];
        string factor_name = decpomdp->GetStateFactorDiscrete(yi)->GetName();
        t1_deps[factor_name] = make_pair(Ys[i], decpomdp->GetNrValuesForFactor(yi));
    }
    for(size_t i = 0; i < Os.size(); i++)
    {
        //Decision node names are not stored in the decpomdp: only the names of their values.
        Index oi = OSoI[i];
        pair<ParserProbModelXML::elm_type, Index> elm = make_pair(ParserProbModelXML::OBSERVATION, oi);
        string factor_name = parser->GetParsedElementName(elm);
        t1_deps[factor_name] = make_pair(Os[i], decpomdp->GetNrObservations(oi));
    }

    pair<ParserProbModelXML::elm_type, Index> elm = make_pair(ParserProbModelXML::OBSERVATION, o);
    string obs_name = parser->GetParsedElementName(elm);
    t1_deps[obs_name] = make_pair(oVal, decpomdp->GetNrObservations(o));

    double p = parser->GetPotential(obs_name, t0_deps, t1_deps);
    if(DEBUG_PARSE)
    {
        cout << "Xs: ";
        for(size_t i = 0; i < Xs.size(); i++)
            cout << Xs[i] << " ";
        cout << "| As: ";
        for(size_t i = 0; i < As.size(); i++)
            cout << As[i] << " ";
        cout << "| Ys: ";
        for(size_t i = 0; i < Ys.size(); i++)
            cout << Ys[i] << " ";
        cout << "| Os: ";
        for(size_t i = 0; i < Os.size(); i++)
            cout << Os[i] << " ";
        cout << "| o=" << o << " | oVal=" << oVal << " | p = " << p << endl;
    }
    return p;
}
#endif
}

//Default constructor
ParserProbModelXML::ParserProbModelXML(FactoredDecPOMDPDiscrete* problem) :
    _m_agentsInitialized(false),
    _m_fDecPOMDP(problem),
    _m_asynchronousModel(false)
{
    if(DEBUG_PARSE)
    {
        cout << "Creating XML parser, referring to problem...";
        cout << problem->MultiAgentDecisionProcess::SoftPrint();
        cout << endl;
    }

}

#ifdef HAVE_LIBXML2
void ParserProbModelXML::InitializeXPath()
{
    string pf = _m_fDecPOMDP->GetProblemFile();
    const char* pf_c = pf.c_str();
    _m_doc = xmlParseFile(pf_c);
    if(_m_doc == NULL)
        throw EParse("Could not open XML file.");

    _m_context = xmlXPathNewContext(_m_doc); //context for global lookups
    if (_m_context == NULL) {
        throw EParse("Could not retrieve context from XML file.");
    }
}

const xmlXPathObjectPtr ParserProbModelXML::GetNodesMatchingExpression(const xmlChar *expr, const xmlNodePtr context_node)
{
    if(context_node != 0){
        SetContext(context_node);
    }
    
    xmlXPathObjectPtr match = xmlXPathEvalExpression(expr, GetContext());
    if (match == NULL)
        return NULL;

    if(xmlXPathNodeSetIsEmpty(match->nodesetval))
    {
        xmlXPathFreeObject(match);
        return NULL;
    }

    if(context_node != 0){
        ResetContext();
    }
    
    return match;
}

string ParserProbModelXML::GetVariableName(const xmlNodePtr node) const
{
    if(!xmlHasProp(node, (xmlChar*) "name")){
        cout << "Warning: Attempted to extract the name of a nameless node" << endl;
        return "";
    }
    string name = (char*) xmlGetProp(node, (xmlChar*) "name");
    size_t pos = name.find_last_not_of(" \t");
    //remove the [0]/[1] tag from the name if it exists (for older versions of OpenMarkov).
    if(pos <= name.length()-1 && pos > 0)
    {
        if(name[pos] == ']')
            name.erase(pos-2);
    }
    //remove remaining whitespace
    return StringTools::Trim(name);
}

int ParserProbModelXML::GetVariableTimeslice(const xmlNodePtr node) const
{
    if(!xmlHasProp(node, (xmlChar*) "timeSlice")){
        cout << "Warning: Attempted to extract the timeSlice of a time-less node. Defaulting to 0." << endl;
        return 0;
    }
    string timeslice = (char*) xmlGetProp(node, (xmlChar*) "timeSlice");
    return atoi(timeslice.c_str());
}

std::string ParserProbModelXML::GetVariableComment(const xmlNodePtr node)
{
    //only works for plain text comments so far.
    string comment = "";
    xmlXPathObjectPtr comment_node = GetNodesMatchingExpression((xmlChar*) "Comment",node);
    if(comment_node != NULL)
    {
        comment = (char*) xmlNodeListGetString(GetDoc(), comment_node->nodesetval->nodeTab[0]->xmlChildrenNode, 1);
        if(DEBUG_PARSE)
        {
            cout << "read comment: " << comment << endl;
        }
        xmlXPathFreeObject (comment_node);
    }
    return comment;
}

void ParserProbModelXML::ReadStates()
{
    if(DEBUG_PARSE)
    {
        cout << " --- Now Parsing States ---" << endl;
    }
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Variables/Variable[@timeSlice='0' and @role='chance']";
    xmlXPathObjectPtr state_nodes = GetNodesMatchingExpression(xpath);

    if (state_nodes == NULL)
        throw EParse("No state nodes found in target file.");

    xmlNodeSetPtr state_nodeset = state_nodes->nodesetval;
    for (int i = 0; i < state_nodeset->nodeNr; i++) 
    {
        xmlXPathObjectPtr child_nodes = GetNodesMatchingExpression((xmlChar*) "States/State",state_nodeset->nodeTab[i]);
        if(child_nodes != NULL)
        {
            xmlNodeSetPtr child_nodeset = child_nodes->nodesetval;
            if(child_nodeset->nodeNr > 1)
            {
                string factorName = GetVariableName(state_nodeset->nodeTab[i]);
                if(DEBUG_PARSE)
                {
                    cout << "Added state with name: " << factorName << endl;
                }
                Index sfI = _m_fDecPOMDP->AddStateFactor(factorName,
                                                         GetVariableComment(state_nodeset->nodeTab[i]));
                for(int j = 0; j < child_nodeset->nodeNr; j++)
                {
                    _m_fDecPOMDP->AddStateFactorValue(sfI, GetVariableName(child_nodeset->nodeTab[j]));
                    if(DEBUG_PARSE)
                    {
                        cout << "    Added value: " << GetVariableName(child_nodeset->nodeTab[j]) << endl;
                    }
                }
                pair<elm_type, Index> element = make_pair(STATE, i);
                _m_parsedElements[factorName] = element;
            }
            xmlXPathFreeObject (child_nodes);
        }
    }
    
    _m_fDecPOMDP->SetStatesInitialized(true);

    xmlXPathFreeObject (state_nodes);
    return;
}

void ParserProbModelXML::ReadAgents()
{
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Agents/Agent";
    xmlXPathObjectPtr agent_nodes = GetNodesMatchingExpression(xpath);

    if (_m_asynchronousModel || agent_nodes == NULL || agent_nodes->nodesetval->nodeNr < 2){
        _m_fDecPOMDP->SetNrAgents(1);
        xmlXPathFreeObject (agent_nodes);
        _m_agentsInitialized = true;
        return;
    }
    xmlNodeSetPtr nodeset = agent_nodes->nodesetval;
    for (int i = 0; i < nodeset->nodeNr; i++) {
      _m_fDecPOMDP->AddAgent(GetVariableName(nodeset->nodeTab[i]));
    }
    xmlXPathFreeObject (agent_nodes);
    _m_agentsInitialized = true;
    return;
}


void ParserProbModelXML::ReadActions()
{
    if(DEBUG_PARSE)
    {
        cout << " --- Now Parsing Actions ---" << endl;
    }
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Variables/Variable[@timeSlice='0' and @role='decision']";
    xmlXPathObjectPtr action_nodes = GetNodesMatchingExpression(xpath);

    if (action_nodes == NULL)
        throw EParse("No action nodes found in target file.");

    xmlNodeSetPtr action_nodeset = action_nodes->nodesetval;
    if(!_m_agentsInitialized || _m_fDecPOMDP->GetNrAgents() != (unsigned) action_nodeset->nodeNr)
    {
        if(!_m_asynchronousModel){
            //this is a failsafe for models with more than one action space factor, but no specified agents.
            cout << "Warning: The specified number of agents is different from the number of decision nodes." << endl;
            _m_fDecPOMDP->SetNrAgents(action_nodeset->nodeNr);
            _m_agentsInitialized = false;
        }
    }
    for (int i = 0; i < action_nodeset->nodeNr; i++) 
    {
        Index aI;
        if(_m_agentsInitialized && _m_fDecPOMDP->GetNrAgents() > 1)
        {
            xmlXPathObjectPtr agent_node = GetNodesMatchingExpression((xmlChar*) "Agent",action_nodeset->nodeTab[i]);
            if(agent_node == NULL)
                throw EParse("Found an action node with no associated agent.");
            
            string agent_name = GetVariableName(agent_node->nodesetval->nodeTab[0]);
            aI = _m_fDecPOMDP->GetAgentIndexByName(agent_name);
            xmlXPathFreeObject (agent_node);
        }
        else
        {
            aI = i;
        }
        xmlXPathObjectPtr child_nodes = GetNodesMatchingExpression((xmlChar*) "States/State",action_nodeset->nodeTab[i]);
        if(child_nodes != NULL)
        {
            xmlNodeSetPtr child_nodeset = child_nodes->nodesetval;
            if(child_nodeset->nodeNr > 0)
            {
                string comment = GetVariableComment(action_nodeset->nodeTab[i]);
                for(int j = 0; j < child_nodeset->nodeNr; j++)
                {
                    if(!_m_asynchronousModel)
                    {
                        _m_fDecPOMDP->AddAction(aI, GetVariableName(child_nodeset->nodeTab[j]), comment);
                        if(DEBUG_PARSE)
                        {
                            cout << "Added action: " << GetVariableName(child_nodeset->nodeTab[j]) << " to agent " << aI << endl;
                        }
                    }
                    else
                    {
                        if(_m_ASizes.size() <= aI)
                            _m_ASizes.push_back(child_nodeset->nodeNr);
                        if(_m_ANames.size() <= aI)
                            _m_ANames.push_back(vector<string> (1,GetVariableName(child_nodeset->nodeTab[j])));
                        else
                            _m_ANames[aI].push_back(GetVariableName(child_nodeset->nodeTab[j]));
                    }
                }
                pair<elm_type, Index> element = make_pair(ACTION, aI);
                _m_parsedElements[GetVariableName(action_nodeset->nodeTab[i])] = element;
            }
            xmlXPathFreeObject (child_nodes);
        }
    }
    xmlXPathFreeObject (action_nodes);
    
    if(_m_asynchronousModel)
    {
        vector<Index> As(_m_ASizes.size(),0);
        do{
            stringstream aName;
            for(size_t i = 0; i < As.size(); i++)
            {
                aName << _m_ANames[i][As[i]];
                if(i+1 < As.size())
                    aName << "_";
            }
            _m_fDecPOMDP->AddAction(0, aName.str());
            if(DEBUG_PARSE)
            {
                cout << "Added action: " << aName.str() << " to agent 0" << endl;
            }
        } while(! IndexTools::Increment( As, _m_ASizes ) ); 
    }
    //    _m_fDecPOMDP->ConstructJointActions();
    _m_fDecPOMDP->SetActionsInitialized(true);

    return;
}

void ParserProbModelXML::ReadObservations()
{
    if(DEBUG_PARSE)
    {
        cout << " --- Now Parsing Observations ---" << endl;
    }
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Variables/Variable[@timeSlice='1' and @role='decision']";
    xmlXPathObjectPtr action_nodes = GetNodesMatchingExpression(xpath);

    if (action_nodes == NULL){
        cout  << "Warning: No action nodes for the second timeslice were found in the target file. There are no observations in the model." << endl;
        return;
    }

    xmlNodeSetPtr action_nodeset = action_nodes->nodesetval;
    vector<pair<Index, xmlNodePtr> > parsed_actions;
    for (int i = 0; i < action_nodeset->nodeNr; i++)
    {
        Index aI;
        if(_m_agentsInitialized && _m_fDecPOMDP->GetNrAgents() > 1)
        {
            xmlXPathObjectPtr agent_node = GetNodesMatchingExpression((xmlChar*) "Agent",action_nodeset->nodeTab[i]);
            if(agent_node == NULL)
                throw EParse("Found an action node with no associated agent.");
            
            string agent_name = GetVariableName(agent_node->nodesetval->nodeTab[0]);
            aI = _m_fDecPOMDP->GetAgentIndexByName(agent_name);
            xmlXPathFreeObject (agent_node);
        }
        else
        {
            aI = i;
        }
        parsed_actions.push_back(make_pair(aI, action_nodeset->nodeTab[i]));
    }

    //Observations have to be loaded in agent-wise order
    sort(parsed_actions.begin(), parsed_actions.end(), CompareParsedActions);

    for (size_t i = 0; i < parsed_actions.size(); i++)
    {
        Index aI;
        if(_m_asynchronousModel)
            aI = 0; //asynchronous models are assumed to be centralized, for now.
        else
            aI = parsed_actions[i].first;
        //retrieving the nodes which link to each decision node for T+1
        stringstream link_path_ss;
        link_path_ss << "/ProbModelXML/ProbNet/Links/Link/Variable[starts-with(@name, \""
                     << (char*) xmlGetProp(parsed_actions[i].second, (xmlChar*) "name")
                     << "\") and @timeSlice='1']";
        xmlXPathObjectPtr link_nodes = GetNodesMatchingExpression((xmlChar*) link_path_ss.str().c_str());

        if(link_nodes != NULL)
        {
            xmlNodeSetPtr link_nodeset = link_nodes->nodesetval;
            if(link_nodeset->nodeNr > 1){
                cout << "Warning: multiple obervation factors for one agent are not yet supported." << endl;
            }
            //For now, it is assumed that the observation model is factored agent-wise - that is, there is no more than one 
            //chance node linked to each decision node in slice T+1. If support for this feature is needed, it should be added here.
            //Each link_nodeset->nodeTab element should represent one of the observation factors of agent i.
            xmlNodePtr parent_node = xmlFirstElementChild(link_nodeset->nodeTab[0]->parent);
            string parent_name =  GetVariableName(parent_node);
            //now we must retrieve the original chance nodes with this name and parse them.
            stringstream obs_path_ss;
            obs_path_ss << "/ProbModelXML/ProbNet/Variables/Variable[(@name='"
                        << parent_name << "' or @name='" << parent_name << " [1]') and @timeSlice='1' and @role='chance']";
            string comment = GetVariableComment(parent_node);
            obs_path_ss << "/States/State";
            xmlXPathObjectPtr obs_nodes = GetNodesMatchingExpression((xmlChar*) obs_path_ss.str().c_str());
            if(obs_nodes != NULL)
            {        
                xmlNodeSetPtr obs_nodeset = obs_nodes->nodesetval;
                for(int j = 0; j < obs_nodeset->nodeNr; j++)
                {
                    string obs_name = GetVariableName(obs_nodeset->nodeTab[j]);
                    _m_fDecPOMDP->AddObservation(aI, obs_name, comment);
                    pair<elm_type, Index> element = make_pair(OBSERVATION, aI);
                    _m_parsedElements[GetVariableName(parent_node)] = element;
                    if(DEBUG_PARSE)
                    {
                        cout << "Added observation: " << obs_name << " to agent " << aI << endl;
                    }
                }
                xmlXPathFreeObject (obs_nodes);
            }
            xmlXPathFreeObject (link_nodes);
        }
    }
    xmlXPathFreeObject (action_nodes);

    _m_fDecPOMDP->SetObservationsInitialized(true);

    return;
}

const std::pair<ParserProbModelXML::elm_type, Index> ParserProbModelXML::GetParsedElement(const std::string name) const
{
    map<string, pair<elm_type, Index> >::const_iterator it = _m_parsedElements.find(name);

    if(it == _m_parsedElements.end())
    {
        throw EParse("Requested undefined variable \"" + name + "\".");
    }

    return it->second;
}

std::string ParserProbModelXML::GetParsedElementName(const std::pair<ParserProbModelXML::elm_type, Index> elm) const
{
    map<string, pair<elm_type, Index> >::const_iterator it;

    for(it = _m_parsedElements.begin(); it != _m_parsedElements.end(); it++)
    {
      if(elm == (*it).second)
        return (*it).first;
    }
    return " ";
}

void ParserProbModelXML::SetISD()
{
    FSDist_COF* isd = new FSDist_COF(*_m_fDecPOMDP);
    for(size_t i = 0; i < _m_fDecPOMDP->GetNrStateFactors(); i++)
    {
        bool setUniform = false;
        string factor_name = _m_fDecPOMDP->GetStateFactorDiscrete(i)->GetName();
        stringstream xpath_ss; 
        xpath_ss << "/ProbModelXML/ProbNet/Potentials/Potential[@role='conditionalProbability']/Variables/Variable";
        xpath_ss << "[starts-with(@name, \"" << factor_name << "\") and @timeSlice='0']";
        xmlXPathObjectPtr isd_nodes = GetNodesMatchingExpression((xmlChar*) xpath_ss.str().c_str());
        if(isd_nodes == NULL)
        {
            cout << "Warning: Variable " << factor_name << " has no ISD. Setting to uniform. " << endl;
            setUniform = true;
        }
        else
        {
            xmlNodeSetPtr isd_nodeset = isd_nodes->nodesetval;
            xmlNodePtr isd_node = isd_nodeset->nodeTab[0];
            for(int j = 0; j < isd_nodeset->nodeNr; j++)
            {
                isd_node = isd_nodeset->nodeTab[j];
                xmlNodePtr p = isd_node->parent;
                if(xmlChildElementCount(p) == 1)
                    break; //ISD nodes are one-dimensional arrays
            }
            if(xmlChildElementCount(isd_node->parent) != 1)
            {
                cout << "Warning: Variable " << factor_name << " has no ISD. Setting to uniform. " << endl;
                setUniform = true;
            }
            string CPD_type = (char*) xmlGetProp(isd_node->parent->parent, (xmlChar*) "type"); //Potential
            if(CPD_type == "Table")
            {
                xmlNodePtr data_node = FindChild(isd_node->parent->parent, "Values");
                if(data_node == NULL)
                    throw EParse("Variable " + factor_name + " has an empty ISD");
                const vector<double>* dist = ParseArray(data_node);
                for(size_t j = 0; j < dist->size(); j++)
                {
                    isd->SetProbability(i, j, dist->at(j));
                }
            }
            else if(CPD_type == "Uniform")
            {
                setUniform = true;
            }
            else if(CPD_type == "Tree/ADD")
            {
                throw EParse("At variable " + factor_name + ": CPD type \"" + CPD_type + "\" not allowed for initial belief.");
            }
        }
        if(setUniform)
        {
            size_t n = _m_fDecPOMDP->GetNrValuesForFactor(i);
            for(size_t j = 0; j < n; j++)
            {
                isd->SetProbability(i, j, 1.0/(float) n);
            }
        }

        xmlXPathFreeObject (isd_nodes);
    }

    isd->SanityCheck();
    _m_fDecPOMDP->SetISD(isd);
  
    return;
}

double ParserProbModelXML::GetPotential(const std::string var_name,
                                        const std::map<std::string, std::pair<Index, Index> > t0_deps, 
                                        const std::map<std::string, std::pair<Index, Index> > t1_deps,
                                        const bool isUtility)
{
    stringstream xpath_ss;
    xmlXPathObjectPtr var_nodes;
    xpath_ss << "/ProbModelXML/ProbNet/Potentials/Potential";
    if(!isUtility)
    {
        xpath_ss << "[@role='conditionalProbability']/Variables/Variable[@timeSlice='1' and contains(@name, \"" 
                 << var_name << "\")]";
    }
    else
    {
        xpath_ss << "[@role='utility']/UtilityVariable[starts-with(@name, \"" 
                 << var_name << "\")]/following-sibling::Variables/Variable";
    }
    var_nodes = GetNodesMatchingExpression((xmlChar*) xpath_ss.str().c_str());
    if(var_nodes == NULL)
        throw EParse("Could not find CPD for variable " + var_name);

    xmlNodeSetPtr var_nodeset = var_nodes->nodesetval;
    xmlNodePtr top = var_nodeset->nodeTab[0];
    if(!isUtility)
    {
        bool cpd_found = false;
        for(int i = 0; !cpd_found && i < var_nodeset->nodeNr; i++)
        {
            top = var_nodeset->nodeTab[i];
            cpd_found = (top == xmlFirstElementChild(top->parent) &&
                         GetVariableName(top) == var_name); //this prevents possible partial matchings in the variable name
        }

        if(!cpd_found)
            throw EParse("Could not find CPD for variable " + var_name + "(empty potential)");
    }
    xmlNodePtr parent = top->parent->parent;
    string CPD_type = (char*) xmlGetProp(parent, (xmlChar*) "type"); //Potential
    if(CPD_type == "Uniform")
    {
        map<string, pair<Index, Index> >::const_iterator it = t1_deps.find(var_name);
        double p = 1.0/(it->second.second);
        if(DEBUG_PARSE)
        {
              cout << "Output: " << var_name << "=" << it->second.first << " | Input: Uniform | " << "p=" << p << endl;
        }
        xmlXPathFreeObject (var_nodes);
        return p;
    }
    else if(CPD_type == "Table")
    {
        double p = QueryTable(parent, t0_deps, t1_deps);
        xmlXPathFreeObject (var_nodes);
        return p;
    }
    else if(CPD_type == "Tree/ADD")
    {
        double p = QueryADD(parent, parent, t0_deps, t1_deps);
        xmlXPathFreeObject (var_nodes);
        return p;
    }
    else
    {
        throw EParse("CPD type \"" + CPD_type + "\" not supported.");
    }
}

xmlNodePtr ParserProbModelXML::FindChild(const xmlNodePtr node, const std::string child) const
{
    xmlNodePtr cur = xmlFirstElementChild(node);
    while(cur != NULL && strcmp((char*) cur->name,child.c_str()))
        cur = cur->next;
    return cur;
}

double ParserProbModelXML::QueryADD(const xmlNodePtr root_node,
                                    const xmlNodePtr current_node,
                                    const std::map<std::string, std::pair<Index, Index> > t0_deps, 
                                    const std::map<std::string, std::pair<Index, Index> > t1_deps)
{
    //find top variable
    xmlNodePtr top = FindChild(current_node, "TopVariable");
    if(top == NULL)
        throw EParse("Incorrectly specified Tree/ADD (no top variable).");
   
    string top_name = GetVariableName(top);
    int timeslice = GetVariableTimeslice(top);    
    
    map<string, pair<Index, Index> >::const_iterator it;
    if(timeslice == 0){ 
        it = t0_deps.find(top_name);
        if(it == t0_deps.end())
        {
            throw EParse("Top variable " + top_name + " not present in dependencies.");
        }
    }
    else
    {
        it = t1_deps.find(top_name);
        if(it == t1_deps.end())
        {
            throw EParse("Top variable " + top_name + " not present in dependencies.");
        }
    }

    Index value = it->second.first; //the (integer) value of the requested top variable
    std::pair<elm_type, Index> elm = GetParsedElement(top_name);
    string value_name;
    switch(elm.first)
    {
    case STATE:
        value_name = _m_fDecPOMDP->GetStateFactorDiscrete(elm.second)->GetStateFactorValue(value);
        break;
    case ACTION:
        if(!_m_asynchronousModel)
            value_name = _m_fDecPOMDP->GetAction(elm.second, value)->GetName();
        else
            value_name = _m_ANames[elm.second][value];
        break;
    case OBSERVATION:
        value_name = _m_fDecPOMDP->GetObservation(elm.second, value)->GetName();
        break;
    default:
        throw EParse("Variable " + top_name + " has unrecognized dependencies.");
        break;
    }
    xmlNodePtr branch_set = FindChild(current_node, "Branches");
    if(branch_set == NULL)
        throw EParse("Incorrectly specified Tree/ADD (no branches).");

    xmlNodePtr branch = xmlFirstElementChild(branch_set);
    bool found_branch = false;
    while(branch != NULL && !found_branch)
    {
        if(branch->type == XML_ELEMENT_NODE)
        {
            xmlNodePtr states = FindChild(branch, "States");
            if(xmlChildElementCount(states) == 0)
            {
                found_branch = true;
            }
            else
            {
                xmlNodePtr var_state = xmlFirstElementChild(states);
                while(var_state != NULL && !found_branch)
                {
                    if(var_state->type == XML_ELEMENT_NODE)
                    {
                        string var_name = GetVariableName(var_state);
                        if(value_name == var_name)
                        {
                            found_branch = true;
                        }
                    }
                    var_state = var_state->next;
                }
            }
            if(found_branch)
            {
                //Check if it is a reference to another branch
                xmlNodePtr reference = FindChild(branch, "Reference");
                if(reference != NULL)
                {
                    string ref_name((char*) xmlNodeListGetString(GetDoc(), reference->xmlChildrenNode, 1));
                    string xpath_str = ".//Branch/Label[contains(.,'"+ref_name+"')]";
                    xmlXPathObjectPtr label_nodes = GetNodesMatchingExpression((xmlChar*) xpath_str.c_str(),root_node);                    
                    if(label_nodes == NULL)
                    {
                        throw EParse("Label \"" + ref_name + "\" not found.");
                    }
                    else if(label_nodes->nodesetval->nodeNr > 1)
                    {
                        throw EParse("Label \"" + ref_name + "\" defined multiple times.");
                    }
                    branch = label_nodes->nodesetval->nodeTab[0]->parent;
                }
                
                xmlNodePtr potential = FindChild(branch, "Potential");
                if(potential != NULL)
                {
                    string CPD_type = (char*) xmlGetProp(potential, (xmlChar*) "type");
                    if(CPD_type == "Tree/ADD")
                    {
                        return QueryADD(root_node, potential, t0_deps, t1_deps);
                    }
                    else if(CPD_type == "Table")
                    {
                        return QueryTable(potential, t0_deps, t1_deps);
                    }
                    else if(CPD_type == "Uniform")
                    {
                        map<string, pair<Index, Index> >::const_iterator it;
                        xmlNodePtr dep_var = FindChild(FindChild(potential, "Variables"), "Variable");
                        string dep_var_name = GetVariableName(dep_var);
                        int dep_var_time = GetVariableTimeslice(dep_var);
                        if(dep_var_time == 0)
                        {
                            it = t0_deps.find(dep_var_name);
                        }
                        else
                        {
                            it = t1_deps.find(dep_var_name);
                        }
                        return 1.0/(float) it->second.second;
                    }
                    else
                    {
                        throw EParse("CPD type \"" + CPD_type + "\" not supported.");
                    }
                }
            }
        }
        branch = branch->next;
    }

    throw EParse("Probability under top variable " + top_name + " not found.");
}

double ParserProbModelXML::QueryTable(const xmlNodePtr node,
                                      const std::map<std::string, std::pair<Index, Index> > t0_deps, 
                                      const std::map<std::string, std::pair<Index, Index> > t1_deps)
{
    xmlNodePtr data_node = FindChild(node, "Values");
    if(data_node == NULL)
        throw EParse("Variable " + GetVariableName(node) + " has an empty CPD");
    const vector<double>* data = ParseArray(data_node);

    if(data->size() == 1) //typically the case for rewards in ADDs
        return data->at(0);

    xmlNodePtr current = xmlLastElementChild(FindChild(node, "Variables"));
    vector<Index> factor_values;
    vector<size_t> factor_sizes;
    map<string, pair<Index, Index> >::const_iterator it;
        
    for(Index i = 0; current != NULL;)
    {
        if(current->type == XML_ELEMENT_NODE)
        {
            int timeslice = GetVariableTimeslice(current);
            string name = GetVariableName(current);
            if(timeslice == 0)
            {
                it = t0_deps.find(name);
            }
            else
            {
                it = t1_deps.find(name);
            }
            if(it == t0_deps.end() || it == t1_deps.end())
            {
              throw EParse("Requested variable \"" + name + "\" which is not present in the dependencies of this CPT.");
            }
            factor_values.push_back(it->second.first);
            factor_sizes.push_back(it->second.second);
            i++;
        }
        current = current->prev;
    }

    Index jIdx = IndexTools::IndividualToJointIndices(factor_values, factor_sizes);

    if(jIdx < data->size())
    {
      return data->at(jIdx);
    }
    else
      throw EParse("CPD indexed value out of bounds.");
}

const vector<double>* ParserProbModelXML::ParseArray(const xmlNodePtr node)
{
    if(_m_arrayCache.count(node) == 0)
    {
      //need to create new array
      vector<double>* p = new vector<double>();
      stringstream data;
      data << (char*) xmlNodeListGetString(GetDoc(), node->xmlChildrenNode, 1);

      double d;
      while(!data.eof())
      {
        data >> skipws >> d;
        p->push_back(d);
        ws(data);
      }
      
      _m_arrayCache[node] = p;
    }

    return _m_arrayCache[node];
}

void ParserProbModelXML::InitializeLRFs()
{
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Variables/Variable[@role='utility']";
    xmlXPathObjectPtr reward_nodes = GetNodesMatchingExpression(xpath);

    if(reward_nodes == NULL){
        throw EParse("No reward variables found in the model.");
    }

    xmlNodeSetPtr reward_nodeset = reward_nodes->nodesetval;
    for(int i = 0; i < reward_nodeset->nodeNr; i++)
    {
        string name = GetVariableName(reward_nodeset->nodeTab[i]);
        pair<elm_type, Index> element = make_pair(REWARD, i);
        _m_parsedElements[name] = element;
    }

    _m_fDecPOMDP->SetNrLRFs(reward_nodeset->nodeNr);

    xmlXPathFreeObject (reward_nodes);
}

void ParserProbModelXML::ReadLRFs()
{
    _m_fDecPOMDP->InitializeInstantiationInformation();

    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/Potentials/Potential[@role='utility']";
    xmlXPathObjectPtr reward_potentials = GetNodesMatchingExpression(xpath);

    if(reward_potentials == NULL){
        throw EParse("No reward potentials found in the model.");
    }

    xmlNodeSetPtr potentials_nodeset = reward_potentials->nodesetval;
    for(int i = 0; i < potentials_nodeset->nodeNr; i++)
    {
        xmlXPathObjectPtr utility_var_node = GetNodesMatchingExpression((xmlChar*) "UtilityVariable",potentials_nodeset->nodeTab[i]);
        if(utility_var_node == NULL)
            throw EParse("Found a reward table node with no codomain.");
            
        string utility_var_name = GetVariableName(utility_var_node->nodesetval->nodeTab[0]);

        std::pair<ParserProbModelXML::elm_type, Index> elm;
        elm = GetParsedElement(utility_var_name);
        if(elm.first != REWARD)
            throw EParse("Found a reward table defined over non-reward variable \"" + utility_var_name + "\".");
        Index LRFix = elm.second; //LRF index

        xmlXPathObjectPtr vars = GetNodesMatchingExpression((xmlChar*) "Variables/Variable",potentials_nodeset->nodeTab[i]);
        xmlNodeSetPtr var_nodeset = vars->nodesetval;

        Scope X, A, Y, O;
        vector<Index> Xs, As, Ys, Os;
        vector<vector<Index>* > ptr_values;
        vector<size_t> factor_sizes;
        vector<string> factor_names;
        vector<bool> factor_times;
        int nValues = 1;
        for(int j = var_nodeset->nodeNr-1; j >= 0; j--) //have to store everything bottom-up
        {
            size_t size;
            xmlNodePtr node = var_nodeset->nodeTab[j];
            string var_name = GetVariableName(node);
            elm = GetParsedElement(var_name);
            switch(elm.first)
            {
                case STATE:
                    {
                        int timeslice = GetVariableTimeslice(node);
                        size = _m_fDecPOMDP->GetNrValuesForFactor(elm.second);
                        if(timeslice == 0)
                        {
                            X.Insert(elm.second);
                            ptr_values.push_back(&Xs);
                            factor_times.push_back(0);
                        }
                        else
                        {
                            Y.Insert(elm.second);
                            ptr_values.push_back(&Ys);
                            factor_times.push_back(1);
                        }
                    }
                    break;
                case ACTION:
                    if(!_m_asynchronousModel){
                        size = _m_fDecPOMDP->GetNrActions(elm.second);
                    }else{
                        size = _m_ASizes[elm.second];
                    }
                    A.Insert(elm.second);
                    ptr_values.push_back(&As);
                    factor_times.push_back(0);
                    break;
                case OBSERVATION:
                    if(!_m_asynchronousModel)
                        size = _m_fDecPOMDP->GetNrObservations(elm.second);
                    else
                        size = _m_fDecPOMDP->GetNrObservations(0);
                    O.Insert(elm.second);
                    ptr_values.push_back(&Os);
                    factor_times.push_back(1);
                    break;
                default:
                    throw EParse("Reward \"" + utility_var_name + "\" depends on unrecognized factors.");
            }
            factor_names.push_back(var_name);
            factor_sizes.push_back(size);
            nValues *=size;
        }
        vector<Index> factor_values(factor_names.size(), 0);        

        const Scope& sfSC = _m_fDecPOMDP->GetStateFactorScopeForLRF(LRFix);
        const Scope& agSC = _m_fDecPOMDP->GetAgentScopeForLRF(LRFix);
        string sf_descr = sfSC.SoftPrint();
        string ag_descr = agSC.SoftPrint();
        size_t nrXIs = _m_fDecPOMDP->GetNrXIs(LRFix);
        size_t nrAIs = _m_fDecPOMDP->GetNrAIs(LRFix);
        
        RewardModelMapping* RMe = 
            new RewardModelMapping(nrXIs, nrAIs, sf_descr, ag_descr);
        _m_fDecPOMDP->SetRM(LRFix, RMe);

        map<string, pair<Index, Index> > t0_deps;
        map<string, pair<Index, Index> > t1_deps;
        for(int j = 0; j < nValues; j++)
        {
            for(size_t k = 0; k < factor_names.size(); k++)
            {
              if(factor_times[k] == 0)
                t0_deps[factor_names[k]] = make_pair(factor_values[k], factor_sizes[k]);
              else
                t1_deps[factor_names[k]] = make_pair(factor_values[k], factor_sizes[k]);
            }

            double r = GetPotential(utility_var_name, t0_deps, t1_deps, true);

            for(size_t k = 0; k < ptr_values.size(); k++)
            {
                ptr_values[k]->push_back(factor_values[k]);
            }

            if(!_m_asynchronousModel)
                _m_fDecPOMDP->SetRewardForLRF(LRFix, X, Xs, A, As, Y, Ys, O, Os, r);
            else{
                Scope jaScope;
                jaScope.Insert(0);
                
                if(As.size() > 0){
                    ///now we have to go over all joint actions which map to A, As...
                    vector<Index> AsAll(_m_ASizes.size(), 0);
                    vector<size_t> r_AsAll = _m_ASizes;
                    for(size_t i = 0; i < A.size(); i++){
                        r_AsAll[A[i]] = 1; ///this prevents Increment from going over the variables already in the scope.
                        AsAll[A[i]] = As[i];
                    }
                    do{
                       Index jaI = IndexTools::IndividualToJointIndices(AsAll,_m_ASizes);
                        _m_fDecPOMDP->SetRewardForLRF(LRFix, X, Xs, jaScope, vector<Index>(1,jaI), Y, Ys, O, Os, r);
                    }while(! IndexTools::Increment( AsAll, r_AsAll ));
                }
                else
                {
                    _m_fDecPOMDP->SetRewardForLRF(LRFix, X, Xs, Scope(), vector<Index>(), Y, Ys, O, Os, r);
                }
            }
                
            Xs.clear();
            As.clear();
            Ys.clear();
            Os.clear();
            if(DEBUG_PARSE)
            {
                cout << "X: " << X.SoftPrint() << "->";
                for(size_t k = 0; k < Xs.size(); k++)
                    cout << Xs[k] << " ";
                cout << endl << "Y: " << Y.SoftPrint() << "->";
                for(size_t k = 0; k < Ys.size(); k++)
                    cout << Ys[k] << " ";
                cout << endl << "A: " << A.SoftPrint() << "->";
                for(size_t k = 0; k < As.size(); k++)
                    cout << As[k] << " ";
                cout << endl << "O: " << O.SoftPrint() << "->";
                for(size_t k = 0; k < Os.size(); k++)
                    cout << Os[k] << " ";
                cout << endl;

                cout << "LRF " << LRFix << " has sf scope " << sf_descr << " and ag scope " << ag_descr << " and for indexes ";
                    for(size_t k = 0; k < factor_values.size(); k++)
                      cout << factor_values[k] << " ";
                    cout << "it has reward " << r << endl;
            }
            IndexTools::Increment(factor_values, factor_sizes);
        }

        xmlXPathFreeObject (utility_var_node);
        xmlXPathFreeObject (vars);
    }

    xmlXPathFreeObject (reward_potentials);
}

void ParserProbModelXML::ReadProperties()
{
    xmlChar *xpath = (xmlChar*) "/ProbModelXML/ProbNet/AdditionalProperties/Property";
    xmlXPathObjectPtr properties = GetNodesMatchingExpression(xpath);
    
    if(properties == NULL) return; ///Properties are optional.
    
    xmlNodeSetPtr properties_nodeset = properties->nodesetval;
    for(int i = 0; i < properties_nodeset->nodeNr; i++)
    {
        string property_name = GetVariableName(properties->nodesetval->nodeTab[i]);
        if(property_name == "EventDriven"){
            string value = (char*) xmlGetProp(properties->nodesetval->nodeTab[i], (xmlChar*) "value");
            _m_asynchronousModel = (value != "0");
        }else{
            //Additional properties go here.
        }
    }
        
    xmlXPathFreeObject (properties);
}

void ParserProbModelXML::ConvertToLocalSFActionScope(Index sfI, Index aI, vector<Index>& AValues, Scope& AScope)
{
    try{
        AScope = _m_AScopes_Y.at(sfI);
    }
    catch(E& e)
    {
        e.Print();
    }
        
    vector<Index> allAValues = IndexTools::JointToIndividualIndices(aI, _m_ASizes);
    AValues.clear();
    for(size_t i = 0; i < AScope.size(); i++)
        AValues.push_back(allAValues.at(AScope[i]));
}

void ParserProbModelXML::ConvertToLocalObsActionScope(Index oI, Index aI, vector<Index>& AValues, Scope& AScope)
{
    try{
        AScope = _m_AScopes_O.at(oI);
    }
    catch(E& e)
    {
        e.Print();
    }
        
    vector<Index> allAValues = IndexTools::JointToIndividualIndices(aI, _m_ASizes);
    AValues.clear();
    for(size_t i = 0; i < AScope.size(); i++)
        AValues.push_back(allAValues.at(AScope[i]));
}

void ParserProbModelXML::AddToLocalSFActionScope(Index sfI, Scope AScope)
{
    while(_m_AScopes_Y.size() <= sfI)
        _m_AScopes_Y.push_back(Scope());
    
    _m_AScopes_Y[sfI].Insert(AScope);
}

void ParserProbModelXML::AddToLocalObsActionScope(Index oI, Scope AScope)
{
    while(_m_AScopes_O.size() <= oI)
        _m_AScopes_O.push_back(Scope());
    
    _m_AScopes_O[oI].Insert(AScope);
}

void ParserProbModelXML::AddToLocalLRFActionScope(Index lrfI, Scope AScope)
{
    while(_m_AScopes_LRF.size() <= lrfI)
        _m_AScopes_LRF.push_back(Scope());
    
    _m_AScopes_LRF[lrfI].Insert(AScope);
}
#endif

//Main program
void ParserProbModelXML::Parse()
{   
#ifdef HAVE_LIBXML2
    xmlInitParser(); 
    //Parse
    try{
        InitializeXPath();
        ReadProperties();
        ReadAgents();
        ReadStates();
        ReadActions();
        ReadObservations();
        InitializeLRFs();
        boost::function<double (Index o,
                                Index oVal,
                                const std::vector< Index>& Xs,
                                const std::vector< Index>& As,
                                const std::vector< Index>& Ys,
                                const std::vector< Index>& Os) > f
          = boost::bind(ComputeObservationProb, _1,_2,_3,_4,_5,_6, this, _m_fDecPOMDP);
        _m_fDecPOMDP->SetEventObservability(_m_asynchronousModel);
        _m_fDecPOMDP->Initialize2DBN(boost::bind(SetScopes, this, _m_fDecPOMDP),
                                     boost::bind(ComputeTransitionProb, _1, _2, _3, _4, _5, this, _m_fDecPOMDP),
                                     f);
        SetISD();
        ReadLRFs();

        xmlXPathFreeContext(GetContext());

      if(DEBUG_PARSE) cout << ">>>Parsing succeeded\n";
    }    
    catch(E& e)
    {
        e.Print();
    }
    xmlCleanupParser();
    if(DEBUG_PARSE)  cout << "-------------------------\n";
#else

    throw(E("ParserProbModelXML: not compiled with libxml2 support"));
#endif

    return;
}
