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

#ifndef _PARSERPROBMODELXML_H_
#define _PARSERPROBMODELXML_H_ 1

/* the include directives */
#include <iostream>
#include <fstream>
#include "Globals.h"
#include "FactoredDecPOMDPDiscrete.h"
#include "EParse.h"
#include <string>
#include "ParserInterface.h"
#include "StringTools.h"
#include "IndexTools.h"
#include "RewardModelMapping.h"
#include "StateFactorDiscrete.h"
#include "FSDist_COF.h"
#include <map>
#include <vector>
#include "boost/bind.hpp"
#include <config.h>
#ifdef HAVE_LIBXML2
#include <libxml/xmlmemory.h>
#include <libxml/parser.h>
#include <libxml/xpath.h>
#endif

//General parsing debug informations
#ifndef DEBUG_PARSE
#define DEBUG_PARSE 0
#endif

/** ParserProbModelXML is a parser for factored Dec-POMDP models written in ProbModelXML.*/
class ParserProbModelXML :
    public ParserInterface
{   
    public:
        enum elm_type {STATE, ACTION, OBSERVATION, REWARD};
        
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        ParserProbModelXML(FactoredDecPOMDPDiscrete *problem=0);
        // Destructor.
        //~ParserProbModelXML();
        
        /**The function that starts the parsing.*/
        void Parse();
#ifdef HAVE_LIBXML2
        /** These methods are public since they need to be called by the
         * functions that will be passed onto the FactoredDecPOMDP.*/

        /**Given an xpath expression, this function simply returns a
         * pointer to respective set of results, or a null pointer if the
         * xpath is invalid (the caller must explicitly handle that case).*/
        const xmlXPathObjectPtr GetNodesMatchingExpression(const xmlChar *expr, 
                                                           const xmlNodePtr context_node = 0);

        /**This function simply extracts the "name" attribute from an element node.*/
        std::string GetVariableName(const xmlNodePtr node) const;

        /**This function extracts the "timeslice" attribute from an element node.*/
        int GetVariableTimeslice(const xmlNodePtr node) const;
        
        /**Given the name of an element node, this function recovers its type
         * (an elm_type defined above) and the respective index of its factor.
         * This is necessary since elements in ProbModelXML are always indexed
         * by their name.
         * This function assumes that all states, actions and observations have
         * been initialized.*/
        const std::pair<elm_type, Index> GetParsedElement(const std::string name) const;

        /**This does the opposite of GetParsedElement: given the element type and
         * index, it returns the name of that variable.*/
        std::string GetParsedElementName(const std::pair<elm_type, Index> elm) const;

        /**This function simply reads a string of numbers in a node into a vector of doubles.*/
        const std::vector<double>* ParseArray(const xmlNodePtr node);

        /**The GetPotential function returns the potential (i.e. probability or reward)
         * associated with a given named variable, given the values of its parents.
         * These dependencies are passed as maps of the following form:
         * <variable name | <variable value, size of its factor> >
         * Dependencies must be explicitly organized into timeslices, and rewards
         * must also be signaled externally, since their specification is a bit different
         * (although much of the parsing is the same).*/
        double GetPotential(const std::string var_name,
                            const std::map<std::string, std::pair<Index, Index> > t0_deps, 
                            const std::map<std::string, std::pair<Index, Index> > t1_deps,
                            const bool isUtility = false);
        
        /**In MADP it is assumed that each agent has
         * its own observation factor. In an asynchronous model there is
         * only one observation factor, but actions can still be factored;
         * then, the ParserProbModelXML is forced to produce fully-centralized model
         * with only one agent/action factor, even if there are multiple agents
         * in the DBN. Action Scopes/Names are maintained locally, in this case (see below).*/
        
        /**From a joint action index and a state factor index, extract the factored action values and respective agent scope.*/
        void ConvertToLocalSFActionScope(Index sfI, Index aI, std::vector<Index>& AValues, Scope& AScope);
        /**From a joint action index and an observation factor index, extract the factored action values and respective agent scope.*/
        void ConvertToLocalObsActionScope(Index oI, Index aI, std::vector<Index>& AValues, Scope& AScope);

        /**Public interfaces to add actions to state factor, observations, lrf scopes*/
        void AddToLocalSFActionScope(Index sfI, Scope AScope);
        void AddToLocalObsActionScope(Index oI, Scope AScope);
        void AddToLocalLRFActionScope(Index lrfI, Scope AScope);
        
        bool IsAsynchronous()
        {return _m_asynchronousModel;}
        
        /**Note that the resulting FactoredDecPOMDP, if it is asynchronous, is only factored in its state space.
         * Therefore, only the parser knows the actual number of actions for each agent*/
        size_t GetLocalNrActions(Index aI)
        {return _m_ASizes.at(aI);}
#endif
    private:
#ifdef HAVE_LIBXML2
        /**Parses all the "chance" elements at the first timeslice as state factors*/
        void ReadStates();

        /**Parses the agent team size and respective names*/
        void ReadAgents();

        /**Parses all the "decision" elements as actions and assigns them to agents*/
        void ReadActions();
        
        /**Reads all "chance" elements which link to actions at the second timeslice
         * as observations (this is how observations are specified in OpenMarkov).*/
        void ReadObservations();

        /**Fetches the potentials of all state factor nodes at the first timeslice and
         * loads them into the model ISD.*/
        void SetISD();

        /**This function finds out how many factors there are for the reward function,
         * and initializes their scopes.*/
        void InitializeLRFs();

        /**After the scopes of the LRFs have been found, this function reads all "utility"
         * potentials into the reward model.*/
        void ReadLRFs();

        /**Reads optional ProbNet properties set by the user.*/
        void ReadProperties();
        
        /**This opens the problem file of the FactoredDecPOMDP with libxml2,
         * ready to be queried with xpath.*/
        void InitializeXPath();

        /**Given an absolute xpath to a variable, this tries to match its children with the relative
         * xpath in 'expr'. This is necessary since libxml2 doesn't handle relative xpaths on its own.*/
        const xmlXPathObjectPtr GetChildrenMatchingExpression(const std::string var_path, const xmlChar *expr) const;

        /**This just loads any comments in a variable as plain text.*/
        std::string GetVariableComment(const xmlNodePtr);

        /**Given a pointer to a table element, and the parent factor names, values, and sizes, this
         * gets the respective probability or reward.*/
        double QueryTable(const xmlNodePtr node,
                          const std::map<std::string, std::pair<Index, Index> > t0_deps, 
                          const std::map<std::string, std::pair<Index, Index> > t1_deps);

        /**Given a pointer to a 'Tree/ADD' element, and the parent factor names, values, and sizes, this
         * gets the respective probability or reward.
         * QueryADD can be recursive (if nodes are themselves ADDs), so we keep track of the root node.
         */
        double QueryADD(const xmlNodePtr root_node, 
                        const xmlNodePtr current_node,
                        const std::map<std::string, std::pair<Index, Index> > t0_deps, 
                        const std::map<std::string, std::pair<Index, Index> > t1_deps);
        xmlNodePtr FindChild(const xmlNodePtr node, const std::string child) const;

        /**xpath utility function. Gets the context.*/
        const xmlXPathContextPtr GetContext() const {return _m_context;}
        /**libxml2 utility function. Gets the document pointer.*/
        const xmlDocPtr GetDoc() const {return _m_doc;}
        /**xpath utility function. Sets the query context to a given node.*/
        void SetContext(const xmlNodePtr context_node)
        {((xmlXPathContext *) _m_context)->node = context_node;}
        /**xpath utility function. Resets the query context to the doc root node.*/
        void ResetContext()
        {((xmlXPathContext *) _m_context)->node = xmlDocGetRootElement(_m_doc);}
        /**libxml2 utility function. Sets the document pointer.*/
        void SetDoc(const xmlDocPtr doc){_m_doc = doc;}
        
        /**Context pointer needed for xpath queries.*/
        xmlXPathContextPtr _m_context;
        /**Libxml2 parsed document object*/
        xmlDocPtr _m_doc;

        /**Storage for parsed elements, so that they can be accessed by their name
         * when reading links and probabilities.
         * From here we can figure out the respective factor in the model.*/
        std::map<std::string, std::pair<elm_type, Index> > _m_parsedElements;
        
        std::map<xmlNodePtr, std::vector<double>* > _m_arrayCache;
#endif

        /**Whether or not agents have been found and initialized.
         * If no agent names are present in the file, the parser infers
         * the team size by the number of action nodes and addresses
         * everyone by their indexes.*/
        bool _m_agentsInitialized;
        
        FactoredDecPOMDPDiscrete* _m_fDecPOMDP;
        
        /**The following variables are used for asynchronous models with
         * factored actions.*/
        bool _m_asynchronousModel; ///whether or not this is an asynchronous model
 
        /**Locally maintained action scopes for state factors, observations and lrfs.*/
        std::vector<Scope> _m_AScopes_Y;
        std::vector<Scope> _m_AScopes_O;
        std::vector<Scope> _m_AScopes_LRF;
        /**Locally maintained action names*/
        std::vector<std::vector<std::string> > _m_ANames;
        /**Locally maintained action sizes*/
        std::vector<std::size_t> _m_ASizes;
};


#endif /* !_PARSERDECPOMDPDISCRETE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
