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

#include "BG_FactorGraphCreator.h"
#include "BayesianGameCollaborativeGraphical.h"
#include "BayesianGameIdenticalPayoff.h"
#include <float.h>
#include <iostream>
#include <fstream>
#include "var.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameCollaborativeGraphical.h"

using namespace std;

//Default constructor
BG_FactorGraphCreator::BG_FactorGraphCreator(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bg,                
            BGFactorGraph_t type,
            int verbosity,
            bool exploitSparseness
    )
    :
        _m_bg(bg),
        _m_FGt(type),
        _m_FG(0),
        _m_verbosity(verbosity),
        _m_exploitSparse(exploitSparseness)
{
    //cout << "creating BG_FactorGraphCreator at verbosity "<<verbosity<<endl;
}

BG_FactorGraphCreator::~BG_FactorGraphCreator()
{
    delete _m_FG;
}

const libDAI::FactorGraph*  
BG_FactorGraphCreator::CreateFG()
{
    switch(_m_FGt)
    {
        case AgentTypeIndependence:
            return CreateFG_AgentTypeIndepence();
        case AgentIndependence:
            return CreateFG_AgentIndepence();
        case TypeIndependence:
            return CreateFG_TypeIndepence();
    }
    return 0;
}

//will construct a FG with both agent and type independence
const libDAI::FactorGraph*  
BG_FactorGraphCreator::CreateFG_AgentTypeIndepence()
{
    Construct_AgentTypePair_Variables();
    Construct_LocalJointType_Factors();
    _m_FG = new libDAI::FactorGraph(_m_facs); 
    return _m_FG;
}

const libDAI::FactorGraph*  
BG_FactorGraphCreator::CreateFG_AgentIndepence()
{
    Construct_AgentBGPolicy_Variables();
    Construct_LocalPayoff_Factors();
    _m_FG = new libDAI::FactorGraph(_m_facs); 
    return _m_FG;
}
const libDAI::FactorGraph*  BG_FactorGraphCreator::CreateFG_TypeIndepence()
{
    Construct_AgentTypePair_Variables();
    Construct_JointType_Factors();
    _m_FG = new libDAI::FactorGraph(_m_facs); 
    return _m_FG;
}

void BG_FactorGraphCreator::Construct_AgentTypePair_Variables()
{    
    _m_var_indices.resize( _m_bg->GetNrAgents() );
    size_t nrATs=0;
    vector<size_t> nrTypes(_m_bg->GetNrAgents() );
    for(Index agI = 0; agI < _m_bg->GetNrAgents(); agI++)
    {
        nrTypes[agI] =  _m_bg->GetNrTypes(agI);
        if(_m_verbosity >= 2)
            cout << "Agent "<<agI<<" has " << nrTypes[agI] << " types."<<endl;
        nrATs += nrTypes[agI];
        _m_var_indices[agI].reserve( _m_bg->GetNrTypes(agI) );
    }
    if(_m_verbosity >= 1)
        cout << "FactorGraph has "<<nrATs<<" variables (agent-types)"<<endl;
   
    //reserve memory 
    for(Index agI = 0; agI < _m_bg->GetNrAgents(); agI++)
        _m_var_indices[agI].reserve( nrTypes[agI] );
    _m_vars.reserve(nrATs);

    Index atI = 0;
    for(Index agI = 0; agI < _m_bg->GetNrAgents(); agI++)
        for(Index tI = 0; tI < _m_bg->GetNrTypes(agI); tI++)
        {
            //Note variables for higher agents have higher indices!
            libDAI::Var v(atI, _m_bg->GetNrActions(agI));
            _m_var_indices[agI].push_back(atI);
            _m_vars.push_back(v);
            atI++;
        }

}

void BG_FactorGraphCreator::Construct_AgentBGPolicy_Variables()
{   
    _m_vars.reserve( _m_bg->GetNrAgents() );

    for(Index agI = 0; agI < _m_bg->GetNrAgents(); agI++)
    {
        libDAI::Var v(agI, CastLIndexToIndex(_m_bg->GetNrPolicies(agI)));
        _m_vars.push_back(v);
    }
}

void BG_FactorGraphCreator::Construct_LocalJointType_Factors()
{
    size_t nrLRFs = _m_bg->GetNrLRFs();
    //compute nrFacs:
    size_t nrFacs =  0;
    for(Index e=0; e < nrLRFs; e++)
    {
        const BayesianGameIdenticalPayoff * _m_bg_e = _m_bg->GetBGIPforLRF (e);
        size_t nrJT_e = _m_bg_e->GetNrJointTypes();
        nrFacs += nrJT_e;
    }
    size_t worstCaseFacs = nrFacs;
    _m_facs.reserve(nrFacs);


    for(Index e=0; e < nrLRFs; e++)
    {
        //construct the factors for LRF component e.
        const BayesianGameIdenticalPayoff * _m_bg_e = _m_bg->GetBGIPforLRF (e);
        Scope agentScope = _m_bg->GetScope(e);

        const size_t nrJA_e =  _m_bg_e->GetNrJointActions();
        size_t nrJT_e = _m_bg_e->GetNrJointTypes();
        for(Index jtI_e=0; jtI_e < _m_bg_e->GetNrJointTypes(); jtI_e++)
        {
            double p_jtI_e =  _m_bg_e->GetProbability(jtI_e);
            if(_m_exploitSparse && EqualProbability(p_jtI_e, 0.0))
            {
                if(_m_verbosity > 4)
                {
                    cout << "Removed 0-prob factor for joint type jtI_e="
                         << jtI_e << endl;
                }
                nrJT_e--;
                nrFacs--;
                continue;
            }
            vector<Index> indTypes_e =
                _m_bg_e->JointToIndividualTypeIndices(jtI_e);

            if(_m_verbosity > 5)
                cout << "Adding factor for joint type jtI_e="<<jtI_e<<
                    " ( indtypes_e="<<SoftPrintVector(indTypes_e) << " )"<<endl;

            //compute the agent-type (i.e., factor) indices atI, get the
            //relevant variables and put them in a VarSet
            libDAI::VarSet vs;
            for(Index agI_e=0; agI_e < _m_bg_e->GetNrAgents(); agI_e++)
            {
                //translate agI_e to global agent index:
                Index agI = agentScope.at(agI_e);
                Index atI = _m_var_indices[agI][indTypes_e[agI_e]];
                vs = vs | _m_vars[atI];
                if(_m_verbosity > 5)
                    cout << "agent "<<agI<<
                        " contributes through variable atI="<<atI<<endl;

            }
            libDAI::Real factor_vals[nrJA_e];
            for(Index jaI_e=0; jaI_e < nrJA_e; jaI_e++)
            {
                vector<Index> indAcs = 
                    _m_bg_e->JointToIndividualActionIndices(jaI_e);
                vector<size_t> indAcsST(indAcs.size());
                for(Index i=0; i<indAcs.size(); i++)  
                    indAcsST[i] = (size_t) indAcs[i];

                if(_m_verbosity > 5)
                    cout << "Computing joint factor index (JFI) for vs="<<vs
                        <<", (action)indices="<< SoftPrintVector(indAcs) <<endl;
                Index jointFactorIndex = 
                    libDAI::Factor::IndividualToJointFactorIndex(vs, indAcsST);
                
                if(_m_verbosity > 5)
                    cout << "JFI="<<jointFactorIndex<< " Utility "
                         << _m_bg_e->GetUtility(jtI_e, jaI_e) << " Prob "
                         << p_jtI_e << endl;

                double perturbation=(1e-6*rand())/RAND_MAX;
                factor_vals[jointFactorIndex] =
                    (_m_bg_e->GetUtility(jtI_e, jaI_e) + perturbation) *
                    _m_bg_e->GetProbability(jtI_e);
            }

            libDAI::Factor f(vs, factor_vals);

            if(_m_verbosity > 4)
            {
                cout << "Added factor f:"<< f << endl;
            }
            _m_facs.push_back(f);
        }// <- end for jtI_e
        if(_m_verbosity >= 2)
        {
            cout << "Edge "<<e<<" has " << nrJT_e
                 << " local joint types (i.e. factors)";
            if(_m_exploitSparse)
                cout << " reduced from " << _m_bg_e->GetNrJointTypes();
            cout << endl;
        }
        
    }// <- end for e
    if(_m_verbosity >= 1)
        cout << "FactorGraph has "<<nrFacs<<" factors (worstcase="<<worstCaseFacs<<")"<<endl;
}

void BG_FactorGraphCreator::Construct_JointType_Factors()
{
    const size_t nrJA =  _m_bg->GetNrJointActions();
    size_t nrFacs =  _m_bg->GetNrJointTypes() ;
    size_t worstCaseFacs = nrFacs;
    _m_facs.reserve(nrFacs);

    //factor values are copied by value (?) so this is not needed:
    //vector<double **> factor_vals( _m_bg->GetNrJointTypes() );
    for(Index jtI=0; jtI < _m_bg->GetNrJointTypes(); jtI++)
    {
        
        double p_jtI = _m_bg->GetProbability(jtI);
        if(_m_exploitSparse && EqualProbability(p_jtI, 0.0))
        {
            if(_m_verbosity > 4)
            {
                cout << "Removed 0-prob factor for joint type jtI="
                     << jtI << endl;
            }
            nrFacs--;
            continue;
        }
        
                
        vector<Index> indTypes = _m_bg->JointToIndividualTypeIndices(jtI);
        if(_m_verbosity > 4)
            cout << "Adding factor for joint type jtI="<<jtI<<" ( indtypes="<<
                SoftPrintVector(indTypes) << " )"<<endl;

        //compute the agent-type (i.e., factor) indices atI, get the
        //relevant variables and put them in a VarSet
        libDAI::VarSet vs;
        for(Index agI=0; agI < _m_bg->GetNrAgents(); agI++)
        {

            Index atI = _m_var_indices[agI][indTypes[agI]];
            vs = vs | _m_vars[atI];
            if(_m_verbosity > 5)
                cout << "agent "<<agI<<" contributes through variable atI="<<atI<<endl;

        }

        libDAI::Real factor_vals[nrJA];
        for(Index jaI=0; jaI < nrJA; jaI++)
        {
            vector<Index> indAcs = _m_bg->JointToIndividualActionIndices(jaI);
            vector<size_t> indAcsST(indAcs.size());
            for(Index i=0; i<indAcs.size(); i++)  
                indAcsST[i] = (size_t) indAcs[i];  
            if(_m_verbosity > 6)
                cout << "Computing joint factor index (JFI) for vs="<<vs<<", (action)indices="<<
                    SoftPrintVector(indAcs) <<endl;
            Index jointFactorIndex = 
                libDAI::Factor::IndividualToJointFactorIndex(vs, indAcsST);
            if(_m_verbosity > 6)
                cout << "JFI="<<jointFactorIndex<<endl;
            /* this doesn't work:
            // in our implementation (actions of) higher numbered agents are 
            // the least significant, but in libDAI higher numbered variables
            // (corresponding to higher numbered agents, see above) are the most
            // significant. Therefore this is done in reverse:
            Index rev_index = nrJA - jaI - 1;
            */
            double perturbation=PerturbationTerm(); //(1e-6*rand())/RAND_MAX;
            factor_vals[jointFactorIndex] =
                (_m_bg->GetUtility(jtI, jaI) + perturbation ) * p_jtI;
        }

        libDAI::Factor f(vs, factor_vals);
        if(_m_verbosity > 4)
        {
            cout << "Added factor f:"<< f << endl;
        }
        _m_facs.push_back(f);
    }
    if(_m_verbosity >= 1)
        cout << "FactorGraph has "<<nrFacs<<" factors (worstcase="<<worstCaseFacs<<")"<<endl;
}

void BG_FactorGraphCreator::Construct_LocalPayoff_Factors()
{
    size_t nrLRFs = _m_bg->GetNrLRFs();
    size_t nrFacs =  nrLRFs;

    if(_m_verbosity >= 1)
        cout << "FactorGraph has "<<nrFacs<<" factors"<<endl;
    _m_facs.reserve(nrFacs);

    for(Index e=0; e < nrLRFs; e++)
    {
#define SHOWMEWHYITCRASHES 0 
#if SHOWMEWHYITCRASHES
        cout << ">>>Starting edge "<< e << endl;
#endif
        //--now add the factor for payoff component e--
        //it is a function of the BG policies of each agent in this scope.
        //construct thos single factor for LRF component e.

        const BayesianGameIdenticalPayoff * _m_bg_e = _m_bg->GetBGIPforLRF (e);
        Scope agentScope = _m_bg->GetScope(e);
        size_t nrAg_e = _m_bg_e->GetNrAgents();

        //make sure this is made the right size!!!!!!!!
        //(otherwise memory gets overwritten etc. below, which make nasty debugging..)
        LIndex nrJPols = _m_bg_e->GetNrJointPolicies();
        size_t nr_st = CastLIndexToIndex(nrJPols);
#if SHOWMEWHYITCRASHES
        cout << "nrJPols="<<nrJPols<<endl;
        cout << "nr_st="<<nr_st<<endl;
#endif
        //libDAI::Real factor_vals[ nr_st ];
        libDAI::Prob factor_vals( nr_st );
#if SHOWMEWHYITCRASHES
        cout << "allocation of "<< nr_st << "reals succeeded." << endl;
        cout.flush();
#endif
        
        //get the variables relevant for thie factor and put them in a VarSet
        libDAI::VarSet vs;
        for(Index agI_e=0; agI_e < _m_bg_e->GetNrAgents(); agI_e++)
        {
            //translate agI_e to global agent index:
            Index agI = agentScope.at(agI_e);
            vs = vs | _m_vars[agI];
        }
       
        //so we loop over all the joint policy, determine their payoff, and put it in
        //the factor
        JointPolicyPureVector jpol_e( _m_bg_e, TYPE_INDEX);
        bool carryover = false;
        while(!carryover)
        {
            //determine jpols value
            double v = 0.0;
            for(Index jt = 0; jt < _m_bg_e->GetNrJointTypes(); ++jt)
            {                
                Index ja = jpol_e.GetJointActionIndex(jt);  
                v += _m_bg_e->GetProbability(jt) * _m_bg_e->GetUtility(jt, ja);
            }
            
            //compute the joint index to the factor
            //(indices to (values of) variables are done with 'size_t' in libDAI)
            vector<size_t> indPolIs(nrAg_e);
            const std::vector< PolicyPureVector * > & indPols = 
                jpol_e.GetIndividualPolicies();
            for(Index agI=0; agI < nrAg_e; agI++)
            {
                Index polI = CastLIndexToIndex(indPols.at(agI)->GetIndex());
                indPolIs.at(agI) = static_cast<size_t>( polI );
            }
            Index jointFactorIndex = 
                libDAI::Factor::IndividualToJointFactorIndex(vs, indPolIs);
            //set the value:
            factor_vals[jointFactorIndex] = v + PerturbationTerm();
            carryover = jpol_e.Increment();
        }
        libDAI::Factor f(vs, factor_vals);
        if(_m_verbosity > 4)
        {
            cout << "Added factor f:"<< f << endl;
        }
        _m_facs.push_back(f);


    }// <- end for e
}

void BG_FactorGraphCreator::Construct_Payoff_Factor()
{
    throw (E("BG_FactorGraphCreator::Construct_Payoff_Factor() is not (yet) implemented - it seems not useful?"));

    const size_t nrJA =  _m_bg->GetNrJointActions();
    size_t nrFacs =  _m_bg->GetNrJointTypes() ;
    _m_facs.reserve(nrFacs);

    if(_m_verbosity >= 1)
        cout << "FactorGraph has "<<nrFacs<<" factors"<<endl;
    //factor values are copied by value (?) so this is not needed:
    //vector<double **> factor_vals( _m_bg->GetNrJointTypes() );
    for(Index jtI=0; jtI < _m_bg->GetNrJointTypes(); jtI++)
    {
        vector<Index> indTypes = _m_bg->JointToIndividualTypeIndices(jtI);
        if(_m_verbosity > 4)
            cout << "Adding factor for joint type jtI="<<jtI<<" ( indtypes="<<
                SoftPrintVector(indTypes) << " )"<<endl;

        //compute the agent-type (i.e., factor) indices atI, get the
        //relevant variables and put them in a VarSet
        libDAI::VarSet vs;
        for(Index agI=0; agI < _m_bg->GetNrAgents(); agI++)
        {

            Index atI = _m_var_indices[agI][indTypes[agI]];
            vs = vs | _m_vars[atI];
            if(_m_verbosity > 5)
                cout << "agent "<<agI<<" contributes through variable atI="<<atI<<endl;

        }

        libDAI::Real factor_vals[nrJA];
        for(Index jaI=0; jaI < nrJA; jaI++)
        {
            vector<Index> indAcs = _m_bg->JointToIndividualActionIndices(jaI);
            vector<size_t> indAcsST(indAcs.size());
            for(Index i=0; i<indAcs.size(); i++)  indAcsST[i] = (size_t) indAcs[i];  
            if(_m_verbosity > 6)
                cout << "Computing joint factor index (JFI) for vs="<<vs<<", (action)indices="<<
                    SoftPrintVector(indAcs) <<endl;
            Index jointFactorIndex = libDAI::Factor::IndividualToJointFactorIndex(vs, indAcsST);
            if(_m_verbosity > 6)
                cout << "JFI="<<jointFactorIndex<<endl;
            /* this doesn't work:
            // in our implementation (actions of) higher numbered agents are 
            // the least significant, but in libDAI higher numbered variables
            // (corresponding to higher numbered agents, see above) are the most
            // significant. Therefore this is done in reverse:
            Index rev_index = nrJA - jaI - 1;
            */
            double perturbation=(1e-6*rand())/RAND_MAX;
            factor_vals[jointFactorIndex] =
                (_m_bg->GetUtility(jtI, jaI) + perturbation ) *
                _m_bg->GetProbability(jtI);
        }

        libDAI::Factor f(vs, factor_vals);
        if(_m_verbosity > 4)
        {
            cout << "Added factor f:"<< f << endl;
        }
        _m_facs.push_back(f);
    }
}

Index 
BG_FactorGraphCreator::GetVariableIndexForAgentType(Index agI, Index typeI) const
{   
    if(_m_FGt == AgentTypeIndependence || _m_FGt == TypeIndependence)
        return _m_var_indices.at(agI).at(typeI); 
    else
        //when only agent independence, varI=agI
        return(agI);
}
