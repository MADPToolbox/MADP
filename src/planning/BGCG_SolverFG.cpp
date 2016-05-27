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

#include "BGCG_SolverFG.h"
#include <limits>
#include "FG_SolverMaxPlus.h"
#include "FG_SolverNDP.h"
#include "maxplus.h"

using namespace std;

//Default constructor
BGCG_SolverFG::BGCG_SolverFG(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,  
            BG_FactorGraphCreator::BGFactorGraph_t type,  
            FG_Solver::FG_Solver_t FGSt,
            size_t nrSolutions,
            int verbosity,
            double deadlineInSeconds,
            bool exploitSparseness
    ) :
    BGCG_Solver(bgcg,  nrSolutions),
    _m_FGt(type),
    _m_FGSt(FGSt),
    _m_verbosity(verbosity),
    _m_fgc(0),
    _m_deadlineInSeconds(deadlineInSeconds)
{
    //cout << "creating BGCG_SolverFG at verbosity "<<verbosity<<endl;
    _m_fgc = new BG_FactorGraphCreator( bgcg, type, verbosity-4, exploitSparseness );
    _m_fgc->CreateFG();
    //the default options:
    // max-plus parameters
    _m_damping = 0.05;
    _m_nrRestarts = 1;
    _m_maxIter = 1000;
    _m_updateType=std::string("PARALL");
    
    //do not create the FG_Solver yet, first let the user modify the options
}
//Copy constructor.    
BGCG_SolverFG::BGCG_SolverFG(const BGCG_SolverFG& o) :
    BGCG_Solver(o)
{
    throw (E("BGCG_SolverFG::<COPY CTOR>  not implemented - not clear if that would be useful?"));
}
//Destructor
BGCG_SolverFG::~BGCG_SolverFG()
{
    if(_m_fgc != 0)
        delete _m_fgc;

}
//Copy assignment operator
BGCG_SolverFG& BGCG_SolverFG::operator= (const BGCG_SolverFG& o)
{
    throw (E("BGCG_SolverFG::<ASSIGNMENT OPERATOR>  not implemented - not clear if that would be useful?"));
    /*if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;*/
}

double
BGCG_SolverFG::Solve()
{
    const libDAI::FactorGraph* f = _m_fgc->GetFG();
    //std::list< libDAI::MADP_util::valConf >* bestConfs;
    FG_Solver* fgs=0;

    switch(_m_FGSt)
    {
        case (FG_Solver::FGSt_NDP):
            fgs = new FG_SolverNDP (*f,  _m_verbosity-1, GetNrDesiredSolutions(), _m_deadlineInSeconds );
            break;
        case (FG_Solver::FGSt_MaxPlus):
            fgs = new FG_SolverMaxPlus (
                *f, 
                //maxplus parameters:  
                _m_maxIter, _m_updateType, _m_verbosity-1, _m_damping, 
                GetNrDesiredSolutions(), _m_nrRestarts, _m_deadlineInSeconds);
            break;
    default:
        throw(E("BGCG_SolverFG::Solve() unhandled solver type"));
    }
    //don't forget to SOLVE IT!
    fgs->Solve();
    std::list< libDAI::MADP_util::valConf >& bestConfs = fgs->GetBestConfigurations();

    double v=-DBL_MAX;
    switch(_m_fgc->GetBGFactorGraph_t() )
    {
        //these can use the same post-processing it seems:
        case (BG_FactorGraphCreator::AgentTypeIndependence):
        case (BG_FactorGraphCreator::TypeIndependence):
            v = ProcessFoundConfigurations_ATI(bestConfs);
            break;
        case (BG_FactorGraphCreator::AgentIndependence):
            v = ProcessFoundConfigurations_AI(bestConfs);
            break;
    default:
        throw(E("BGCG_SolverFG::Solve() unhandled factor graph type"));
    }

    delete fgs;

    return v;
}

double
BGCG_SolverFG::ProcessFoundConfigurations_ATI(
        std::list< libDAI::MADP_util::valConf >& bestConfs)
{
    //Create the BG policy as computed by MaxPlus...
    size_t nrConfsFound = bestConfs.size();
    double value = 0.0;
    double maxV = std::numeric_limits<double>::min();
    for(Index bcI=0; bcI < nrConfsFound; bcI++)
    {
        libDAI::MADP_util::valConf& vc = bestConfs.front();
        std::vector<size_t> & config = vc.second;
        value = vc.first;
        maxV = std::max(value, maxV);
    
        //translate found configuration to jpol
        JPPV_sharedPtr temp = boost::dynamic_pointer_cast<JointPolicyPureVector>( GetNewJpol() );
        JointPolicyPureVector jpolBG( *temp);
//        delete temp;
        //JointPolicyPureVector jpolBG( _m_solution.GetJointPolicyPureVector() );
        for(Index agI = 0; agI < _m_bgcg->GetNrAgents(); agI++)
            for(Index tI = 0; tI < _m_bgcg->GetNrTypes(agI); tI++)
            {
                Index varIndex = _m_fgc->GetVariableIndexForAgentType(agI, tI);
                Index bestAction = config[varIndex];
                jpolBG.SetAction(agI, tI, bestAction);
            }
        //store the solution
        AddSolution(*(new JointPolicyPureVector(jpolBG)),
                    value);

        if(_m_verbosity >= 8)
        {
            cout << "adding jpol# "<<bcI<<" constructed from config: "<< endl;
            cout << jpolBG.SoftPrint()<<endl;
        }   

        bestConfs.pop_front();
    }
    return(maxV);
}

double
BGCG_SolverFG::ProcessFoundConfigurations_AI(
        std::list< libDAI::MADP_util::valConf >& bestConfs)
{
    //Create the BG policy as computed by MaxPlus...
    size_t nrConfsFound = bestConfs.size();
    double value = 0.0;
    double maxV = std::numeric_limits<double>::min();
    for(Index bcI=0; bcI < nrConfsFound; bcI++)
    {
        libDAI::MADP_util::valConf& vc = bestConfs.front();
        std::vector<size_t> & config = vc.second;
        value = vc.first;
        maxV = std::max(value, maxV);
    
        //translate found configuration to jpol
        JPPV_sharedPtr temp = boost::dynamic_pointer_cast<JointPolicyPureVector>( GetNewJpol() );
        JointPolicyPureVector jpolBG( *temp);
        std::vector<PolicyPureVector*>& indPols = jpolBG.GetIndividualPolicies();
        for(Index agI = 0; agI < _m_bgcg->GetNrAgents(); agI++)
        {
            Index varIndex = agI; 
            Index bestPolicy = config[varIndex];
            indPols.at(agI)->SetIndex( bestPolicy );
        }
        //store the solution
        AddSolution(*(new JointPolicyPureVector(jpolBG)),
                    value);

        if(_m_verbosity >= 3)
        {
            cout << "adding jpol# "<<bcI<<" constructed from config: "<< endl;
            cout << jpolBG.SoftPrint()<<endl;
        }   

        bestConfs.pop_front();
    }
    return(maxV);
}

/* we don't need this: configuration is the same as for ATI!!
double
BGCG_SolverFG::ProcessFoundConfigurations_TI(
        std::list< libDAI::MADP_util::valConf >& bestConfs)
{
    //Create the BG policy as computed by MaxPlus...
    size_t nrConfsFound = bestConfs.size();
    double value = 0.0;
    double maxV = std::numeric_limits<double>::min();
    for(Index bcI=0; bcI < nrConfsFound; bcI++)
    {
        libDAI::MADP_util::valConf& vc = bestConfs.front();
        std::vector<size_t> & config = vc.second;
        value = vc.first;
        maxV = std::max(value, maxV);
    
        //translate found configuration to jpol
        JointPolicyPureVector *temp=GetNewJpol();
        JointPolicyPureVector jpolBG( *temp);
        delete temp;
        //JointPolicyPureVector jpolBG( _m_solution.GetJointPolicyPureVector() );
        for(Index agI = 0; agI < _m_bgcg->GetNrAgents(); agI++)
            for(Index tI = 0; tI < _m_bgcg->GetNrTypes(agI); tI++)
            {
                Index varIndex = _m_fgc->GetVariableIndexForAgentType(agI, tI);
                Index bestAction = config[varIndex];
                jpolBG.SetAction(agI, tI, bestAction);
            }
        //store the solution
        AddSolution(*(new JointPolicyPureVector(jpolBG)),
                    value);

        if(_m_verbosity >= 3)
        {
            cout << "adding jpol# "<<bcI<<" constructed from config: "<< endl;
            cout << jpolBG.SoftPrint()<<endl;
        }   

        bestConfs.pop_front();
    }
    return(maxV);
}
*/
