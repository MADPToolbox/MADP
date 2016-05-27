/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "BGCG_SolverMaxPlus.h"

#include <float.h>
#include <iostream>
#include <fstream>
#include "var.h"
#include "maxplus.h"

#include "BayesianGameCollaborativeGraphical.h"
#include "BayesianGameIdenticalPayoff.h"
#include "JointPolicyPureVector.h"

using namespace std;

//Default constructor
BGCG_SolverMaxPlus::BGCG_SolverMaxPlus(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,                
            size_t maxiter,
            std::string updateType,
            int verbosity,
            double damping,
            size_t nrSolutions,
            size_t nrRestarts)
    :
        BGCG_Solver(bgcg,  nrSolutions),
        MaxPlusSolverForBGs(maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts)
{
}

double BGCG_SolverMaxPlus::Solve()
{
    if(_m_verbosity >= 1)
        cout << "BGCG_SolverMaxPlus::Solve(), starting..." << endl;
    BGCG_constPtr bgcg = GetBGCG();

    //an assignment of variables (v3, v8, v14) corresponds to an assignment
    //of joint actions (a1, a2, a3) for a particular joint type.
    //I.e., each (the action taken for eah) (agent,type)-pair is a variable 
    //of the factor graph.
    //let's create these variables
    
    // var_indices[agI][tI] stores the (agent,type)-pair index atI
    vector< vector<Index> > var_indices;    
    // vars stores the variables. vars[atI]
    vector< libDAI::Var > vars;
    Construct_AgentTypePair_Variables(bgcg, var_indices, vars, _m_verbosity);

    //each joint type corresponds to a factor... 
    //let's create them
    vector<libDAI::Factor> facs;
    Construct_JointType_Factors_CGBG(bgcg, var_indices, vars, facs, _m_verbosity);

    //and finally you construct the FactorGraph from that:
    libDAI::FactorGraph fg(facs);

    //size_t  maxiter = 1000;
    //size_t  verb = 2;
    libDAI::Properties props;
    props.Set("maxiter",_m_maxiter);
    size_t verb = (size_t) _m_verbosity;
    props.Set("verbose", verb);
    props.Set("updates",_m_updateType);
    props.Set("damping",_m_damping);
    double  tol = 1e-2;
    props.Set("tol",tol);    
    libDAI::MaxPlus mp (fg, props, _m_nrSolutions);
    for(Index restI=0; restI < _m_nrRestarts; restI++)
    {
        mp.init();
        if(GetWriteAnyTimeResults())
            mp.SetAnyTimeResults(true,
                                 GetResultsOFStream(),
                                 GetTimingsOFStream());
        double value = mp.run();
        if(_m_verbosity >= 1)
            cout << "MP run found solution with value="<<value<<endl;
    }
    //Create the BG policy as computed by MaxPlus...
    //const std::vector<size_t> & config = mp.GetBestConfiguration();
    std::list< libDAI::MADP_util::valConf >& bestConfs = mp.GetBestConfigurations();
    size_t nrConfsFound = bestConfs.size();
    if(_m_verbosity >= 1)
        cout << "MP found "<< nrConfsFound <<" configurations"<<endl;

    double value = 0.0;
    for(Index bcI=0; bcI < nrConfsFound; bcI++)
    {
        libDAI::MADP_util::valConf& vc = bestConfs.front();
        std::vector<size_t> & config = vc.second;
        value = vc.first;
    
        //translate found configuration to jpol
        boost::shared_ptr< JointPolicyDiscretePure > t1 = GetNewJpol();
        //JPPV_sharedPtr temp= boost::dynamic_pointer_cast<JPPV_sharedPtr>(jpol);
        boost::shared_ptr< JointPolicyPureVector > temp = boost::dynamic_pointer_cast<JointPolicyPureVector>(t1);
        JointPolicyPureVector jpolBG( *temp);
        //JointPolicyPureVector jpolBG( _m_solution.GetJointPolicyPureVector() );
        for(Index agI = 0; agI < bgcg->GetNrAgents(); agI++)
            for(Index tI = 0; tI < bgcg->GetNrTypes(agI); tI++)
            {
                Index varIndex = var_indices[agI][tI];
                Index bestAction = config[varIndex];
                jpolBG.SetAction(agI, tI, bestAction);
            }
        //_m_solution.SetPolicy(jpolBG);

        //store the solution
        AddSolution(jpolBG,value);

        if(_m_verbosity >= 3)
        {
            cout << "adding jpol# "<<bcI<<" constructed from config: "<< endl;
            cout << jpolBG.SoftPrint()<<endl;
        }   

        bestConfs.pop_front();
    }

    return(value);



}


void BGCG_SolverMaxPlus::Construct_JointType_Factors_CGBG(
        const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,
        const vector< vector<Index> >& var_indices,
        const vector< libDAI::Var >& vars,
        vector<libDAI::Factor>& facs,
        int verbosity)
{
    size_t nrLRFs = bgcg->GetNrLRFs();
    //compute nrFacs:
    size_t nrFacs =  0;
    for(Index e=0; e < nrLRFs; e++)
    {
        const BayesianGameIdenticalPayoff * bgip_e = bgcg->GetBGIPforLRF (e);
        size_t nrJT_e = bgip_e->GetNrJointTypes();
        if(verbosity >= 2)
            cout << "Edge "<<e<<" has " << nrJT_e << 
                " local joint types (i.e. factors)" <<endl;
        nrFacs += nrJT_e;
    }
    if(verbosity >= 1)
        cout << "FactorGraph has "<<nrFacs<<" factors"<<endl;
    facs.reserve(nrFacs);


    for(Index e=0; e < nrLRFs; e++)
    {
        //construct the factors for LRF component e.
        const BayesianGameIdenticalPayoff * bgip_e = bgcg->GetBGIPforLRF (e);
        Scope agentScope = bgcg->GetScope(e);

        const size_t nrJA_e =  bgip_e->GetNrJointActions();
        for(Index jtI_e=0; jtI_e < bgip_e->GetNrJointTypes(); jtI_e++)
        {
            vector<Index> indTypes_e =
                bgip_e->JointToIndividualTypeIndices(jtI_e);

            if(verbosity > 5)
                cout << "Adding factor for joint type jtI_e="<<jtI_e<<
                    " ( indtypes_e="<<SoftPrintVector(indTypes_e) << " )"<<endl;

            //compute the agent-type (i.e., factor) indices atI, get the
            //relevant variables and put them in a VarSet
            libDAI::VarSet vs;
            for(Index agI_e=0; agI_e < bgip_e->GetNrAgents(); agI_e++)
            {
                //translate agI_e to global agent index:
                Index agI = agentScope.at(agI_e);
                Index atI = var_indices[agI][indTypes_e[agI_e]];
                vs = vs | vars[atI];
                if(verbosity > 5)
                    cout << "agent "<<agI<<
                        " contributes through variable atI="<<atI<<endl;

            }
            libDAI::Real factor_vals[nrJA_e];
            for(Index jaI_e=0; jaI_e < nrJA_e; jaI_e++)
            {
                vector<Index> indAcs = 
                    bgip_e->JointToIndividualActionIndices(jaI_e);
                vector<size_t> indAcsST(indAcs.size());
                for(Index i=0; i<indAcs.size(); i++)  
                    indAcsST[i] = (size_t) indAcs[i];

                if(verbosity > 5)
                    cout << "Computing joint factor index (JFI) for vs="<<vs
                        <<", (action)indices="<< SoftPrintVector(indAcs) <<endl;
                Index jointFactorIndex = 
                    libDAI::Factor::IndividualToJointFactorIndex(vs, indAcsST);
                
                if(verbosity > 5)
                    cout << "JFI="<<jointFactorIndex<< " Utility "
                         << bgip_e->GetUtility(jtI_e, jaI_e) << " Prob "
                         << bgip_e->GetProbability(jtI_e) << endl;

                double perturbation=(1e-6*rand())/RAND_MAX;
                factor_vals[jointFactorIndex] =
                    (bgip_e->GetUtility(jtI_e, jaI_e) + perturbation) *
                    bgip_e->GetProbability(jtI_e);
            }

            libDAI::Factor f(vs, factor_vals);
            if(verbosity > 4)
            {
                cout << "Added factor f:"<< f << endl;
            }
            facs.push_back(f);
        }// <- end for jtI_e
    }// <- end for e
}
