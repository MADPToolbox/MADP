/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "SimulationDecPOMDPDiscrete.h"
#include "JointPolicyDiscrete.h"
#include "JointPolicyPureVectorForClusteredBG.h"
#include "BayesianGameWithClusterInfo.h"
#include "AgentLocalObservations.h"
#include "AgentSharedObservations.h"
#include "AgentDelayedSharedObservations.h"
#include "AgentFullyObservable.h"

#include "JointObservationHistoryTree.h"
#include "JointObservation.h"
#include "JointAction.h"

using namespace std;

SimulationDecPOMDPDiscrete::
SimulationDecPOMDPDiscrete(const PlanningUnitDecPOMDPDiscrete &pu, 
                           int nrRuns, int seed) : 
    Simulation(nrRuns, seed),
    _m_pu(&pu),
    _m_saveIntermediateResults(false)
{
    Initialize();
}

SimulationDecPOMDPDiscrete::
SimulationDecPOMDPDiscrete(const PlanningUnitDecPOMDPDiscrete &pu, 
                           const ArgumentHandlers::Arguments &args) : 
    Simulation(args.nrRuns, args.randomSeed),
    _m_pu(&pu),
    _m_saveIntermediateResults(false)
{
    if(args.verbose >= 4)
        SetVerbose(true);
    Initialize();
}

//Destructor
SimulationDecPOMDPDiscrete::~SimulationDecPOMDPDiscrete()
{
}

void SimulationDecPOMDPDiscrete::Initialize()
{
    if(_m_pu->GetHorizon()==MAXHORIZON)
    {
        // figure out until what horizon we should sample to get a
        // maximum error smaller than 1-e6
        double maxAbsReward=0;
        for(Index s=0;s!=_m_pu->GetNrStates();++s)
            for(Index ja=0;ja!=_m_pu->GetNrJointActions();++ja)
                if(abs(_m_pu->GetReward(s,ja))>maxAbsReward)
                    maxAbsReward=abs(_m_pu->GetReward(s,ja));
        
        _m_horizon=lrint(ceil((log(1e-6/maxAbsReward)/
                               log(_m_pu->GetDiscount()))));
        if(GetVerbose())
            cout << "Set horizon to " << _m_horizon << " (g "
                 << _m_pu->GetDiscount() << " max|R| " << maxAbsReward
                 << ")" << endl;
    }
    else
        _m_horizon=_m_pu->GetHorizon();

    if(GetVerbose())
        cout << "Simulation::RunSimulations horizon " << _m_horizon
             << " nrRuns " << GetNrRuns() << " seed " 
             << GetRandomSeed() << endl;
    
    if(GetRandomSeed()!=illegalRandomSeed)
    {
        // Seed the random number generator
        srand(GetRandomSeed());
    }
}

void SimulationDecPOMDPDiscrete::SaveIntermediateResults(string filename)
{
    _m_saveIntermediateResults=true;
    _m_intermediateResultsFilename=filename;
}

SimulationResult
SimulationDecPOMDPDiscrete::RunSimulations(const boost::shared_ptr<JointPolicyDiscrete> &jp) const
{
    return(RunSimulations(jp.get()));
}

SimulationResult
SimulationDecPOMDPDiscrete::RunSimulations(const JointPolicyDiscrete *jp) const
{
    SimulationResult result(_m_horizon,GetRandomSeed(),GetNrRuns());
#if 0
    if(GetVerbose())
        jp->Print();
#endif
    // Run the simulations
    int i;
    for(i=0;i<GetNrRuns();i++)
    {
        double res = RunSimulation(jp);
        if(GetVerbose())
            cout << "Run ended r="<<res<<endl;
        result.AddReward(res);
    }

    return(result);
}

double
SimulationDecPOMDPDiscrete::RunSimulation(const JointPolicyDiscrete *jp) const
{
    if(dynamic_cast<const JointPolicyPureVectorForClusteredBG*>(jp)!=0)
        return(RunSimulationClusteredBG(dynamic_cast<const JointPolicyPureVectorForClusteredBG*>(jp)));

    Index jaI,sI,joI;
    double r,sumR=0;
    Index johI = INITIAL_JOHI;

    sI = _m_pu->GetDPOMDPD()->SampleInitialState();

    if(GetVerbose())
        cout << "Simulation::RunSimulation " << endl
             << "Simulation::RunSimulation set initial state to " 
             << sI << endl;

    for(unsigned int t=0;t<_m_horizon;t++)
    {	
        jaI = jp->SampleJointAction(johI);

        Step(jaI, t, sI, joI, r, sumR, 0);
        
        /* action taken at ts=0,...,hor-1 - therefore only observation
         * histories at ts=0,...,hor-2 have successors.*/
        if(t < _m_horizon-1) 
            johI = _m_pu->GetSuccessorJOHI(johI, joI);
    }

    return(sumR);
}

double
SimulationDecPOMDPDiscrete::
RunSimulationClusteredBG(const JointPolicyPureVectorForClusteredBG* jp) const
{
    // instead of converting it to a JointPolicyPureVector, here we
    // work directly in the clustered BGs

    Index jaI,joI=INT_MAX;
    Index sI;
    double r, sumR=0;

    sI = _m_pu->GetDPOMDPD()->SampleInitialState();

    if(GetVerbose())
        cout << "Simulation::RunSimulation " << endl
             << "Simulation::RunSimulation set initial state to " 
             << sI << endl;

    // first we get the partial joint policies and BGS for each time
    // step, since we will need to go forward in time, and the
    // provided jp refers to the last time step. This code is very
    // similar to
    // JointPolicyPureVectorForClusteredBG::ToJointPolicyPureVector()
    const JointPolicyPureVectorForClusteredBG* jpolBG = jp;
    BGwCI_constPtr bg;
    vector<const JointPolicyPureVectorForClusteredBG*> jpolBGVec(_m_horizon);
    vector<BGwCI_constPtr> bgVec(_m_horizon);
    Index t=0;
    for(Index t_inv = 0; t_inv < _m_horizon; t_inv++)
    {
        t = _m_horizon - 1 - t_inv;
        bg =  jpolBG->GetBG();
        jpolBGVec.at(t) = jpolBG;
        bgVec.at(t) = bg;
        if( t > 0  )
        {
            jpolBG = jpolBG->GetPrevJPPVfCBG().get();
            if(jpolBG == 0)
            {
                throw(E("SimulationDecPOMDPDiscrete::RunSimulationClusteredBG only complete JointPolicyPureVectorForClusteredBG are supported for now"));
            }
        }
        //cout << bg->SoftPrint() << endl;
    }

    vector<const TypeCluster*> tcPrev(_m_pu->GetNrAgents(),0);
    vector<Index> aIs(_m_pu->GetNrAgents());
    vector<Index> oIs(_m_pu->GetNrAgents());
    for(t=0;t<_m_horizon;++t)
    {
        if(t>0)
        {
            oIs=_m_pu->JointToIndividualObservationIndices(joI);
            for(Index agI=0;agI!=_m_pu->GetNrAgents();++agI)
            {
                // find the typecluster for the action and observation...
                Index tcI=bgVec.at(t)->FindTypeClusterIndex(
                    agI, tcPrev.at(agI), aIs.at(agI), oIs.at(agI));
                tcPrev.at(agI)=bgVec.at(t)->GetTypeCluster(agI, tcI);
                // ... and get the action prescribed for it by the policy
                aIs.at(agI)=jpolBGVec.at(t)->GetActionIndex(agI,tcI);
            }
            jaI=_m_pu->IndividualToJointActionIndices(aIs);
        }
        else // first time step, no joint observation yet
        {
            jaI=jpolBGVec.at(t)->GetJointActionIndex(INITIAL_JOHI);
            aIs=_m_pu->JointToIndividualActionIndices(jaI);
            for(Index agI=0;agI!=_m_pu->GetNrAgents();++agI)
                tcPrev.at(agI)=bgVec.at(t)->GetTypeCluster(agI, 0);

        }

        Step(jaI, t, sI, joI, r, sumR, 0);
    }

    return(sumR);
}

void SimulationDecPOMDPDiscrete::Step(Index jaI, unsigned int t, Index &sI,
                                      Index &joI, double &r,
                                      double &sumR, double specialR) const
{
    Index sI_suc=_m_pu->GetDPOMDPD()->SampleSuccessorState(sI,jaI);

    joI=_m_pu->GetDPOMDPD()->SampleJointObservation(jaI,sI_suc);
    r = _m_pu->GetDPOMDPD()->GetReward(sI,jaI) + specialR;

    // calc. the discounted reward
    sumR+=r*pow(_m_pu->GetDiscount(),static_cast<double>(t));
    
    if(GetVerbose())
        cout << "Simulation::RunSimulation ("
             << sI << "," << jaI << "," << sI_suc << ") ("
             << (_m_pu->GetDPOMDPD()->GetState(sI)->SoftPrintBrief())
             << ","
             << (_m_pu->GetJointAction(jaI))->SoftPrintBrief()
             << ","
             << (_m_pu->GetDPOMDPD()->GetState(sI_suc)->SoftPrintBrief())
             << ") (p " 
             << _m_pu->GetDPOMDPD()->GetTransitionProbability(sI,jaI,sI_suc) 
             << ") jo " << joI << " "
             << (_m_pu->GetJointObservation(joI))->SoftPrintBrief()
             << " (p " 
             << _m_pu->GetDPOMDPD()->GetObservationProbability(jaI,sI_suc,joI)
             << ") r " << r << " sumR " << sumR << endl;

    sI = sI_suc;
}

Index SimulationDecPOMDPDiscrete::
GetAction(const vector<AgentLocalObservations*> &agents, Index i,
          Index jaI, Index joI, double r, Index prevJoI, Index sI, 
          Index prevJaI, double &specialR) const
{
    if(joI==INT_MAX) //first stage: there is no joI
        //return(agents[i]->Act(INT_MAX)); //<- let's not push magic numbers inside other classes, please!
        return(agents[i]->ActFirstStage());
    else
    {
        vector<Index> oIs=_m_pu->JointToIndividualObservationIndices(joI);
        return(agents[i]->Act(oIs[i]));
    }
}

Index SimulationDecPOMDPDiscrete::
GetAction(const vector<AgentSharedObservations*> &agents, Index i,
          Index jaI, Index joI, double r, Index prevJoI, Index sI,
          Index prevJaI, double &specialR) const
{
    return(agents[i]->Act(joI));
}

Index SimulationDecPOMDPDiscrete::
GetAction(const vector<AgentDelayedSharedObservations*> &agents,
          Index i, Index jaI, Index joI, double r, Index prevJoI,
          Index sI, Index prevJaI, double &specialR) const
{
    if(joI==INT_MAX) //first stage: there is no joI
        return(agents[i]->Act(INT_MAX,prevJoI));
    else
    {
        vector<Index> oIs=_m_pu->JointToIndividualObservationIndices(joI);
        return(agents[i]->Act(oIs[i],prevJoI));
    }
}

Index SimulationDecPOMDPDiscrete::
GetAction(const vector<AgentFullyObservable*> &agents,
          Index i, Index jaI, Index joI, double r, Index prevJoI,
          Index sI, Index prevJaI, double &specialR) const
{
    return(agents[i]->Act(sI,joI,r));
}
