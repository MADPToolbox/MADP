/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "Perseus.h"
#include "AlphaVectorPlanning.h"
#include "PerseusBackupType.h"
#include <fstream>
#include <float.h>

using namespace std;

//Default constructor
Perseus::Perseus(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    _m_verbose(0),
    _m_initializeWithImmediateReward(false),
    _m_initializeWithZero(false),
    _m_bestValue(-DBL_MAX),
    _m_beliefsInitialized(false),
    _m_identification("Perseus"),
    _m_storeIntermediateValueFunctions(false),
    _m_storeTimings(false),
    _m_computeVectorForEachBelief(false),
    _m_dryrun(false)
{
    SetMinimumNumberOfIterations(10);
    SetMaximumNumberOfIterations(INT_MAX);
    UpdateValueFunctionName();
}

Perseus::Perseus(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    _m_verbose(0),
    _m_initializeWithImmediateReward(false),
    _m_initializeWithZero(false),
    _m_bestValue(-DBL_MAX),
    _m_beliefsInitialized(false),
    _m_identification("Perseus"),
    _m_storeIntermediateValueFunctions(false),
    _m_storeTimings(false),
    _m_computeVectorForEachBelief(false),
    _m_dryrun(false)
{
    SetMinimumNumberOfIterations(10);
    SetMaximumNumberOfIterations(INT_MAX);
    UpdateValueFunctionName();
}

//Destructor
Perseus::~Perseus()
{
}

void Perseus::Initialize()
{
}

ValueFunctionPOMDPDiscrete Perseus::GetInitialValueFunction() const
{
    int nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions();

    ValueFunctionPOMDPDiscrete V0;

    if(_m_initializeWithImmediateReward && _m_initializeWithZero)
        throw(E("Perseus::GetInitialValueFunction() can only initialize in one way"));

    if(_m_initializeWithImmediateReward)
    {
        AlphaVector alpha(nrS);
        for(int a=0;a<nrA;a++)
        {
            for(int s=0;s!=nrS;s++)
                alpha.SetValue(GetPU()->GetReward(s,a),s);
            V0.push_back(alpha);
        }
    }
    else
    {
        AlphaVector alpha(nrS);
        double initialValue;

        if(_m_initializeWithZero)
            initialValue=0;
        else
        {
            double minReward=DBL_MAX;
            for(int a=0;a<nrA;a++)
                for(int s=0;s<nrS;s++)
                    if(GetPU()->GetReward(s,a)<minReward)
                        minReward=GetPU()->GetReward(s,a);

            // check if the problem is finite or infinite horizon
            if(GetPU()->GetHorizon()!=MAXHORIZON)
                initialValue=minReward*GetPU()->GetHorizon();
            else
                initialValue=minReward/(1-GetPU()->GetDiscount());
        }

        for(int s=0;s!=nrS;s++)
            alpha.SetValue(initialValue,s);
        
        alpha.SetAction(INT_MAX); // set action to an illegal value
        V0.push_back(alpha);
    }

    return(V0);
}

QFunctionsDiscrete Perseus::GetInitialQFunctions() const
{
    QFunctionsDiscrete Q0;

    unsigned int nrS=GetPU()->GetNrStates(),
        nrA=GetPU()->GetNrJointActions();

    if(_m_initializeWithImmediateReward && _m_initializeWithZero)
        throw(E("Perseus::GetInitialQFunctions() can only initialize in one way"));

    if(_m_initializeWithImmediateReward)
    {
        AlphaVector alpha(nrS);
        ValueFunctionPOMDPDiscrete V0;
        for(unsigned int a=0;a!=nrA;++a)
        {
            V0.clear();
            for(unsigned int s=0;s!=nrS;++s)
                alpha.SetValue(GetPU()->GetReward(s,a),s);
            alpha.SetAction(a);
            V0.push_back(alpha);
            Q0.push_back(V0);
        }
    }
    else
    {
        ValueFunctionPOMDPDiscrete V0=GetInitialValueFunction();
        for(unsigned int a=0;a!=GetPU()->GetNrJointActions();++a)
        {
            for(unsigned int i=0;i!=V0.size();++i)
                V0[i].SetAction(a);
            Q0.push_back(V0);
        }
    }

    return(Q0);
}

QFunctionsDiscreteNonStationary
Perseus::GetInitialNonStationaryQFunctions() const
{
    QFunctionsDiscrete Qt0=GetInitialQFunctions();
    QFunctionsDiscreteNonStationary Q0;
    for(Index t=0;t!=GetPU()->GetHorizon();++t)
        Q0.push_back(Qt0);
    return(Q0);
}

void Perseus::PrintMaxRewardInBeliefSet() const
{
    if(!_m_beliefsInitialized)
        throw(E("Perseus::PrintMaxRewardInBeliefSet belief set not initialized"));

    vector<double> beliefValues=GetImmediateRewardBeliefSet();

    double maxBeliefValue=-DBL_MAX;
    for(unsigned int i=0;i!=beliefValues.size();i++)
    {
        if(beliefValues[i]>maxBeliefValue)
            maxBeliefValue=beliefValues[i];
    }
    if(GetVerbose() >= 0)
        cout << GetIdentification() << ": max reward in beliefset is "
             << maxBeliefValue << endl;
}

int Perseus::SampleNotImprovedBeliefIndex(vector<bool> stillNeedToBeImproved,
                                          int nrNotImproved) const
{
    int beliefI,l,k;

    // sample a belief index from the number of not improved beliefs
    beliefI=static_cast<int>(nrNotImproved*(rand() / (RAND_MAX + 1.0)));
    
    // figure out the index k of this belief in S 
    l=0;
    k=-1;
    for(unsigned int j=0;j!=stillNeedToBeImproved.size();j++)
    {
        if(stillNeedToBeImproved[j])
        {
            if(beliefI==l)
            {
                k=j;
                break;
            }
            l++;
        }
    }

    if(k==-1)
    {
        PrintVectorCout(stillNeedToBeImproved);
        cout << "nrNotImproved " << nrNotImproved << " beliefI "
             << beliefI << endl;
        throw(E("Perseus::SampleNotImprovedBeliefIndex did not sample valid k"));
    }
    return(k);
}

bool Perseus::CheckConvergence(const vector<double> &VB,
                               const vector<double> &VBnew,
                               int iter) const
{
    bool converged;

    double maxDiff=0;
    for(unsigned int i=0;i!=VB.size();i++)  // for all beliefs, 
        if(abs(VB[i]-VBnew[i])>maxDiff)     // get the difference in val
            maxDiff=abs(VB[i]-VBnew[i]);    // and store the maximum...

    if(GetVerbose() >= 1)
        cout << GetIdentification() << ":CheckConvergence maxDiff is "
             << maxDiff << endl;
    if(iter>=_m_maximumNumberOfIterations)
        converged=true;
    else if(maxDiff<1e-4 && (GetPU()->GetHorizon() == MAXHORIZON ||
       (static_cast<size_t>(iter) >
        max(static_cast<size_t>(_m_minimumNumberOfIterations),
            5*GetPU()->GetHorizon()))))
        converged=true;
    else
    {
        if(iter>=max(_m_minimumNumberOfIterations,1000))
            converged=true;
        else
            converged=false;
    }

    return(converged);
}

void Perseus::PlanLeadIn()
{
    StartTimer(GetIdentification());

    if(!_m_beliefsInitialized)
    {
        int nrB=1000;
        // set the random seed and sample beliefs
        srand(42);
        if(GetVerbose() >= 0)
            cout << GetIdentification() << ": sampling " << nrB
                 << " beliefs"; cout.flush();
        InitializeBeliefs(nrB,true);
        if(GetVerbose() >= 0)
            cout << "." << endl;
    }

    // just a manual check to figure out if the belief set has potential
    PrintMaxRewardInBeliefSet();
}

void Perseus::PlanEndOfIteration() const
{
    if(_m_storeTimings)
    {
        stringstream ss;
        ss << directories::MADPGetResultsDir("POMDP",GetPU())
           << "/intermediate/" << GetIdentification() << "Timings_h"
           << GetPU()->GetHorizon();
        SaveTimers(ss.str());
    }
#if 0 // reduce verbosity
    PrintTimersSummary();
#endif
}

void Perseus::PlanLeadOut()
{
    StopTimer(GetIdentification());
    if(_m_storeTimings)
    {
        stringstream ss;
        ss << directories::MADPGetResultsDir("POMDP",GetPU()) << "/"
           << GetIdentification() << "Timings_h" << GetPU()->GetHorizon();
        SaveTimers(ss.str());
        if(GetVerbose() >= 1)
            cout << "Saved timing results to " << ss.str() << endl;
    }
}

GaoVectorSet
Perseus::BackupStageLeadIn(const ValueFunctionPOMDPDiscrete &V) const
{
    StartTimer(GetIdentification() + "BackupStage");

    return(BackProject(V));
}

void Perseus::BackupStageLeadOut(GaoVectorSet Gao) const
{
    // release the memory of the back-projected vectors
    for(unsigned int a=0;a!=GetPU()->GetNrJointActions();a++)
        for(unsigned int o=0;o!=GetPU()->GetNrJointObservations();o++)
            delete(Gao[a][o]);
    
    StopTimer(GetIdentification() + "BackupStage");
}

string Perseus::BackupTypeToString(const QAVParameters &params)
{
    stringstream ss;
    switch(params.backup)
    {
    case POMDP:
        ss << "POMDP";
        break;
    case BG:
        ss << "BG" << params.bgBackupType;
        break;
    case EVENT_POMDP:
        ss << "EVENT_POMDP";
        break;
    default:
        ss << "PerseusBackupType " << params.backup << " is unknown";
        throw(E(ss));
    }
    return(ss.str());
}

QAVParameters Perseus::ProcessArguments(const ArgumentHandlers::Arguments &args)
{
    QAVParameters qavParams;
    qavParams.waitPenalty=DBL_MAX;
    qavParams.weight=1;
    qavParams.bgBackupType=args.bgBackup;
    
    qavParams.stationary=true;

    qavParams.backup=args.backup;
    switch(args.backup)
    {
    case POMDP:
    case BG:
    case EVENT_POMDP:
        break;
    default:
        throw(E("PerseusBackupType is unknown"));
    }

    return(qavParams);
}

void Perseus::SetIdentification(const string &identification)
{ 
    _m_identification=identification;
    UpdateValueFunctionName();
}

void Perseus::SetResultsFilename(const string &filename)
{
    _m_resultsFilename=filename;
    UpdateValueFunctionName();
}

void Perseus::UpdateValueFunctionName()
{
    stringstream valueFunctionFilename;
    valueFunctionFilename << _m_resultsFilename
                          << GetIdentification()
                          << "ValueFunction_h" << GetPU()->GetHorizon();
    _m_valueFunctionFilename=valueFunctionFilename.str();

    if(GetVerbose() >= 1)
        cout << "Set value function filename to " << _m_valueFunctionFilename
             << endl;
}

void Perseus::StoreValueFunction(const ValueFunctionPOMDPDiscrete &V)
{
    throw(E("Perseus::StoreValueFunction should be implemented by deriving class"));
}

void Perseus::StoreValueFunction(const QFunctionsDiscrete &Q)
{
    throw(E("Perseus::StoreValueFunction should be implemented by deriving class"));
}

void Perseus::StoreValueFunction(const QFunctionsDiscreteNonStationary &Q)
{
    throw(E("Perseus::StoreValueFunction should be implemented by deriving class"));
}


