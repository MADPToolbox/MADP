/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "QHybrid.h"
#include "directories.h"
#include "JointBeliefInterface.h"
#include "JointBelief.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "BeliefIteratorGeneric.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "BGIP_SolverBranchAndBound.h"

using namespace std;
using namespace qheur;

#define DEBUG_QHybrid 0
#define DEBUG_QHybrid_COMP 0
#define DEBUG_QHybrid_COMPREC 0

#define USE_BNB_SOLVER 0

//Default constructor
QHybrid::QHybrid(const PlanningUnitDecPOMDPDiscrete* pu,
                 Qheur_t QheurTypeFirstTimeSteps,
                 QFunctionJointBeliefInterface *QlastTimeSteps,
                 size_t horizonLastTimeSteps) : 
    QFunctionForDecPOMDP(pu), //virtual base first
    _m_QheurTypeFirstTS(QheurTypeFirstTimeSteps),
    _m_QlastTimeSteps(QlastTimeSteps),
    _m_horizonLastTimeSteps(horizonLastTimeSteps),
    _m_horizonFirstTimeSteps(GetPU()->GetHorizon()-horizonLastTimeSteps),
    _m_optimizedHorLast(false),
    _m_initialized(false)
{
    if(horizonLastTimeSteps>=GetPU()->GetHorizon())
        throw(E("QHybrid horizonLastTimeSteps should be at least 1 less than the problem horizon"));
}
QHybrid::QHybrid(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                 Qheur_t QheurTypeFirstTimeSteps,
                 QFunctionJointBeliefInterface *QlastTimeSteps,
                 size_t horizonLastTimeSteps) : 
    QFunctionForDecPOMDP(pu), //virtual base first
    _m_QheurTypeFirstTS(QheurTypeFirstTimeSteps),
    _m_QlastTimeSteps(QlastTimeSteps),
    _m_horizonLastTimeSteps(horizonLastTimeSteps),
    _m_horizonFirstTimeSteps(GetPU()->GetHorizon()-horizonLastTimeSteps),
    _m_optimizedHorLast(false),
    _m_initialized(false)
{
    if(horizonLastTimeSteps>=GetPU()->GetHorizon())
        throw(E("QHybrid horizonLastTimeSteps should be at least 1 less than the problem horizon"));
}

//Destructor
QHybrid::~QHybrid()
{    
    DeInitialize();
    delete _m_QlastTimeSteps;
}

void QHybrid::DeInitialize()
{
    _m_QValuesFirstTimeSteps.clear();
    _m_initialized=false;
}

void QHybrid::Initialize()
{
#if 1 // now we can handle this case -> not working completely yet...
    if(!_m_QlastTimeSteps)
        throw(E("QHybrid::Initialize QlastTimeSteps should be valid"));
#endif

    if(!(_m_QheurTypeFirstTS==eQBG || _m_QheurTypeFirstTS==eQPOMDP))
        throw(E("QHybrid can only use QBG or QPOMDP for the first time steps"));

    // first we need to compute for how many JAOHs we will store Qvalues
    size_t nr=0;
    for(Index t=0; t<_m_horizonFirstTimeSteps ;++t)
    {
        size_t nrT=1; //the number of histories at 
        for (Index aI=0 ; aI<GetPU()->GetNrAgents(); ++aI)
        {
            size_t historiesThisAgent =  
                GetPU()->GetNrObservationHistories(aI,t) *  
                GetPU()->GetNrActionHistories(aI,t);
            nrT *= historiesThisAgent;
        }
        nr += nrT;
    }
    _m_nrJAOHinFirstTS=nr;

    _m_QValuesFirstTimeSteps.resize(_m_nrJAOHinFirstTS,
                                    GetPU()->GetNrJointActions(),
                                    false);
    _m_QValuesFirstTimeSteps.SetToZero();
    _m_initialized = true;
 
    cout << SoftPrintBrief() << " with " << _m_nrJAOHinFirstTS << " JAOH in tree ("
         << _m_horizonFirstTimeSteps << " ts), and";
    cout << " vectors for " << _m_horizonLastTimeSteps << " ts."
         << endl; 
}

void QHybrid::Compute()
{
    if(!_m_initialized)
        Initialize();

    // first compute the heuristic for the final time steps, as we
    // will use these values for the backup
    if(_m_QlastTimeSteps)
        _m_QlastTimeSteps->Compute();

    ComputeQ();
}

void QHybrid::ComputeQ()
{
    if(GetPU() == 0)
        throw E("QHybrid::ComputeQ - GetPU() returns 0; no PlanningUnit available!");

    size_t time_step = 0;
    JointBeliefInterface* b0p = GetPU()->GetNewJointBeliefFromISD(); 
    JointBeliefInterface& b0 =  *b0p;
    
    bool last_t = false;
    if( (time_step) == (GetPU()->GetHorizon() -
                        _m_horizonLastTimeSteps))
        last_t = true;

    if(last_t)
        throw E("QHybrid::ComputeQ - last_t is true, not possible");


    size_t empty_jaohI = 0;
    for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
    {
        double Q=0.0;
        if(last_t)
        {
            if(_m_QlastTimeSteps)
                Q = _m_QlastTimeSteps->GetQ(b0,
                                            Globals::INITIAL_JAOHI,
                                            newJAI);
            else
            {
                BeliefIteratorGeneric it=b0.GetIterator();
                do
                {
                    double r_s_ja = GetPU()->GetReward(it.GetStateIndex(), newJAI);
                    double prob_s = it.GetProbability();
                    Q += r_s_ja * prob_s;
                } while(it.Next());
                
            }
        }
        else
        {
            //calculate R(joah',newJA) - expected immediate reward for time_step
            double exp_imm_R = 0.0;
            BeliefIteratorGeneric it=b0.GetIterator();
            do
            {
                double r_s_ja = GetPU()->GetReward(it.GetStateIndex(), newJAI);
                double prob_s = it.GetProbability();
                exp_imm_R += r_s_ja * prob_s;
            } while(it.Next());

            double exp_fut_R = ComputeRecursively( 1, Globals::INITIAL_JAOHI, newJAI);

            Q = exp_imm_R + GetPU()->GetDiscount() * exp_fut_R;
        }
        _m_QValuesFirstTimeSteps(empty_jaohI,newJAI)=Q;
    }//end for newJAI

    delete b0p;
    return;
}

double QHybrid::ComputeRecursively(size_t time_step,
                                   LIndex jaohI,
                                   Index lastJAI)
{
    bool last_t = false;
    if( time_step == _m_horizonFirstTimeSteps )
        last_t = true;

    // we don't have a vector-based representation, so at the last
    // time step just return the immediate reward
    if(last_t && !_m_QlastTimeSteps)
    {
        double Q=0.0;
        JointBeliefInterface* jbi =
            GetPU()->GetJointBeliefInterface(jaohI);
        BeliefIteratorGeneric it=jbi->GetIterator();
        do
        {
            double r_s_ja = GetPU()->GetReward(it.GetStateIndex(), lastJAI);
            double prob_s = it.GetProbability();
            Q += r_s_ja * prob_s;
        } while(it.Next());
        delete jbi;
        return(Q);
    }

    BGIP_sharedPtr bg_time_step;
    if(_m_QheurTypeFirstTS==eQBG)
        bg_time_step=BGIP_sharedPtr(
            new BayesianGameIdenticalPayoff(
                GetPU()->GetNrAgents(), 
                GetPU()->GetNrActions(),
                GetPU()->GetNrObservations()));
            
    double Vpomdp = 0.0;
    double discount = GetPU()->GetDiscount();

    for(Index newJOI=0; newJOI < GetPU()->GetNrJointObservations(); newJOI++)
    {
        Index new_jaohI = CastLIndexToIndex(
            GetPU()->GetSuccessorJAOHI(jaohI, lastJAI, newJOI));

        //get the new joint belief at this time-step resulting from lastJAI, 
        //newJOI...(the true prob. dist over states for the actions and obser-
        //vations as given by the history < jaoh, lastJA, newJOI > )
        
        JointBeliefInterface* new_jbi =
            GetPU()->GetJointBeliefInterface(new_jaohI);
        double Po_ba = GetPU()->GetJAOHProbGivenPred(new_jaohI);

        // if the probability of this observation occurring is zero,
        // the belief is not defined, and don't have to consider this
        // part of the tree anymore
        if(Po_ba<PROB_PRECISION)
            continue;

        if(_m_QheurTypeFirstTS==eQBG)
            bg_time_step->SetProbability(newJOI, Po_ba);

        //for all joint actions newJA
        double maxQ = -DBL_MAX;
        for(Index newJAI=0; newJAI < GetPU()->GetNrJointActions(); newJAI++)
        {
            double Q=0.0;
            if(last_t)
                Q = _m_QlastTimeSteps->GetQ(*new_jbi, 0, newJAI);
            else
            {
                //calculate R(joah',newJA) - expected immediate reward for time_step
                double exp_imm_R = 0.0;
                BeliefIteratorGeneric it=new_jbi->GetIterator();
                do
                {
                    double r_s_ja = GetPU()->GetReward(it.GetStateIndex(), newJAI);
                    double prob_s = it.GetProbability();
                    exp_imm_R += r_s_ja * prob_s;
                } while(it.Next());

                double exp_fut_R = ComputeRecursively(time_step+1, new_jaohI, newJAI);
                Q = exp_imm_R + discount * exp_fut_R;
            }
            if(new_jaohI<_m_nrJAOHinFirstTS)
                _m_QValuesFirstTimeSteps(new_jaohI,newJAI)=Q;
            if(Q > maxQ)
                maxQ = Q;
            if(_m_QheurTypeFirstTS==eQBG)
                bg_time_step->SetUtility(newJOI, newJAI, Q);
        }//end for newJAI
        
        //joint belief no longer needed:
        delete new_jbi;

        Vpomdp += Po_ba * maxQ;

    }//end for newJOI

    switch(_m_QheurTypeFirstTS)
    {
    case eQBG:
    {
        //solve this bayesian game, asking for only 1 solution
        int BGsolverVerbose=0;
        int nrSolutionsWanted=1;       
#if USE_BNB_SOLVER       
        BGIP_SolverBranchAndBound<JointPolicyPureVector> bgs(bg_time_step,
                                                             BGsolverVerbose,nrSolutionsWanted,
                                                             false,
                                                             BGIP_BnB::DescendingProbability,
                                                             false); // at least for boxPushing CCI was slower
#else
        BGIP_SolverBruteForceSearch<JointPolicyPureVector> bgs(bg_time_step,BGsolverVerbose,nrSolutionsWanted);
#endif
        double Vbg = bgs.Solve();
        return(Vbg);
    }
    case eQPOMDP:
    {
        return(Vpomdp);
    }
    default:
        throw(E("QHybrid: invalid Qheuristic for first time steps"));
    }
    return(42);
}


string QHybrid::SoftPrintBrief() const
{
    stringstream ss;
    ss << "QHybrid_"
       << SoftPrint(_m_QheurTypeFirstTS)
       << "_";
    if(_m_QlastTimeSteps)
        ss << _m_QlastTimeSteps->SoftPrintBrief();
    else
        ss << "noQlast";
    ss << "_hLT";
    if(_m_optimizedHorLast)
        ss << "0";
    else
        ss <<_m_horizonLastTimeSteps;
    return(ss.str());
}

void QHybrid::ComputeWithCachedQValues(bool computeIfNotCached)
{
    string filenameCache=GetCacheFilename();

    // check whether both files exist
    bool cached=true;
    {
        stringstream ss;
        ss << filenameCache << "_firstTS";
        ifstream fp(ss.str().c_str());
        if(!fp)
            cached=false;
    }

    if(!cached && !computeIfNotCached)
    {
        stringstream ss;
        ss << "QHybrid::ComputeWithCachedQValues: Q function "
           << filenameCache << " not cached, bailing out";
        throw(E(ss.str()));
        return;
    }

    // Couldn't open cache file, so compute
    if(!cached)
    {
        Compute();
        Save(filenameCache);
    }
    else // load Q values from file
        Load(filenameCache);
}
    
void QHybrid::Load(const std::string& filename)
{
    // figure out what _m_horizonLastTimeSteps should be by reading it from disk
    if(_m_horizonLastTimeSteps==0)
    {
        SetOptimizedHorLast(true);
        stringstream ss;
        ss << filename << "_horLast";
        ifstream fp(ss.str().c_str());
        if(!fp)
        {
            stringstream ss1;
            ss1 << "QHybrid::Load: failed to "
               << "open file " << ss.str();
            throw(E(ss1.str()));
        }
        string buffer;
        getline(fp,buffer);
        istringstream is(buffer);
        is >> _m_horizonLastTimeSteps;
    }
    else
        SetOptimizedHorLast(false);

    if(!_m_initialized)
        Initialize();

    {
        stringstream ss;
        ss << filename << "_firstTS";
        QTable::Load(ss.str(),
                     _m_nrJAOHinFirstTS,
                     GetPU()->GetNrJointActions(),
                     _m_QValuesFirstTimeSteps);
    }
    {
        stringstream ss;
        ss << filename << "_lastTS";
        _m_QlastTimeSteps->Load(ss.str());
    }
}

void QHybrid::Save(const std::string& filename) const
{
    if(_m_optimizedHorLast)
    {
        stringstream ss;
        ss << filename << "_horLast";
        ofstream fp(ss.str().c_str());
        if(!fp)
        {
            stringstream ss1;
            ss << "QHybrid::Save: failed to open file " << ss.str() << endl;
            throw E(ss1.str());
        }
        fp << _m_horizonLastTimeSteps << endl;
    }

    {
        stringstream ss;
        ss << filename << "_firstTS";
        QTable::Save(_m_QValuesFirstTimeSteps,ss.str());
    }
    {
        stringstream ss;
        ss << filename << "_lastTS";
        if(_m_QlastTimeSteps)
            _m_QlastTimeSteps->Save(ss.str());
        else // create an empty file not to confuse any loading routine
            ofstream fp(ss.str().c_str());
    }
}

double QHybrid::GetQ(Index jaohI, Index jaI) const
{
    // we check whether this jaohI is stored in the
    // _m_QValuesFirstTimeSteps
    if(jaohI<_m_nrJAOHinFirstTS)
    {
        return (_m_QValuesFirstTimeSteps(jaohI,jaI));
    }
    else
    {
#if 1 // this check is pretty slow
        double pJAOH=GetPU()->GetJAOHProb(jaohI);
        if(pJAOH>0)
        {
#endif
            Index t = GetPU()->GetTimeStepForJAOHI(jaohI) -
                _m_horizonFirstTimeSteps;
            JointBeliefInterface * jb = GetPU()->GetJointBeliefInterface(jaohI);
            double q = _m_QlastTimeSteps->GetQ(*jb, t, jaI);
            delete jb;
            return( q );
#if 1
        }
        else
            return(0);
#endif
    }
}

bool QHybrid::StageRepresentedAsTree(Index ts) const
{
    if(ts<_m_horizonFirstTimeSteps)
        return(true);
    else
        return(false);
}
