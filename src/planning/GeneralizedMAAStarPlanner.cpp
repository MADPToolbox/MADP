/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "GeneralizedMAAStarPlanner.h"
#include "JointPolicyValuePair.h"
#include "QFunctionJAOHInterface.h"
#include "PartialPolicyPoolInterface.h"
#include "PolicyPoolJPolValPair.h"
#include "EDeadline.h"

#include "PartialJointPolicyPureVector.h"

#define DEBUG_GMAA_POLS 0

using namespace std;

//Default constructor
GeneralizedMAAStarPlanner::GeneralizedMAAStarPlanner(
    int verboseness,
    double slack) :
    _m_deadline(0),
    _m_foundPolicy(),
    _m_maxLowerBound(-DBL_MAX),
    _m_verboseness(verboseness),
    _m_slack(slack),
    _m_pointersToAllBGTS(vector<BayesianGameForDecPOMDPStage*>())
{
    _m_intermediateResultFile = 0;
    _m_saveIntermediateTiming=false;
    _m_nrJPolBGsEvaluated=0;
    _m_nrPoliciesToProcess=UINT_MAX;
    _m_bgCounter=0;
    _m_bgBaseFilename="";
}

//Destructor
GeneralizedMAAStarPlanner::~GeneralizedMAAStarPlanner()
{
}

//Copy assignment operator
GeneralizedMAAStarPlanner& GeneralizedMAAStarPlanner::operator= (const GeneralizedMAAStarPlanner& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    throw("GeneralizedMAAStarPlanner::operator= not implemented");

    return *this;
}

void GeneralizedMAAStarPlanner::Initialize()
{
    _m_foundPolicy=JPDP_sharedPtr();
    _m_maxLowerBound=-DBL_MAX;
    _m_expectedRewardFoundPolicy=-DBL_MAX;
    _m_bgCounter=0;

    //this counter maintains the maximum policy pool size.
    _m_maxJPolPoolSize = 0;

    _m_expanded_childs = vector<size_t>();
    _m_max_expanded_childs = vector<LIndex>();

    std::vector< BayesianGameForDecPOMDPStage* >::iterator itBGts = 
        _m_pointersToAllBGTS.begin();
    std::vector< BayesianGameForDecPOMDPStage* >::iterator lastBGts = 
        _m_pointersToAllBGTS.end();
    while(itBGts != lastBGts)
    {
        delete *itBGts;
        itBGts++;
    }
    _m_pointersToAllBGTS.clear();

    /// Call the ResetPlanner() of the derived planners.
    ResetPlanner();
}


/* this is the high-level pseudo-code for what happens:
    start with a horizon 0 joint policy - i.e. specifying 0 actions
    JPolValPool.push( <jpol,val=0.0> )    
    do
        ppi = <pol,val> = JPolValPool.GetNext()

        //poolOfNextPolicies     = {<pol,vals>} 
        //isLowerBound  = bool   - whether the vals are lower bounds to the 
        //                optimal value (i.e. value for the optimal policy)
        <poolOfNextPolicies, isLowerBound> = ConstructAndValuateNextPolicies(ppi)
        if(isLowerBound)
            Prune( JPolValPool, max(lowerBound) )

        poolOfNextPolicies = SelectPoliciesToProcessFurther(poolOfNextPolicies);
        JPolValPool.insert(poolOfNextPolicies)
      
    while !empty JPolValPool
*/
void GeneralizedMAAStarPlanner::Plan()    
{
    Initialize();

    StartTimer("GMAA::Plan");

    //stuff for timing (if used)
    tms ts_start;   //the time struct
    clock_t tck_start;  //ticks
    tck_start = times(&ts_start);
    //the intermediate timing stream
    ofstream & its = *_m_intermediateResultFile;

    tms ts_before, ts_after;
    times(&ts_before);

    boost::shared_ptr<PartialJointPolicyDiscretePure> bestJPol = NewJPol();

    bestJPol->SetPastReward(-DBL_MAX);
#if DEBUG_GMAA_POLS    
    cout << "GMAA initialized with empty policy:"<<endl;
    bestJPol->Print();
#endif

    // Setup the Policy Pool
    PartialPolicyPoolInterface_sharedPtr pp_p = NewPP(); 
    pp_p->Init( GetThisFromMostDerivedPU() ); //initialize with empty joint policy

    do
    {
        if(_m_deadline)
        {
            times(&ts_after);
            clock_t utime = ts_after.tms_utime -
                ts_before.tms_utime;
            double timeSpentInS=static_cast<double>(utime) / sysconf(_SC_CLK_TCK);
//            cout << timeSpentInS << endl;
            if(timeSpentInS>_m_deadline)
            {
                std::stringstream ss;
                ss << "GeneralizedMAAStarPlanner::Plan() we spent "
                   << timeSpentInS << "s, which is more than the deadline of "
                   << _m_deadline << "s, bailing out...";
                EDeadline e(ss.str());
                throw(e);
            }
        }

        StartTimer("GMAA::Plan::iteration");
        if(_m_saveIntermediateTiming)
            SaveTimers(_m_intermediateTimingFilename);

        if(_m_verboseness >= 2) {
            cout << "\n---------------------------------------------------\n";
            cout << "-->>Start of new GMAA iteration, polpool size="<<
                pp_p->Size()<<"<<--"<<endl;
        }

        PartialPolicyPoolItemInterface_sharedPtr ppi = pp_p->Select();
        boost::shared_ptr<PartialJointPolicyDiscretePure> jpol_sel =  ppi->GetJPol();
        double v_sel = ppi->GetValue();
        size_t depth_sel = jpol_sel->GetDepth();
        if(_m_verboseness >= 3) {
            cout << "GMAA Selected a partial jpol of depth="<< depth_sel << " and heur. val="<<v_sel<<
                " to expand" << endl;
            if(_m_verboseness >= 4) 
                ppi->GetJPol()->Print();
        }
        //cout << "SELECTed partial pol with heur. val="<<v_sel<<endl;
        
        if( (v_sel + _m_slack) < _m_maxLowerBound) //the highest upperbound < the best lower
        {
            //  1)if JPolValPool is no priority queue, this should be changed.
            if(_m_verboseness >= 0)
                cout<<"!!!GMAA::Plan highest upper < best found lower bound, stopping\n";
            break;
        }

        //poolOfNextPolicies     = {<pol,vals>} 
        //isLowerBound  = bool   - whether the vals are lower bounds to the 
        //                optimal value (i.e. value for the optimal policy)
        //<poolOfNextPolicies,isLowerBound>=ConstructAndValuateNextPolicies(ppi)

        PartialPolicyPoolInterface_sharedPtr poolOfNextPolicies = NewPP();
        bool cleanUpPPI=true;
        bool are_LBs =
            ConstructAndValuateNextPolicies(ppi,
                                            poolOfNextPolicies,
                                            cleanUpPPI);

        //this keeps track of the actually expanded *nodes*
        //however, due to our clever stuff we never exand all the nodes anymore
        //so we will need to extract the information of 'non-incremental expansion'
        //from within the CBG solver...
        if(_m_expanded_childs.size()<=depth_sel)
            _m_expanded_childs.resize(depth_sel+1,0);
        _m_expanded_childs.at(depth_sel) += poolOfNextPolicies->Size();

        //Clean up ppi - that is we remove the top element of the policy pool because that was just expanded (right?)
        if(cleanUpPPI)
            pp_p->Pop(ppi);
        else
        {
            pp_p->Pop(ppi);
            pp_p->Insert(ppi);
            
        }

#if DEBUG_GMAA4        
        if(DEBUG_GMAA4){
            cout << "--------------------------------------------------\n"<<
                    ">>>The next policies found, poolOfNextPolicies:"<<endl;
            PartialPolicyPoolInterface_sharedPtr pp_copy = NewPP();
            *pp_copy = *poolOfNextPolicies;
            while(! pp_copy->Empty())
            {
                PartialPolicyPoolItemInterface_sharedPtr it = pp_copy->Select();
                it->Print();
                cout << endl;
                pp_copy->Pop();
            }
            cout << "<<<\n---------------------------------------------"<<endl;
        }
#endif        

        //if(isLowerBound)
        //    Prune( JPolValPool, max(lowerBound) )
        if(are_LBs && poolOfNextPolicies->Size() > 0)
        {
            PartialPolicyPoolItemInterface_sharedPtr bestRanked_ppi = poolOfNextPolicies->
                GetBestRanked();
            poolOfNextPolicies->PopBestRanked();
            double bestNextVal = bestRanked_ppi->GetValue();
            if(bestNextVal > _m_maxLowerBound) //new best lowerbound (and policy) found
            {
                _m_maxLowerBound = bestNextVal;
                *bestJPol = *(bestRanked_ppi->GetJPol());
                if(_m_verboseness >= 2) {
                    cout << "new bestJPol (and max. lowerbound) found! - ";
                    cout << "value v="
                         << bestNextVal <<" - "
                         << bestRanked_ppi->GetJPol()->SoftPrintBrief() << endl;
                }
                if(_m_verboseness >= 4) 
                    cout << "new bestJPol->SoftPrint():"<<bestJPol->SoftPrint();

                //if we maintain the internal timings...
                if(_m_intermediateResultFile != 0)
                {
                    tms ts_cur;
                    clock_t tck_cur;
                    tck_cur = times(&ts_cur);
                    clock_t diff = tck_cur - tck_start;
                    its << diff << "\t" <<  _m_maxLowerBound << endl;
                }
                // prune JPolValPool
                pp_p->Prune(_m_maxLowerBound - _m_slack );
            }
        }
        SelectPoliciesToProcessFurther(poolOfNextPolicies, are_LBs, _m_maxLowerBound - _m_slack);
        pp_p->Union(poolOfNextPolicies);

        if( _m_maxJPolPoolSize < pp_p->Size())
            _m_maxJPolPoolSize = pp_p->Size();
        
        StopTimer("GMAA::Plan::iteration");
        if(_m_verboseness >= 2) { 
            if(_m_maxLowerBound==-DBL_MAX)
                cout << "--GMAA::Plan::iteration ending, polpool size="<<pp_p->Size()<<", no complete policy found yet";
            else
            {
                cout << "--GMAA::Plan::iteration ending, polpool size="<<pp_p->Size()<<", best policy found so far:";
                cout << endl << bestJPol->SoftPrintBrief()  <<endl;
                if(_m_verboseness >= 5)  
                    cout << endl << bestJPol->SoftPrint()  <<endl;
            }
        }
    } 
    while(! pp_p->Empty() ); //<- end do...while

    //we don't want to do any conversions here... takes (sometimes too much)
    //time...
    _m_foundPolicy=bestJPol;  //->ToJointPolicyPureVector());

    _m_expectedRewardFoundPolicy=_m_maxLowerBound;
    if(_m_verboseness >= 1) {
        cout << "\n----GMAA::Plan ending, best policy found:----\n";
        cout << bestJPol->SoftPrintBrief() << " = " <<endl;
        if(_m_verboseness >= 3) 
            cout << _m_foundPolicy->SoftPrint() << endl;
        cout << endl;
        if(_m_verboseness >= 2)
        {
            JPPV_sharedPtr jppv = _m_foundPolicy->ToJointPolicyPureVector();
            jppv->Print();
        }
    }
    if(_m_verboseness >= 2)    
        cout << "\n\n ";
    if(_m_verboseness >= 0)    
        cout << "GMAA::Plan GMAA ENDED"<<endl;
    if(_m_verboseness >= 2)    
        cout << "\n\n ";

    StopTimer("GMAA::Plan");

    if(_m_verboseness >= 0 && _m_max_expanded_childs.size()>0)    
    {
        cout << "Expanded nodes at different stages:\n"<<
            PrintTools::SoftPrintVector(_m_expanded_childs)<<endl;
        cout << "Maximum number of nodes that could have been expanded:\n"<<
            PrintTools::SoftPrintVector(_m_max_expanded_childs)<<endl;
    }
}

void 
GeneralizedMAAStarPlanner::SelectKBestPoliciesToProcessFurther(
    const boost::shared_ptr<PartialPolicyPoolInterface> &poolOfNextPolicies, bool are_LBs,
    double bestLB, size_t k)
{
    if(are_LBs)
    {
        //if all policies are full policies, we don't return any
        //of them (these need not be expanded further)
        while(!poolOfNextPolicies->Empty())
            poolOfNextPolicies->Pop();
        return;
    }
    PartialPolicyPoolInterface_sharedPtr pp_new = NewPP();
    
    size_t nr_done = 0;
    while(poolOfNextPolicies->Size() > 0 ) // && nr_done < k) 
    {
        PartialPolicyPoolItemInterface_sharedPtr best_ppi = poolOfNextPolicies->GetBestRanked();
        poolOfNextPolicies->PopBestRanked();
        if(nr_done >= k || best_ppi->GetValue() < bestLB)
        {
            //we do not want this policy, so discard it:
//            delete best_ppi;
        }
        else
        {
            //we do want this policy, so store it:
            pp_new->Insert(best_ppi);
        }
        nr_done++;
    }
    //done: - free the memory of the policies in poolOfNextPolicies
    //that we will not consider further!

    *poolOfNextPolicies = *pp_new;
    //remove elements from pp_new before delete! (otherwise they will be deleted    //with them) and we get a segfault later on...
    while(pp_new->Size() > 0)
        pp_new->Pop();
    
//    delete pp_new;

}

void GeneralizedMAAStarPlanner::Prune(PartialPolicyPoolInterface& pp, size_t k)
{
    if(pp.Size()<=k)
        return;

    PartialPolicyPoolInterface_sharedPtr ppPruned = NewPP();

    while(ppPruned->Size()<k)
    {
        ppPruned->Insert(pp.GetBestRanked());
        pp.PopBestRanked();
    }

    //delete the rest which we will not use further...
    while(pp.Size()>0)
    {
//        delete pp.Select();
    }
    pp=*ppPruned;
}

void GeneralizedMAAStarPlanner::SetIntermediateTimingFilename(
    const string &filename)
{
    _m_saveIntermediateTiming=true;
    _m_intermediateTimingFilename=filename;
}

void GeneralizedMAAStarPlanner::SetVerbose(int verbose)
{
    _m_verboseness = verbose;
}        

boost::shared_ptr<JointPolicy> GeneralizedMAAStarPlanner::GetJointPolicy()
{ return(_m_foundPolicy); }
boost::shared_ptr<JointPolicyDiscrete> GeneralizedMAAStarPlanner::GetJointPolicyDiscrete()
{ return(_m_foundPolicy); }
JPDP_sharedPtr GeneralizedMAAStarPlanner::GetJointPolicyDiscretePure()
{ return(_m_foundPolicy); }

void GeneralizedMAAStarPlanner::SetDeadline(size_t deadlineInS)
{
    _m_deadline=deadlineInS;
    if(_m_verboseness>=0)
        cout << "GeneralizedMAAStarPlanner set deadline to " << _m_deadline
             << "s" << endl;
}
