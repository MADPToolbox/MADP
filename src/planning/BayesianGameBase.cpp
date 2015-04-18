/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

#include "BayesianGameBase.h"
#include "E.h"

using namespace std;

//Default constructor
BayesianGameBase::BayesianGameBase()
    : 
        _m_jTypeProbs(0)
        ,_m_jTypeProbsSparse(0)
        ,_m_verboseness(0)
{
    _m_initialized=false;
    _m_nrAgents = 0;
    _m_nrJTypes = 0;
    _m_nrJA = 0;
    _m_JAoverflow = false;
    _m_JToverflow = false;
    _m_stepSizeActions = 0;
    _m_stepSizeTypes = 0;
    _m_jointToIndTypes = 0;
    _m_jointToIndTypesMap = 0;
}

BayesianGameBase::BayesianGameBase(
        const size_t nrAgents, 
        const vector<size_t> & nrActions,  
        const vector<size_t> & nrTypes,
        int verb)
    :       
        _m_jointToIndTypes(0)
        ,_m_jointToIndTypesMap(0)
        ,_m_verboseness(verb)
        ,_m_JAoverflow(false)
        ,_m_JToverflow(false)
{
    if( nrActions.size() != nrAgents || nrTypes.size() != nrAgents)
        throw E("Dimension mismatches in creating a Bayesian game");

    _m_stepSizeActions = 0;
    _m_stepSizeTypes = 0;
    _m_nrAgents = nrAgents;
    _m_nrActions = nrActions;
    _m_nrTypes = nrTypes;

    Initialize();
}
void BayesianGameBase::ChangeNrActions(Index agI, size_t new_nr)
{
    _m_nrActions.at(agI) = new_nr;
    Initialize();
}
void BayesianGameBase::ChangeNrTypes(Index agI, size_t new_nr)
{
    _m_nrTypes.at(agI) = new_nr;
    Initialize();
}

void BayesianGameBase::Initialize()
{
    if(_m_stepSizeTypes)
        delete[] _m_stepSizeTypes;
    if(_m_stepSizeActions)
        delete[] _m_stepSizeActions;
    _m_stepSizeTypes = IndexTools::CalculateStepSize(_m_nrTypes);
    _m_stepSizeActions = IndexTools::CalculateStepSize(_m_nrActions);

    //calculate # joint types and actions
    _m_nrJTypes = 1;
    _m_nrJA = 1;
    for(Index i = 0; i < _m_nrAgents; i++)
    {
        size_t old_JT = _m_nrJTypes;
        _m_nrJTypes *= _m_nrTypes[i];
        if (old_JT > _m_nrJTypes)
            _m_JToverflow = true;
            //throw EOverflow("BayesianGameBase::Number of joint types are overflowing");

        size_t old_JA = _m_nrJA;
        _m_nrJA *= _m_nrActions[i];
        if (old_JA > _m_nrJA)
            _m_JAoverflow = true;
            //throw EOverflow("BayesianGameBase::Number of joint actions are overflowing");
    }

    if(_m_nrJTypes>1e5 || _m_JToverflow)
    {
        cout << "BayesianGameBase: using sparse models" << endl;
        _m_useSparse=true;
    }
    else
    {
        _m_useSparse=false;
        try {
            _m_jTypeProbs = vector<double>(_m_nrJTypes, 0.0);
            _m_jointToIndTypes = new vector<vector<Index> >(_m_nrJTypes);
        } catch(std::bad_alloc)
        {
            cout << "BayesianGameBase: too many joint types ("
                 << _m_nrJTypes
                 << ") to have a full vector in memory, switching to sparse models..." << endl;
            _m_useSparse=true;
        }
    }

    //initialize the probability distribution over joint types:
    if(_m_useSparse)
    {
        _m_jTypeProbsSparse.resize(_m_nrJTypes);
        _m_jTypeProbsSparse.clear();
        _m_jointToIndTypesMap = new map<Index, vector<Index> >();
        _m_jointToIndTypes=0;
    }
    else
    {
        _m_jointToIndTypesMap=0;
    }


    // call this as it checks whether the joint policy indices will be
    // overflowing, resulting in an exception
    try {
        GetNrJointPolicies();
    }
    catch(EOverflow& e){ 
        cout << "Warning, joint policy indices are overflowing" << endl;
    }
    if(_m_JToverflow)
        cout << "Warning, joint types are overflowing" << endl;
    if(_m_JAoverflow)
        cout << "Warning, joint actions are overflowing" << endl;

    _m_initialized=true;
}

//Copy constructor.    
BayesianGameBase::BayesianGameBase(const BayesianGameBase& o) :
    _m_useSparse(o._m_useSparse),
    _m_jTypeProbs(o._m_jTypeProbs),
    _m_jTypeProbsSparse(o._m_jTypeProbsSparse),
    _m_jointToIndTypes(0),
    _m_jointToIndTypesMap(0),
    _m_initialized(o._m_initialized),
    _m_verboseness(o._m_verboseness),
    _m_nrAgents(o._m_nrAgents),
    _m_nrActions(o._m_nrActions),
    _m_nrTypes(o._m_nrTypes),
    _m_nrJTypes(o._m_nrJTypes),
    _m_nrJA(o._m_nrJA)
{
    if(o._m_jointToIndTypes)
        _m_jointToIndTypes=new vector<vector<Index> >(*o._m_jointToIndTypes);
    if(o._m_jointToIndTypesMap)
        _m_jointToIndTypesMap=new map<Index, vector<Index> >(*o._m_jointToIndTypesMap);
        
    _m_stepSizeTypes = IndexTools::CalculateStepSize(_m_nrTypes);
    _m_stepSizeActions = IndexTools::CalculateStepSize(_m_nrActions);

}

BayesianGameBase& BayesianGameBase::operator= (const BayesianGameBase& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment

    Interface_ProblemToPolicyDiscretePure::operator=(o);

    _m_useSparse=o._m_useSparse;
    _m_jTypeProbs=o._m_jTypeProbs;
    _m_jTypeProbsSparse=o._m_jTypeProbsSparse;

    if(o._m_jointToIndTypes)
        _m_jointToIndTypes=new vector<vector<Index> >(*o._m_jointToIndTypes);
    else
        _m_jointToIndTypes=0;
    if(o._m_jointToIndTypesMap)
        _m_jointToIndTypesMap=new map<Index, vector<Index> >(*o._m_jointToIndTypesMap);
    else
        _m_jointToIndTypesMap=0;

    _m_initialized=o._m_initialized;
    _m_verboseness=o._m_verboseness;
    _m_nrAgents=o._m_nrAgents;
    _m_nrActions=o._m_nrActions;
    _m_nrTypes=o._m_nrTypes;
    _m_nrJTypes=o._m_nrJTypes;
    _m_nrJA=o._m_nrJA;
    _m_stepSizeTypes = IndexTools::CalculateStepSize(_m_nrTypes);
    _m_stepSizeActions = IndexTools::CalculateStepSize(_m_nrActions);
    
    return *this;
}

//Destructor
BayesianGameBase::~BayesianGameBase()
{
    delete [] _m_stepSizeTypes;
    delete [] _m_stepSizeActions;
    delete _m_jointToIndTypes;
    delete _m_jointToIndTypesMap;
}

bool BayesianGameBase::SetInitialized(bool b)
{
    _m_initialized = b;
    return(true);
}

size_t BayesianGameBase::GetNrJointActions() const 
{
    if (_m_JAoverflow)
        throw EOverflow("BayesianGameBase::GetNrJointActions - Not available, because number of joint actions are overflowing");
    return _m_nrJA;
}
size_t BayesianGameBase::GetNrJointTypes() const 
{
    if (_m_JToverflow)
        throw EOverflow("BayesianGameBase::GetNrJointTypes - Not available, because number of joint types are overflowing");
    return _m_nrJTypes;
}


double BayesianGameBase::GetProbability(Index i) const
{
    //cout << "BayesianGameBase::GetProbability called"<<endl;
    if (_m_useSparse) return(_m_jTypeProbsSparse[i]);
    else  return(_m_jTypeProbs[i]);
}







string BayesianGameBase::SoftPrint() const
{
    stringstream ss;
    ss << "Bayesian game with "<<_m_nrAgents<<" agents"<<endl;
    ss << "Number of actions ";
    ss << SoftPrintVector(_m_nrActions);
    ss << " ("<< _m_nrJA <<" joint actions)";
    ss << "\nNumber of types ";
    ss << SoftPrintVector(_m_nrTypes);
    ss << " ("<< _m_nrJTypes <<" joint types)"<<endl;
    ss << "joint type probs ";
    if(_m_useSparse)
        ss << "(sparse) " << SoftPrintVector(_m_jTypeProbsSparse);
    else
        ss << SoftPrintVector(_m_jTypeProbs);
    ss << endl;
    return(ss.str());
}

string BayesianGameBase::SoftPrintType(Index agentI, Index typeIndex) const
{
    stringstream ss;
    ss << "BGType"<<typeIndex;
    return(ss.str());
}
string BayesianGameBase::SoftPrintPolicyDomainElement(Index agentI, 
        Index typeIndex,
        PolicyGlobals::PolicyDomainCategory cat) const
{
    if(cat != PolicyGlobals::TYPE_INDEX)
        throw E("BGs only work with types as the policy domain");
    return SoftPrintType(agentI, typeIndex);
}

string BayesianGameBase::SoftPrintAction(Index agentI, Index actionI) const
{
    stringstream ss;
    ss << "BGAction"<<actionI;
    return(ss.str());
}

bool BayesianGameBase::AreCachedJointToIndivIndices(
    const PolicyGlobals::PolicyDomainCategory pdc) const 
{
    switch ( pdc )
    {
    case PolicyGlobals::OHIST_INDEX :
        return CacheJointToIndivOH_Indices();
        break;
    case PolicyGlobals::AOHIST_INDEX :
        return CacheJointToIndivAOH_Indices();
        break;
    case PolicyGlobals::TYPE_INDEX : 
        return CacheJointToIndivType_Indices();
        break;
    default:
        throw(E("Used a JointPolicyDiscrete::PolicyGlobals::PolicyDomainCategory unknown to BayesianGameBase!"));
        break;
    }				/* -----  end switch  ----- */
};

PolicyGlobals::PolicyDomainCategory BayesianGameBase::GetDefaultIndexDomCat()
    const
{
    return PolicyGlobals::TYPE_INDEX;
}

size_t BayesianGameBase::GetNrPolicyDomainElements(
    Index agentI, 
    PolicyGlobals::PolicyDomainCategory cat,
    size_t depth) const
{
#if 0
    if(depth!=MAXHORIZON)
        throw(E("BayesianGameBase::GetNrPolicyDomainElements depth argument not supported for BGs"));
#endif
    return _m_nrTypes[agentI];
}

void
BayesianGameBase::SanityCheckBGBase()
{
    double p=0.0;
    if (_m_JToverflow)
    {
        cout << "\n!!! BayesianGameBase::SanityCheck() aborted since the joint types are overflowing. BayesianGameBase really is ill-suited for this situation and a new implementation needs to be created for this case!!! Only leaving this now such that we can run experiments for the JAAMAS paper...\n"<< endl;
        return;
    }

    if(_m_useSparse)
    {
        for( SparseVector::const_iterator it = _m_jTypeProbsSparse.begin(); 
                it != _m_jTypeProbsSparse.end();
                it++)
            p += *it;
        //throw E("BayesianGameBase::SanityCheck() not yet implemented for sparse stuff");

    }
    else
    {
        for(Index jtI=0; jtI < _m_jTypeProbs.size(); jtI++)
            p += GetProbability(jtI);
    }

    if(! Globals::EqualProbability(p, 1.0) )
    {
        stringstream ss;
        ss << "BayesianGameBase::SanityCheck() Warning, total probability of joint types does not sum to 1.0, "
            << "but to "<<setprecision(16) << p<<"! ";
        if(  abs(p - 1.0) < 1e-9  )
        {
            ss << "Since the difference is < 1e-9, we should just renormalize... (not yet implemented...)"<<endl;
            cerr << ss.str();
        }
        else
            throw(E(ss));
    }
    return;
}

