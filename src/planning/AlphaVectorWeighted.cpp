/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "AlphaVectorWeighted.h"
#include "BeliefValue.h"
#include "PlanningUnitDecPOMDPDiscrete.h"

using namespace std;

#define DEBUG_AlphaVectorWeighted 0

//Default constructor
AlphaVectorWeighted::
AlphaVectorWeighted(const PlanningUnitDecPOMDPDiscrete* pu,
                    double weight) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    AlphaVectorBG(pu),
    _m_weights(GetPU()->GetNrStates(),weight)
{
    if(weight < 0 || weight > 1)
        throw(E("AlphaVectorWeighted ctor: weight should be >=0 and <=1"));
    BackProjectWeights();
}

AlphaVectorWeighted::
AlphaVectorWeighted(const PlanningUnitDecPOMDPDiscrete* pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    AlphaVectorBG(pu),
    _m_initialized(false)
{
}

AlphaVectorWeighted::
AlphaVectorWeighted(const PlanningUnitDecPOMDPDiscrete* pu,
                    std::vector<double> weights) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    AlphaVectorBG(pu),
    _m_initialized(false),
    _m_weights(weights)
{
    for(Index s=0;s!=GetPU()->GetNrStates();++s)
        if(_m_weights.at(s) < 0 || _m_weights.at(s) > 1)
            throw(E("AlphaVectorWeighted ctor: weights should be >=0 and <=1"));
    BackProjectWeights();
}
AlphaVectorWeighted::
AlphaVectorWeighted(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                    double weight) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    AlphaVectorBG(pu),
    _m_weights(GetPU()->GetNrStates(),weight)
{
    if(weight < 0 || weight > 1)
        throw(E("AlphaVectorWeighted ctor: weight should be >=0 and <=1"));
    BackProjectWeights();
}

AlphaVectorWeighted::
AlphaVectorWeighted(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    AlphaVectorBG(pu),
    _m_initialized(false)
{
}

AlphaVectorWeighted::
AlphaVectorWeighted(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu,
                    std::vector<double> weights) :
    AlphaVectorPlanning(pu),
    AlphaVectorPOMDP(pu),
    AlphaVectorBG(pu),
    _m_initialized(false),
    _m_weights(weights)
{
    for(Index s=0;s!=GetPU()->GetNrStates();++s)
        if(_m_weights.at(s) < 0 || _m_weights.at(s) > 1)
            throw(E("AlphaVectorWeighted ctor: weights should be >=0 and <=1"));
    BackProjectWeights();
}

//Destructor
AlphaVectorWeighted::~AlphaVectorWeighted()
{
}

void AlphaVectorWeighted::SetWeights(vector<double> weights)
{
    _m_weights=weights;
    BackProjectWeights();
}

AlphaVector
AlphaVectorWeighted::BeliefBackup(const JointBeliefInterface &b,
                                  Index a,
                                  const GaoVectorSet &G,
                                  const QFunctionsDiscrete &Q) const
{
    if(!_m_initialized)
        throw(E("AlphaVectorWeighted::BeliefBackup not initialized"));

    AlphaVector alphaPOMDP=AlphaVectorPOMDP::BeliefBackup(b,a,G);
    AlphaVector alphaBG=
        AlphaVectorBG::BeliefBackup(b,a,G,
                                    QFunctionsToValueFunction(Q),
                                    BGIP_SOLVER_EXHAUSTIVE);

    vector<double> valuesWeighted(alphaPOMDP.GetNrValues(),0);
    double w;
#if 0
    for(unsigned i=0;i!=alphaPOMDP.GetNrValues();++i)
        valuesWeighted[i]=(_m_weights[i] * alphaPOMDP.GetValue(i) +
                           (1-_m_weights[i]) * alphaBG.GetValue(i));
#else
    for(unsigned i=0;i!=alphaPOMDP.GetNrValues();++i)
    {
        w=_m_weightsBackProjected[a][i];
        valuesWeighted[i]=(w * alphaPOMDP.GetValue(i) +
                           (1-w) * alphaBG.GetValue(i));
    }
#endif
    AlphaVector alphaWeighted(alphaPOMDP.GetNrValues());
    alphaWeighted.SetValues(valuesWeighted);
    alphaWeighted.SetAction(a);
    alphaWeighted.SetBetaI(alphaBG.GetBetaI());

#if DEBUG_AlphaVectorWeighted
    double Vpomdp=BeliefValue::GetValue(b,alphaPOMDP),
        Vbg=BeliefValue::GetValue(b,alphaBG),
        Vweighted=BeliefValue::GetValue(b,alphaWeighted);

    cout << "AlphaVectorWeighted::BeliefBackup"
         << " for a " << a
#if 0
         << alphaWeighted.GetAction() << " bI "
         << alphaWeighted.GetBetaI() << " Vpomdp " << Vpomdp
         << " Vbg " << Vbg << " Vweighted " << Vweighted
#endif
         << " b " << b.SoftPrint() << " alpha " << alphaWeighted.SoftPrint()
         << endl;
#endif

    return(alphaWeighted);
}

void AlphaVectorWeighted::BackProjectWeights()
{
    size_t nrS=GetPU()->GetNrStates();

    _m_weightsBackProjected.clear();
    for(Index ja=0;ja!=GetPU()->GetNrJointActions();++ja)
    {
        vector<double> w(nrS,0);
        for(Index s0=0;s0!=nrS;++s0)
            for(Index s1=0;s1!=nrS;++s1)
                w[s0]+=GetPU()->GetTransitionProbability(s0,ja,s1)*
                    _m_weights[s1];

        _m_weightsBackProjected.push_back(w);
    }

    _m_initialized=true;
}

