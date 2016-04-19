/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JointBeliefSparse.h"
#include "TransitionModelDiscrete.h"
#include "ObservationModelDiscrete.h"
#include "TGet.h"
#include <float.h>

using namespace std;

//Necessary as header file contains a forward declaration:
#include "MultiAgentDecisionProcessDiscreteInterface.h" 

#define JointBeliefSparse_doSanityCheckAfterEveryUpdate 0

JointBeliefSparse::JointBeliefSparse()
{
}

JointBeliefSparse::JointBeliefSparse(size_t size) :
    BeliefSparse(size)
{
}

JointBeliefSparse::JointBeliefSparse(const vector<double> &belief) :
    BeliefSparse(belief)
{
}

JointBeliefSparse::JointBeliefSparse(const JointBeliefInterface &belief) :
    BeliefSparse(belief)
{
}

JointBeliefSparse::JointBeliefSparse(const StateDistribution& belief) :
    BeliefSparse(belief)
{
}
//Destructor
JointBeliefSparse::~JointBeliefSparse()
{
}

JointBeliefSparse& 
JointBeliefSparse::operator= (const JointBeliefSparse& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    BeliefSparse::operator=(o);
    return *this;
}

JointBeliefInterface& 
JointBeliefSparse::operator= (const JointBeliefInterface& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    const JointBeliefSparse& casted_o = 
        dynamic_cast<const JointBeliefSparse&>(o);
    return(operator=(casted_o));// call the operator= for JointBeliefSparse
}
    
double JointBeliefSparse::Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                                 Index lastJAI, Index newJOI)
{
    //const TransitionModelDiscrete* T=pu.GetTransitionModelDiscretePtr();
    const ObservationModelDiscrete* O=pu.GetObservationModelDiscretePtr();

    //pointer to the transition probability Get funtion:
    TGet* T = pu.GetTGet();
    if(T==0)
    {
        return(UpdateSlow(pu,lastJAI,newJOI));
    }

    double Po_ba = 0.0; // P(o|b,a) with o=newJO
    double Ps_ba, Po_as, Pso_ba;
    size_t nrS = _m_b.size();
    BS newJB_unnorm(nrS);
    bool isEventDriven = pu.GetEventObservability();
    for(Index sI=0; sI < nrS; sI++)
    {
        Pso_ba = 0;
        
        if(!isEventDriven)
        {
            //P(sI | b, a) = sum_(prec_s) P(sI | prec_s, a)*JB(prec_s)
            Ps_ba = 0.0;

            for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                Ps_ba += T->Get(it.index(), lastJAI, sI) * *it;

            if(Ps_ba>0) // if it is zero, Pso_ba will be zero anyway
            {
                //P(newJOI | lastJAI, sI) :
                Po_as = O->Get(lastJAI, sI, newJOI);

                //the new (unormalized) belief P(s,o|b,a)
                Pso_ba = Po_as * Ps_ba;
            }
        }
        else
        {
            for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                //P(sI, newJOI | b, a) = sum_(prec_s) P(newJOI | prec_sI, lastJAI, sI)*P(sI | prec_s, a)* JB(prec_s)
                Pso_ba += O->Get(it.index(), lastJAI, sI, newJOI) * 
                          T->Get(it.index(), lastJAI, sI) * *it;
        }
        
        if(Pso_ba>PROB_PRECISION) // we don't want to store very
                                  // small probabilities in a
                                  // sparse representation
        {
            newJB_unnorm[sI]=Pso_ba; //unnormalized new belief
            Po_ba += Pso_ba; //running sum of P(o|b,a)
        }
    }
    
    //normalize:    
    if(Po_ba>0)
        for(BSit it=newJB_unnorm.begin(); it!=newJB_unnorm.end(); ++it)
            *it/=Po_ba;

    _m_b=newJB_unnorm;

#if JointBeliefSparse_doSanityCheckAfterEveryUpdate
    if(!SanityCheck())
        throw(E("JointBeliefSparse::Update SanityCheck failed"));
#endif

    return(Po_ba);
}

/** Almost literal copy of Update(). */
double JointBeliefSparse::UpdateSlow(const MultiAgentDecisionProcessDiscreteInterface &pu,
                                     Index lastJAI, Index newJOI)
{
    double Po_ba = 0.0; // P(o|b,a) with o=newJO
    double Ps_ba, Po_as, Pso_ba;
    size_t nrS = _m_b.size();
    BS newJB_unnorm(nrS);
    bool isEventDriven = pu.GetEventObservability();
    for(Index sI=0; sI < nrS; sI++)
    {
        Pso_ba = 0;
        
        if(!isEventDriven)
        {
            //P(sI | b, a) = sum_(prec_s) P(sI | prec_s, a)*JB(prec_s)
            Ps_ba = 0.0;

            for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                Ps_ba += pu.GetTransitionProbability(it.index(), lastJAI, sI) * *it;
            
            if(Ps_ba>0) // if it is zero, Pso_ba will be zero anyway
            {
                //P(newJOI | lastJAI, sI) :
                Po_as = pu.GetObservationProbability(lastJAI, sI, newJOI);

                //the new (unormalized) belief P(s,o|b,a)
                Pso_ba = Po_as * Ps_ba;
            }
        }
        else
        {
            for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                //P(sI, newJOI | b, a) = sum_(prec_s) P(newJOI | prec_sI, lastJAI, sI)*P(sI | prec_s, a)* JB(prec_s)
                Pso_ba += pu.GetObservationProbability(it.index(), lastJAI, sI, newJOI) * 
                          pu.GetTransitionProbability(it.index(), lastJAI, sI) * *it;
        }
        
        if(Pso_ba>PROB_PRECISION) // we don't want to store very
                                  // small probabilities in a
                                  // sparse representation
        {
            newJB_unnorm[sI]=Pso_ba; //unnormalized new belief
            Po_ba += Pso_ba; //running sum of P(o|b,a)
        }
    }
    
    //normalize:    
    if(Po_ba>0)
        for(BSit it=newJB_unnorm.begin(); it!=newJB_unnorm.end(); ++it)
            *it/=Po_ba;

    _m_b=newJB_unnorm;

#if JointBeliefSparse_doSanityCheckAfterEveryUpdate
    if(!SanityCheck())
        throw(E("JointBeliefSparse::UpdateSlow SanityCheck failed"));
#endif

    return(Po_ba);
}
