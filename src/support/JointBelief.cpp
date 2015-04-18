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

#include "JointBelief.h"
#include <float.h>
//Necessary as header file contains a forward declaration:
#include "MultiAgentDecisionProcessDiscreteInterface.h" 
#include <typeinfo>

#include "TGet.h"

using namespace std;

#define JointBelief_doSanityCheckAfterEveryUpdate 0

JointBelief::JointBelief(size_t size) :
    Belief(size)
{
}

JointBelief::JointBelief(const vector<double> &belief) :
    Belief(belief)
{
}

JointBelief::JointBelief(const JointBeliefInterface &belief) :
    Belief(belief)
{
}
JointBelief::JointBelief(const StateDistribution& belief):
    Belief(belief)
{}

//Destructor
JointBelief::~JointBelief()
{
}

JointBelief& 
JointBelief::operator= (const JointBelief& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment

    Belief::operator=(o);

    return(*this);
}

JointBeliefInterface& 
JointBelief::operator= (const JointBeliefInterface& o)
{
    //this code is called when we perform 
    // jb1 = jb2
    // and jb1 is a JointBelief (I.e., this is the operator= function of jb1
    // and *this* is a JointBelief, since we got here)
    //
    // Therefore this code assumes that jb2 (I.e., 'o') is also a JointBelief 
    // (and not a JointBeliefSparse or something like that).
    //
    // If it is something else, we get in trouble here!
    if (this == &o) return *this;   // Gracefully handle self assignment
    try{
        const JointBelief& casted_o = dynamic_cast<const JointBelief&>(o);
        return(operator=(casted_o));// call the operator= for JointBelief
    }catch(std::bad_cast bc){
        throw E("JointBelief::operator= bad_cast exception. Are you trying to assign me (a JointBelief) with some other JointBeliefInterface (e.g. JointBeliefSparse) ??");
    }

}

double JointBelief::Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                           Index lastJAI, Index newJOI)
{
    double Po_ba = 0.0; // P(o|b,a) with o=newJO
    vector<double> newJB_unnorm;
    size_t nrS = pu.GetNrStates();
    
    TGet* T = 0;
    T = pu.GetTGet();
    if(T != 0)
    {
        for(Index sI=0; sI < nrS; sI++)
        {
            double Pso_ba = 0.0;
            if(!pu.GetEventObservability())
            {
                //P(newJOI | lastJAI, sI) :
                double Po_as = pu.GetObservationProbability(lastJAI, sI, newJOI);
                //P(sI | b, a) = sum_(prec_s) P(sI | prec_s, a)*JB(prec_s)
                double Ps_ba = 0.0;
                //for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                //Ps_ba += T->Get(it.index(), lastJAI, sI) * *it;
                for(Index prec_sI=0; prec_sI < nrS; prec_sI++)
                    Ps_ba += T->Get(prec_sI, lastJAI, sI) * _m_b[prec_sI];
                //Ps_ba += pu.GetTransitionProbability(prec_sI, lastJAI, sI) * 
                //_m_b[prec_sI];

                //the new (unormalized) belief P(s,o|b,a)
                Pso_ba = Po_as * Ps_ba;
            }
            else
            {
                for(Index prec_sI=0; prec_sI < nrS; prec_sI++)
                {
                    //P(newJOI | prec_sI, lastJAI, sI) :
                    double Po_sas = pu.GetObservationProbability(prec_sI, lastJAI, sI, newJOI);
                    //P(sI, newJOI | b, a) = sum_(prec_s) P(newJOI | prec_sI, lastJAI, sI)*P(sI | prec_s, a)* JB(prec_s)
                    Pso_ba += Po_sas * T->Get(prec_sI, lastJAI, sI) * _m_b[prec_sI];
                }
            }
            newJB_unnorm.push_back(Pso_ba); //unormalized new belief
            Po_ba += Pso_ba; //running sum of P(o|b,a)
        }
    }
    else 
    {
        for(Index sI=0; sI < nrS; sI++)
        {
            double Pso_ba = 0.0;
            if(!pu.GetEventObservability())
            {
                //P(newJOI | lastJAI, sI) :
                double Po_as = pu.GetObservationProbability(lastJAI, sI, newJOI);
                //P(sI | b, a) = sum_(prec_s) P(sI | prec_s, a)*JB(prec_s)
                double Ps_ba = 0.0;
                //for(BScit it=_m_b.begin(); it!=_m_b.end(); ++it)
                //Ps_ba += T->Get(it.index(), lastJAI, sI) * *it;
                for(Index prec_sI=0; prec_sI < nrS; prec_sI++)
                  Ps_ba += pu.GetTransitionProbability(prec_sI, lastJAI, sI) * 
                    _m_b[prec_sI];
                
                //the new (unormalized) belief P(s,o|b,a)
                Pso_ba = Po_as * Ps_ba;
            }
            else
            {
                for(Index prec_sI=0; prec_sI < nrS; prec_sI++)
                {
                    //P(newJOI | prec_sI, lastJAI, sI) :
                    double Po_sas = pu.GetObservationProbability(prec_sI, lastJAI, sI, newJOI);
                    //P(sI, newJOI | b, a) = sum_(prec_s) P(newJOI | prec_sI, lastJAI, sI)*P(sI | prec_s, a)* JB(prec_s)
                    Pso_ba += Po_sas * pu.GetTransitionProbability(prec_sI, lastJAI, sI) * _m_b[prec_sI];
                }
            }                
            newJB_unnorm.push_back(Pso_ba); //unormalized new belief
            Po_ba += Pso_ba; //running sum of P(o|b,a)
        }
        //throw E("JointBelief::Update tried to obtain a TGet, but apparently the transition model is not cached? (it should be, since this belief update loops over all states it should be possible to cache the transition model)");
    }
   
    //normalize:
    if(Po_ba>0)
        for(Index sI=0; sI < nrS; sI++)
            _m_b[sI]=newJB_unnorm[sI]/Po_ba;

    delete T;

#if JointBelief_doSanityCheckAfterEveryUpdate
    if(!SanityCheck())
        throw(E("JointBelief::Update SanityCheck failed"));
#endif

    return(Po_ba);
}
