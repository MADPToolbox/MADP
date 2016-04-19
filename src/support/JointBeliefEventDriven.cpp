/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Joao Messias 
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "JointBeliefEventDriven.h"
#include <float.h>
//Necessary as header file contains a forward declaration:
#include "MultiAgentDecisionProcessDiscreteInterface.h" 
#include <typeinfo>

#include "TransitionModelMapping.h"
#include "EventObservationModelMapping.h"

using namespace std;

#define JointBeliefEventDriven_doSanityCheckAfterEveryUpdate 0
#define JointBeliefEventDriven_hardCap 10 ///Maximum number of iterations. Limits computational time in large systems in exchange for accuracy.
#define JointBeliefEventDriven_softCap 10 ///Protection against badly conditioned systems.
#define JointBeliefEventDriven_convergenceTh 0.01

JointBeliefEventDriven::JointBeliefEventDriven(size_t size, int falseNegativeObs) :
  Belief(size),
  _m_falseNegativeObs(falseNegativeObs)
{
}

JointBeliefEventDriven::JointBeliefEventDriven(const vector<double> &belief, int falseNegativeObs) :
    Belief(belief),
    _m_falseNegativeObs(falseNegativeObs)
{
}

JointBeliefEventDriven::JointBeliefEventDriven(const JointBeliefInterface &belief, int falseNegativeObs) :
    Belief(belief),
    _m_falseNegativeObs(falseNegativeObs)
{
}
JointBeliefEventDriven::JointBeliefEventDriven(const StateDistribution& belief, int falseNegativeObs) :
    Belief(belief),
    _m_falseNegativeObs(falseNegativeObs)
{}

//Destructor
JointBeliefEventDriven::~JointBeliefEventDriven()
{
}

JointBeliefEventDriven& 
JointBeliefEventDriven::operator= (const JointBeliefEventDriven& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment

    Belief::operator=(o);

    return(*this);
}

JointBeliefInterface& 
JointBeliefEventDriven::operator= (const JointBeliefInterface& o)
{
    //this code is called when we perform 
    // jb1 = jb2
    // and jb1 is a JointBeliefEventDriven (I.e., this is the operator= function of jb1
    // and *this* is a JointBeliefEventDriven, since we got here)
    //
    // Therefore this code assumes that jb2 (I.e., 'o') is also a JointBeliefEventDriven 
    // (and not a JointBeliefEventDrivenSparse or something like that).
    //
    // If it is something else, we get in trouble here!
    if (this == &o) return *this;   // Gracefully handle self assignment
    try{
        const JointBeliefEventDriven& casted_o = dynamic_cast<const JointBeliefEventDriven&>(o);
        return(operator=(casted_o));// call the operator= for JointBeliefEventDriven
    }catch(std::bad_cast bc){
        throw E("JointBeliefEventDriven::operator= bad_cast exception. Are you trying to assign me (a JointBeliefEventDriven) with some other JointBeliefInterface (e.g. JointBelief) ??");
    }

}

double JointBeliefEventDriven::Update(const MultiAgentDecisionProcessDiscreteInterface &pu,
                                      Index lastJAI, Index newJOI)
{
    using namespace boost::numeric::ublas;

    if(!pu.GetEventObservability())
        throw E("Trying to use a JointBeliefEventDriven on a model without event observability.");

    size_t nrS = pu.GetNrStates();
    
    const TransitionModelMapping *T = dynamic_cast<const TransitionModelMapping *>(pu.GetTransitionModelDiscretePtr());
    const EventObservationModelMapping *O = dynamic_cast<const EventObservationModelMapping *>(pu.GetObservationModelDiscretePtr());
    double eta = 0;
    if(T != 0 && O != 0)
    {
        using namespace boost::numeric::ublas;
      
        //we need to break the abstraction here since we'll be using ublas over T and O.
        //Ideally, we could have some kind of built in conversion.
        const matrix<double>* Ta = T->GetMatrixPtr(lastJAI);
        const matrix<double>* Oao = O->GetMatrixPtr(lastJAI,newJOI);
        matrix<double>* Hao = new matrix<double>(element_prod(*Ta,*Oao));
      
        matrix<double> H = *Hao;
      
        double epsilon, last_epsilon = 0;
      
        //We need b in ublas form.
        boost::numeric::ublas::vector<double> b(nrS);
        std::copy(_m_b.begin(), _m_b.end(), b.begin());

        boost::numeric::ublas::vector<double> res = zero_vector<double>(nrS);
        boost::numeric::ublas::vector<double> bUpdated = prod(b,H);
        epsilon = JointBeliefEventDriven_convergenceTh;
        if(_m_falseNegativeObs >= 0)
        {
            const matrix<double>* Oaf = O->GetMatrixPtr(lastJAI,_m_falseNegativeObs);
            //At this point we have two |S|x|S| matrices
            //What we want to do is; 
            //Hao = Ta.*Oao;
            //Haf = Ta.*Oaf;
            //b' = (b^T * (inv(I - Haf) * Hao))/n
            //Since we can't compute the inverse of arbitrarily large matrices, we have to approximate this
            //by cutting off the respective power series (I + Haf + Haf^2 + Haf^3 + ...) at some point.
            //Since Ta, Oao, Oaf are defined over (s,s') and we want to avoid having to transpose,
            //we have to left-multiply this by b^T to get the update
            //We also cannot rely on determinant / trace, so we have to keep track of b' and see if it is converging.

            matrix<double>* Haf = new matrix<double>(element_prod(*Ta,*Oaf));
            for(int safety_s=0, safety_h=0; 
                epsilon >= JointBeliefEventDriven_convergenceTh && safety_h < JointBeliefEventDriven_hardCap;
                safety_h++)
            {
                noalias(H) = prod(*Haf, H); //remember that we are implicitly using transposes.
                res = prod(b,H);
              
                bUpdated = bUpdated + res;
                last_epsilon = epsilon;
                if(sum(bUpdated) > 0)
                    epsilon = sum(res);
              
                if(last_epsilon != 0 && epsilon >= last_epsilon){
                    safety_s++;
                }else{
                    safety_s=0;
                }
                if(safety_s >= JointBeliefEventDriven_softCap){
                    cout << "Warning: Event-driven belief update not converging! - The system has non-observable chains." << endl;
                    break;
                }
            }
      
            delete Haf;
        }
        //only the normalization factor remains
        eta = sum(bUpdated);

        if(eta > 0)
        {
            bUpdated = bUpdated / eta;
      
            _m_b.clear();
            _m_b.insert(_m_b.end(), bUpdated.begin(), bUpdated.end());
        }
        else
        {
            cout << "Event-driven belief update failed: The system is badly conditioned. There is 0 probability of observing this symbol given this belief." << endl;
        }  
      
        delete Hao;      
    }
    
#if JointBeliefEventDriven_doSanityCheckAfterEveryUpdate
    if(!SanityCheck())
        throw(E("JointBeliefEventDriven::Update SanityCheck failed"));
#endif

    return(eta);
}
