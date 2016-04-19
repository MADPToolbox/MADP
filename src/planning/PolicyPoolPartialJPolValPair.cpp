/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */
#include <float.h>
#include "PolicyPoolPartialJPolValPair.h"
#include "PartialJointPolicyPureVector.h"
#define DEBUG_PPJPVP_ASSIGN 0

using namespace std;

//Default constructor
PolicyPoolPartialJPolValPair::PolicyPoolPartialJPolValPair()
{   
    _m_jpvpQueue_p = new priority_queue<PartialJPDPValuePair_sharedPtr>(); //&JPolValPool;    
}


//Copy constructor.    
//PolicyPoolPartialJPolValPair::PolicyPoolPartialJPolValPair(const PolicyPoolPartialJPolValPair& o) 
//{
    //throw E("PolicyPoolPartialJPolValPair /Copy constructor. is not (and never will be?) defined.");
//}
//Destructor
PolicyPoolPartialJPolValPair::~PolicyPoolPartialJPolValPair()
{
    while(_m_jpvpQueue_p->size() > 0)
    {
//        delete _m_jpvpQueue_p->top();
        _m_jpvpQueue_p->pop();
    }
    delete _m_jpvpQueue_p;
}
//Copy assignment operator
PolicyPoolPartialJPolValPair& PolicyPoolPartialJPolValPair::operator= 
    (const PolicyPoolPartialJPolValPair& o)
{
    if(DEBUG_PPJPVP_ASSIGN)
        cout <<"PolicyPoolPartialJPolValPair& PolicyPoolPartialJPolValPair::operator= \
called"<<endl;

    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...
    //
    // empty own queue 
    while(_m_jpvpQueue_p->size() > 0)
    {
//        delete _m_jpvpQueue_p->top();
        _m_jpvpQueue_p->pop();
    }
    *_m_jpvpQueue_p = *(o._m_jpvpQueue_p);

    return *this;
}
PartialPolicyPoolInterface& PolicyPoolPartialJPolValPair::operator= 
    (const PartialPolicyPoolInterface& o)
{
    if(DEBUG_PPJPVP_ASSIGN)
        cout <<"PartialPolicyPoolInterface& PolicyPoolPartialJPolValPair::operator= \
called."<<endl;
    if (this == &o) return *this;   // Gracefully handle self assignment
    const PolicyPoolPartialJPolValPair& casted_o = 
        dynamic_cast<const PolicyPoolPartialJPolValPair&>(o);

    return(operator=(casted_o));

}


void PolicyPoolPartialJPolValPair::Init(const Interface_ProblemToPolicyDiscretePure* pu)
{
    //start with a horizon 0 joint policy - i.e. specifying 0 actions
#if 0 // old code, now we can specify the depth in the constructors,
      // which prevents us from allocating huge vectors in case of
      // high horizons
    PartialJointPolicyPureVector* jpol_empty = 
        new PartialJointPolicyPureVector(pu, OHIST_INDEX, 0.0); 
    jpol_empty->SetDepth(0);
#else
    boost::shared_ptr<PartialJointPolicyPureVector> jpol_empty = 
        boost::shared_ptr<PartialJointPolicyPureVector>(
            new PartialJointPolicyPureVector(pu, OHIST_INDEX, 0.0, 0));
#endif
    PartialJPDPValuePair_sharedPtr jpv_empty =
        PartialJPDPValuePair_sharedPtr(new PartialJPDPValuePair(jpol_empty, DBL_MAX) ); 
    _m_jpvpQueue_p->push(jpv_empty);
    //JPolValPool_p->push(jpv_empty);
}

PartialPolicyPoolItemInterface_sharedPtr PolicyPoolPartialJPolValPair::Select() const
{
    if(_m_jpvpQueue_p->size() > 0)
    {
        PartialPolicyPoolItemInterface_sharedPtr ppi = 
            boost::dynamic_pointer_cast<PartialPolicyPoolItemInterface>(
                _m_jpvpQueue_p->top());
        return(ppi);
    }
    else
        throw E("Pool empty!");

}
void PolicyPoolPartialJPolValPair::Pop(PartialPolicyPoolItemInterface_sharedPtr ppiToBeRemoved)
{
    if(ppiToBeRemoved!=0 &&
       _m_jpvpQueue_p->top()!=ppiToBeRemoved)
            throw(E("PolicyPoolPartialJPolValPair::Pop not the correct ppi to be popped"));
    _m_jpvpQueue_p->pop();
}


void PolicyPoolPartialJPolValPair::Insert(PartialPolicyPoolItemInterface_sharedPtr ppi)
{
    PartialJPDPValuePair_sharedPtr jp = boost::dynamic_pointer_cast<PartialJPDPValuePair>(ppi);

    if(jp==0)
         throw(E("PolicyPoolPartialJPolValPair::Insert could not cast input to PartialJPDPValuePair"));
       
    _m_jpvpQueue_p->push(jp);
    
}

void PolicyPoolPartialJPolValPair::Union(PartialPolicyPoolInterface_sharedPtr  o_ppi)
{
    PolicyPoolPartialJPolValPair_sharedPtr o = boost::dynamic_pointer_cast<PolicyPoolPartialJPolValPair>(o_ppi);

    if(o==0)
        throw(E("PolicyPoolPartialJPolValPair::Union could not cast input to PolicyPoolPartialJPolValPair"));

    while(o->_m_jpvpQueue_p->size() > 0)
    {
        this->_m_jpvpQueue_p->push(o->_m_jpvpQueue_p->top());
        o->_m_jpvpQueue_p->pop();
    }
}

void PolicyPoolPartialJPolValPair::Prune(double v)
{
     priority_queue<PartialJPDPValuePair_sharedPtr> * new_jpvpQueue_p = 
         new priority_queue<PartialJPDPValuePair_sharedPtr>;

     while(_m_jpvpQueue_p->size() > 0)
     {
         PartialJPDPValuePair_sharedPtr jpvp = _m_jpvpQueue_p->top();
         if(jpvp->GetValue() > v)
         {
             new_jpvpQueue_p->push(jpvp);
         }
//          else
//              delete jpvp; // no longer necessary
         _m_jpvpQueue_p->pop();    
     }

     delete _m_jpvpQueue_p; //delete old queue
     _m_jpvpQueue_p = new_jpvpQueue_p; //point to new queue


}

string PolicyPoolPartialJPolValPair::SoftPrint() const
{
    stringstream ss;
    priority_queue<PartialJPDPValuePair_sharedPtr> *new_jpvpQueue_p = 
        new priority_queue<PartialJPDPValuePair_sharedPtr>(*_m_jpvpQueue_p);

    while(new_jpvpQueue_p->size() > 0)
    {
        PartialJPDPValuePair_sharedPtr jpvp = new_jpvpQueue_p->top();
        ss << jpvp->SoftPrint() << endl;

        new_jpvpQueue_p->pop();    
    }

    delete new_jpvpQueue_p;
    return(ss.str());
}

