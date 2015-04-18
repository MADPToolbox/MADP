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
#include <float.h>
#include "PolicyPoolJPolValPair.h"
#include "JPPVValuePair.h"
#include "JointPolicyPureVector.h"
#define DEBUG_PPJPVP_ASSIGN 0

using namespace std;

//Default constructor
PolicyPoolJPolValPair::PolicyPoolJPolValPair()
{   
    _m_jpvpQueue_p = new priority_queue<JointPolicyValuePair_sharedPtr>(); //&JPolValPool;    
}

//Destructor
PolicyPoolJPolValPair::~PolicyPoolJPolValPair()
{
    while(_m_jpvpQueue_p->size() > 0)
    {
//        delete _m_jpvpQueue_p->top();
        _m_jpvpQueue_p->pop();
    }
    delete _m_jpvpQueue_p;
}
//Copy assignment operator
PolicyPoolJPolValPair& PolicyPoolJPolValPair::operator= 
    (const PolicyPoolJPolValPair& o)
{
    if(DEBUG_PPJPVP_ASSIGN)
        cout <<"PolicyPoolJPolValPair& PolicyPoolJPolValPair::operator= \
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
PolicyPoolInterface& PolicyPoolJPolValPair::operator= 
    (const PolicyPoolInterface& o)
{
    if(DEBUG_PPJPVP_ASSIGN)
        cout <<"PolicyPoolInterface& PolicyPoolJPolValPair::operator= \
called."<<endl;
    if (this == &o) return *this;   // Gracefully handle self assignment
    const PolicyPoolJPolValPair& casted_o = 
        dynamic_cast<const PolicyPoolJPolValPair&>(o);

    return(operator=(casted_o));

}


void PolicyPoolJPolValPair::Init(const Interface_ProblemToPolicyDiscretePure* pu)
{
    //start with a horizon 0 joint policy - i.e. specifying 0 actions
    JPPV_sharedPtr jpol_empty = 
        JPPV_sharedPtr(
            new JointPolicyPureVector(pu, OHIST_INDEX)); 
    jpol_empty->SetDepth(0);
    JointPolicyValuePair_sharedPtr jpv_empty =
        JointPolicyValuePair_sharedPtr(new JPPVValuePair(jpol_empty, DBL_MAX)); 
    _m_jpvpQueue_p->push(jpv_empty);
    //JPolValPool_p->push(jpv_empty);
}

PolicyPoolItemInterface_sharedPtr PolicyPoolJPolValPair::Select() const
{
    if(_m_jpvpQueue_p->size() > 0)
    {
        PolicyPoolItemInterface_sharedPtr ppi = 
            boost::dynamic_pointer_cast<PolicyPoolItemInterface>( 
                _m_jpvpQueue_p->top());
        return(ppi);
    }
    else
        throw E("Pool empty!");

}
void PolicyPoolJPolValPair::Pop()
{
    _m_jpvpQueue_p->pop();
}


void PolicyPoolJPolValPair::Insert(PolicyPoolItemInterface_sharedPtr  ppi)
{
    JointPolicyValuePair_sharedPtr jp = boost::dynamic_pointer_cast<JointPolicyValuePair>(ppi);

    if(jp==0)
        throw(E("PolicyPoolJPolValPair::Insert could not cast input to JointPolicyValuePair"));

    _m_jpvpQueue_p->push(jp);
    
}

void PolicyPoolJPolValPair::Union(PolicyPoolInterface_sharedPtr  o_ppi)
{
    PolicyPoolJPolValPair_sharedPtr o = boost::dynamic_pointer_cast<PolicyPoolJPolValPair>(o_ppi);

    if(o==0)
        throw(E("PolicyPoolJPolValPair::Union could not cast input to PolicyPoolJPolValPair"));

    while(o->_m_jpvpQueue_p->size() > 0)
    {
        this->_m_jpvpQueue_p->push(o->_m_jpvpQueue_p->top());
        o->_m_jpvpQueue_p->pop();
    }
}

void PolicyPoolJPolValPair::Prune(double v)
{
     priority_queue<JointPolicyValuePair_sharedPtr> * new_jpvpQueue_p = 
         new priority_queue<JointPolicyValuePair_sharedPtr>;

     while(_m_jpvpQueue_p->size() > 0)
     {
         JointPolicyValuePair_sharedPtr jpvp = _m_jpvpQueue_p->top();
         if(jpvp->GetValue() > v)
         {
             new_jpvpQueue_p->push(jpvp);
         }
         _m_jpvpQueue_p->pop();    
     }

     delete _m_jpvpQueue_p; //delete old queue
     _m_jpvpQueue_p = new_jpvpQueue_p; //point to new queue
}
