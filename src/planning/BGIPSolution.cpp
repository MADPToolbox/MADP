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

#include "BGIPSolution.h"
#include <fstream>
#include "PartialJointPolicyDiscretePure.h"
#include "EOverflow.h"
#include "JPPVValuePair.h"
#include "PartialJPDPValuePair.h"
#include "boost/pointer_cast.hpp"
#include <float.h>
#include "boost/make_shared.hpp"

using namespace std;

#define DBGIPSOL_LEAK 0

//Default constructor
BGIPSolution::BGIPSolution(
        const Interface_ProblemToPolicyDiscretePure *pu,
        size_t nrDesiredSolutions
        ) :
    _m_pu(pu),
    _m_nrDesiredSolutions(nrDesiredSolutions),
    _m_qFixedK(FixedCapacityPriorityQueue<boost::shared_ptr<JPPVValuePair> >(0)),
    _m_qpFixedK(FixedCapacityPriorityQueue<boost::shared_ptr<PartialJPDPValuePair> >(0)),
    _m_qInfSize(new priority_queue<boost::shared_ptr<JPPVValuePair> >()),
    _m_qpInfSize(new priority_queue<boost::shared_ptr<PartialJPDPValuePair> >()),
    _m_issuedOverFlowWarning(false)
{
#if DBGIPSOL_LEAK
    std::cout << "Constructing BGIPSolution\n"<<std::endl;
#endif
    SetNrDesiredSolutions(_m_nrDesiredSolutions);
}
BGIPSolution::BGIPSolution(
        const I_PtPDpure_constPtr &pu,
        size_t nrDesiredSolutions
        ) :
    _m_pu(0),
    _m_puShared(pu),
    _m_nrDesiredSolutions(nrDesiredSolutions),
    _m_qFixedK(FixedCapacityPriorityQueue<boost::shared_ptr<JPPVValuePair> >(0)),
    _m_qpFixedK(FixedCapacityPriorityQueue<boost::shared_ptr<PartialJPDPValuePair> >(0)),
    _m_qInfSize(new priority_queue<boost::shared_ptr<JPPVValuePair> >()),
    _m_qpInfSize(new priority_queue<boost::shared_ptr<PartialJPDPValuePair> >()),
    _m_issuedOverFlowWarning(false)

{
#if DBGIPSOL_LEAK
    std::cout << "Constructing BGIPSolution\n"<<std::endl;
#endif
    SetNrDesiredSolutions(_m_nrDesiredSolutions);
}


void BGIPSolution::SetNrDesiredSolutions(size_t n) 
{
    _m_nrDesiredSolutions =  n;
    if(_m_nrDesiredSolutions==INT_MAX)
        _m_useFixedCapacityPriorityQueue=false;
    else
        _m_useFixedCapacityPriorityQueue=true;

    if(_m_useFixedCapacityPriorityQueue)
    {
        _m_qFixedK=FixedCapacityPriorityQueue<boost::shared_ptr<JPPVValuePair> >(_m_nrDesiredSolutions);
        _m_qpFixedK=FixedCapacityPriorityQueue<boost::shared_ptr<PartialJPDPValuePair> >(_m_nrDesiredSolutions);
    }
};

//Destructor
BGIPSolution::~BGIPSolution()
{
#if DBGIPSOL_LEAK
    std::cout << "Destructing BGIPSolution: \n---\n" << this->SoftPrint() << "---"<<std::endl;
#endif

    while(!_m_qInfSize->empty())
    {
//        delete _m_qInfSize->top();
        _m_qInfSize->pop();
    }
    while(!_m_qFixedK.empty())
    {
//        delete _m_qFixedK.top();
        _m_qFixedK.pop();
    }
    while(!_m_qpInfSize->empty())
    {
//        delete _m_qpInfSize->top();
        _m_qpInfSize->pop();
    }
    while(!_m_qpFixedK.empty())
    {
//        delete _m_qpFixedK.top();
        _m_qpFixedK.pop();
    }

    delete _m_qInfSize;
    delete _m_qpInfSize;
//     delete _m_policy;
//     delete _m_policyCluster;
}

void BGIPSolution::Save(const string& filename) const
{
    throw(E("BGIPSolution::Save not yet implemented"));
#if 0
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "BGIPSolution::Save: failed to open file " << filename << endl;
        throw E(ss);
    }

    fp.precision(16);
    fp << _m_jpolIndex << " " << _m_payoff << endl;
#endif
}

void BGIPSolution::Load(const string& filename)
{
    throw(E("BGIPSolution::Load not yet implemented"));
#if 0
    ifstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "BGIPSolution::Load: failed to "
           << "open file " << filename << endl;
        throw E(ss);
    }

    LIndex index;
    double payoff;

    string buffer;
    getline(fp,buffer);
    istringstream is(buffer);
    is >> index;
    is >> payoff;

    SetPolicy(index);
    SetPayoff(payoff);
#endif
}

string BGIPSolution::SoftPrint() const
{
    stringstream ss;
    ss << "Payoff " << GetPayoff()
       << endl << "JPPVValuePair queue:";
    if(!_m_useFixedCapacityPriorityQueue)
    {
        ss << " (inf. capacity) ";
        if(_m_qInfSize->empty())
            ss << " empty" << endl;
        else
            ss << " contains " << _m_qInfSize->size() << " elements\n";
    }
    else
    {
        ss << " (fixed capacity) ";
        if(_m_qFixedK.empty())
            ss << " empty" << endl;
        else
            ss << _m_qFixedK.SoftPrint();
    }

    ss << "PartialJPDPValuePair queue:";
    if(!_m_useFixedCapacityPriorityQueue)
    {
        ss << " (inf. capacity) ";
        if(_m_qpInfSize->empty())
            ss << " empty" << endl;
        else
            ss << " contains " << _m_qpInfSize->size() << " elements\n";
    }
    else
    {
        ss << " (fixed capacity) ";
        if(_m_qpFixedK.empty())
            ss << " empty" << endl;
        else
            ss << _m_qpFixedK.SoftPrint();
    }

    return(ss.str());
}

void BGIPSolution::AddSolution(const JointPolicyPureVector &jp,
                               double value)
{
    LIndex pI = 0;
    // first check if this policy is already in the queue
    try{
        pI = jp.GetIndex();
        if(_m_jpolIndices.find(pI) != _m_jpolIndices.end() )
            return; //policy is already in the queue
    }
    catch(EOverflow& e){ 
        GiveOverFlowWarning();
    }

    _m_jpolIndices.insert(pI);

    JPPV_sharedPtr jpPtr=boost::make_shared<JointPolicyPureVector>(jp);

    boost::shared_ptr<JPPVValuePair> p = boost::make_shared<JPPVValuePair>(jpPtr, value);
    boost::shared_ptr<JPPVValuePair> overflownvp_p;

    //FRANS, 20-3-2010: the following construct seems redundant? can't the try by moved inwards?
    //(around the 'GetIndex', which is the function that can throw EOverflow) 
    //or can 'insert' also throw this?
    try { 

    if(_m_useFixedCapacityPriorityQueue)
    {
        if( _m_qFixedK.insert(p, overflownvp_p) )
        {
            Index pI2 =  CastLIndexToIndex(overflownvp_p->GetJPPV()->GetIndex());
            _m_jpolIndices.erase( pI2 );
        }
    }
    else
        _m_qInfSize->push(p);
    }
    catch(EOverflow& e){ 
        GiveOverFlowWarning();
        if(_m_useFixedCapacityPriorityQueue)
            _m_qFixedK.insert(p, overflownvp_p);
        else
            _m_qInfSize->push(p);
    }
}

void BGIPSolution::AddSolution(const JointPolicyPureVectorForClusteredBG &jp,
                               double value)
{
    boost::shared_ptr<JointPolicyPureVectorForClusteredBG> jpPtr=
        boost::shared_ptr<JointPolicyPureVectorForClusteredBG>(new JointPolicyPureVectorForClusteredBG(jp));
    boost::shared_ptr<PartialJPDPValuePair> p=
        boost::shared_ptr<PartialJPDPValuePair>(new PartialJPDPValuePair(jpPtr, value));

/* we don't have an index here, so no check...
   Index pI = p->GetJPol()->GetIndex();
   if(_m_jpolIndices.find(pI) != _m_jpolIndices.end() )
   return; //policy is already in the queue
    _m_jpolIndices.insert(pI);
*/

    if(_m_useFixedCapacityPriorityQueue)
    {
        boost::shared_ptr<PartialJPDPValuePair> overflownvp_p;
        _m_qpFixedK.insert(p, overflownvp_p);
//         if( _m_qpFixedK.insert(p, overflownvp_p) )
//             delete overflownvp_p;
    }
    else
        _m_qpInfSize->push(p);

}

void BGIPSolution::AddSolution(LIndex jpolIndex, double value)
{
    if(_m_pu)
    {
        JointPolicyPureVector jp=JointPolicyPureVector(_m_pu); 
        jp.SetIndex(jpolIndex);
        AddSolution(jp,value);
    }
    else
    {
        JointPolicyPureVector jp=JointPolicyPureVector(_m_puShared);
        jp.SetIndex(jpolIndex);
        AddSolution(jp,value);
    }
}

double BGIPSolution::GetPayoff() const
{ 
    if(_m_useFixedCapacityPriorityQueue)
    {
        if(_m_qFixedK.size() > 0)
            return(_m_qFixedK.top()->GetValue());
        else if(_m_qpFixedK.size() > 0)
            return(_m_qpFixedK.top()->GetValue());
        else
            return(-DBL_MAX);
    }
    else
    {
        if(_m_qInfSize->size() > 0)
            return(_m_qInfSize->top()->GetValue());
        else if(_m_qpInfSize->size() > 0)
            return(_m_qpInfSize->top()->GetValue());
        else
            return(-DBL_MAX);
    }
}

const boost::shared_ptr<JointPolicy> BGIPSolution::GetJointPolicy() const
{
    if(_m_useFixedCapacityPriorityQueue)
        if(_m_qFixedK.size() > 0)
            return(_m_qFixedK.top()->GetJPPV());
        else
            throw(E("BGIPSolution::GetJointPolicy - No solution left in fixed size queue"));
    else
        if(_m_qInfSize->size() > 0)
            return(_m_qInfSize->top()->GetJPPV());
        else
            throw(E("BGIPSolution::GetJointPolicy - No solution left in inf size queue"));
}

const JointPolicyPureVector& BGIPSolution::GetJointPolicyPureVector() const
{
    if(_m_useFixedCapacityPriorityQueue)
        if(_m_qFixedK.size() > 0)
            return(*_m_qFixedK.top()->GetJPPV());
        else
            throw(E("BGIPSolution::GetJointPolicyPureVector - No solution left in fixed size queue"));
    else
        if(_m_qInfSize->size() > 0)
            return(*_m_qInfSize->top()->GetJPPV());
        else
            throw(E("BGIPSolution::GetJointPolicyPureVector - No solution left in inf size queue"));
}

const JointPolicyPureVectorForClusteredBG& 
BGIPSolution::GetJointPolicyPureVectorForClusteredBG() const 
{
    if(_m_useFixedCapacityPriorityQueue)
        return(*boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(
                   _m_qpFixedK.top()->GetJPol()));
    else
        return(*boost::dynamic_pointer_cast<JointPolicyPureVectorForClusteredBG>(
                   _m_qpInfSize->top()->GetJPol()));
}

void BGIPSolution::PopNextSolutionJPPV()
{ 
    if(_m_useFixedCapacityPriorityQueue)
        _m_qFixedK.pop();
    else
        _m_qInfSize->pop();
}

void BGIPSolution::PopNextSolutionPJPDP()
{ 
    if(_m_useFixedCapacityPriorityQueue)
        _m_qpFixedK.pop();
    else
        _m_qpInfSize->pop();
}

size_t BGIPSolution::GetNrFoundSolutions() const
{
    if(_m_useFixedCapacityPriorityQueue)
        return std::max(_m_qFixedK.size(),_m_qpFixedK.size());
    else
        return std::max(_m_qInfSize->size(),_m_qpInfSize->size());
}

boost::shared_ptr<JPPVValuePair> BGIPSolution::GetNextSolutionJPPV() const
{
    if(!_m_useFixedCapacityPriorityQueue)
        return _m_qInfSize->top();
    else
        return _m_qFixedK.top();
}

bool BGIPSolution::IsEmptyJPPV() const
{
    if(!_m_useFixedCapacityPriorityQueue)
        return _m_qInfSize->empty();
    else
        return _m_qFixedK.empty();
}

boost::shared_ptr<PartialJPDPValuePair> BGIPSolution::GetNextSolutionPJPDP() const
{
    if(!_m_useFixedCapacityPriorityQueue)
        return _m_qpInfSize->top();
    else
        return _m_qpFixedK.top();
}

bool BGIPSolution::IsEmptyPJPDP() const
{
    if(!_m_useFixedCapacityPriorityQueue)
        return _m_qpInfSize->empty();
    else
        return _m_qpFixedK.empty();
}
