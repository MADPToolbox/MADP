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

#include "JPPVValuePair.h"
#include "JointPolicyPureVector.h"

using namespace std;

#if DBG_COUNT
size_t JPPVValuePair::count_construct = 0;
size_t JPPVValuePair::count_destruct = 0;
#endif

JPPVValuePair::JPPVValuePair(const JPPV_sharedPtr &jp, double value) :
    JointPolicyValuePair(value)
{
#if DBG_COUNT
    count_construct++;
    std::cout << "Contructed JPPVValuePair #"<<  count_construct <<std::endl;
#endif
    _m_jpol = jp;
}                

JPPVValuePair::~JPPVValuePair()
{
#if DBG_COUNT
    count_destruct++;
    std::cout << "Destructed JPPVValuePair #"<<  count_destruct <<std::endl;
#endif
//    delete _m_jpol;
}

string JPPVValuePair::SoftPrint() const
{
    stringstream ss;
    ss << "JPPVValuePair: value="<< GetValue() <<", pol:"; 
    ss << _m_jpol->SoftPrint();
    return(ss.str());
}

string JPPVValuePair::SoftPrintBrief() const
{ 
    stringstream ss;
    ss << "JPPVValuePair(" << GetValue() << ","
       << _m_jpol->GetIndex() << "[d=" << _m_jpol->GetDepth()
       << "])";
    return(ss.str());
}
