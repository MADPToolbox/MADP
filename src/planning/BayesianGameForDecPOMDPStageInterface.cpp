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

#include "BayesianGameForDecPOMDPStageInterface.h"
#include "PartialJointPolicyDiscretePure.h"
using namespace std;

BayesianGameForDecPOMDPStageInterface::BayesianGameForDecPOMDPStageInterface() :
    _m_t(0),
    _m_pJPol()
{
}

BayesianGameForDecPOMDPStageInterface::BayesianGameForDecPOMDPStageInterface(
        const boost::shared_ptr<const PartialJointPolicyDiscretePure> &pastJPol)
    :
        _m_t( pastJPol->GetDepth() )
        ,_m_pJPol(pastJPol)
{
}
BayesianGameForDecPOMDPStageInterface::BayesianGameForDecPOMDPStageInterface(
        Index t)
    :
        _m_t( t )
        ,_m_pJPol()
{
}
