/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
