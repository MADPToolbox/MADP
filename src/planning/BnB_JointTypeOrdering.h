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

/* Only include this header file once. */
#ifndef _BNB_JOINTTYPEORDERING_H_
#define _BNB_JOINTTYPEORDERING_H_ 1

#include "Globals.h"
#include <string>

namespace BGIP_BnB {
    enum BnB_JointTypeOrdering { IdentityMapping,
                                 MaxContribution,
                                 MinContribution,
                                 MaxContributionDifference,
                                 DescendingProbability,
                                 BasisTypes,
                                 ConsistentMaxContribution,
                                 ConsistentMinContribution,
                                 ConsistentMaxContributionDifference };

    const std::string BnB_JointTypeOrderingNames[] = {
        "IdentityMapping",
        "MaxContribution",
        "MinContribution",
        "MaxContributionDifference",
        "DescendingProbability",
        "BasisTypes",
        "ConsistentMaxContribution",
        "ConsistentMinContribution",
        "ConsistentMaxContributionDifference"
    };
    std::string SoftPrint(BnB_JointTypeOrdering type);
    const size_t NUMBER_OF_BNB_JOINT_TYPE_ORDERINGS = 9;

    static const Index UNSPECIFIED_ACTION=INDEX_MAX;
}


#endif /* !_BNB_JOINTTYPEORDERING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
