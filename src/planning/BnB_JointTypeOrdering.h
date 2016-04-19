/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
