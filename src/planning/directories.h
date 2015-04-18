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

#ifndef _DIRECTORIES_H_
#define _DIRECTORIES_H_ 1

#include <string>

class PlanningUnit;
class MultiAgentDecisionProcessInterface;
namespace ArgumentHandlers { class Arguments; }

/* This file provides functions to get directory and file names.
 * The code is structered around the following principles.
 *
 * (Input) problem files
 *
 * (Output) result files
 * --------------------
 *  are stored in
 *  MADPGetResultsDir (= ~/.madp/results/METHOD/PROBLEM )
 *  which can be retrieved using MADPGetResultsDir(...)
 *
 *  and have a basename
 *  METHOD_DESCR_PREFIX_ (where DESCR defaults to PROBLEM)
 *  which can be retrieved with MADPGetResultsBaseFilename(...).
 *
 *  The result is that result files are written to
 *  ~/.madp/results/METHOD/PROBLEM/METHOD_DESCR_PREFIX_....
 *
 *  MADPGetResultsFilename(...) gives you the concatenated version directly.
 *
 */

namespace directories {

    std::string MADPGetResultsDir();
    std::string MADPGetResultsDir(const std::string &method,
                                  const std::string &problem);
    std::string MADPGetResultsDir(const std::string &method,
                                  const PlanningUnit &pu);
    std::string MADPGetResultsDir(const std::string &method,
                                  const 
                                  MultiAgentDecisionProcessInterface &problem);
    std::string MADPGetResultsDir(const std::string &method,
                                  const PlanningUnit *pu);
    std::string MADPGetResultsDir(const std::string &method,
                                  const 
                                  MultiAgentDecisionProcessInterface *problem);

    void MADPCreateResultsDir(const std::string &method,
                              const std::string &problem);
    void MADPCreateResultsDir(const std::string &method,
                              const PlanningUnit &pu);
    void MADPCreateResultsDir(const std::string &method,
                              const 
                              MultiAgentDecisionProcessInterface &problem);
    void MADPCreateResultsDir(const std::string &method,
                              const PlanningUnit *pu);
    void MADPCreateResultsDir(const std::string &method,
                              const 
                              MultiAgentDecisionProcessInterface *problem);

    std::string MADPGetResultsBaseFilename(const std::string &method,
                                           const std::string &problem,
                                           const
                                           ArgumentHandlers::Arguments &args);

    std::string MADPGetResultsFilename(const std::string &method,
                                       const std::string &problem,
                                       const ArgumentHandlers::Arguments &args);
    std::string MADPGetResultsFilename(const std::string &method,
                                       const PlanningUnit &pu,
                                       const ArgumentHandlers::Arguments &args);
    std::string MADPGetResultsFilename(const std::string &method,
                                       const
                                       MultiAgentDecisionProcessInterface 
                                       &problem,
                                       const ArgumentHandlers::Arguments &args);

    std::string MADPGetProblemsDir();
    std::string MADPGetProblemFilename(const std::string &problem,
                                       const std::string &extension);
    std::string MADPGetProblemFilename(const std::string &problem);
    std::string MADPGetProblemFilename(const ArgumentHandlers::Arguments &args);

} // namespace directories

#endif /* !_DIRECTORIES_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
