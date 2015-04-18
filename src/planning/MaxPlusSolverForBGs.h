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
#ifndef _MAXPLUSSOLVERFORBGS_H_
#define _MAXPLUSSOLVERFORBGS_H_ 1

/* the include directives */
#include "Globals.h"
#include "MaxPlusSolver.h"
#include "factor.h"


class BayesianGameIdenticalPayoffInterface;
using namespace std;

/** \brief MaxPlusSolverForBGs solves BG via Max Plus
 * \deprecated THIS CLASS IS OBSOLETE - 
 * -> will be replaced or changed to make use of BG_FactorGraphCreator.
 *
 **/
class MaxPlusSolverForBGs :
    public MaxPlusSolver
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        MaxPlusSolverForBGs(
            size_t maxiter = 1000,
            std::string updateType=std::string("PARALL"),
            int verbosity = 2,
            double damping = 0.0,
            size_t nrSolutions = 1,
            size_t nrRestarts = 1);
/*        /// Copy constructor.
        MaxPlusSolverForBGs(const MaxPlusSolverForBGs& a);
        /// Destructor.
        ~MaxPlusSolverForBGs();
        /// Copy assignment operator
        MaxPlusSolverForBGs& operator= (const MaxPlusSolverForBGs& o);
*/
        static void Construct_AgentTypePair_Variables(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bgip, 
            vector< vector<Index> >& var_indices, vector< libDAI::Var >& vars,
            int verbosity );

        static void Construct_JointType_Factors(
            const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bgip,
            const vector< vector<Index> >& var_indices,
            const vector< libDAI::Var >& vars,
            vector<libDAI::Factor>& facs,
            int verbosity);

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
};


#endif /* !_MAXPLUSSOLVERFORBGS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
