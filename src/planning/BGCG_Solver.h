/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGCG_SOLVER_H_
#define _BGCG_SOLVER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "BayesianGameIdenticalPayoffSolver.h"
#include "JointPolicyPureVector.h"

class BayesianGameCollaborativeGraphical;

/**BGCG_Solver is a base class for collaborative graphical bayesian games.
 **/
class BGCG_Solver 
    :
        public BayesianGameIdenticalPayoffSolver
{
    private:    
    
    protected:
        boost::shared_ptr<const BayesianGameCollaborativeGraphical> _m_bgcg;

    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        BGCG_Solver(const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg, size_t nrSolutions=1);
        /// Copy constructor.
        //BGCG_Solver(const BGCG_Solver& a);
        /// Destructor.
        virtual ~BGCG_Solver();
        /// Copy assignment operator
        BGCG_Solver& operator= (const BGCG_Solver& o);

        //operators:

        
        // perhaps derive from BayesianGameIdenticalPayoffSolver, (then 
        // we do not need the following functions)
        /**The methods that performs the planning. 
         * Returns the expected reward.*/
        virtual double Solve() = 0;
        
        //get (data) functions:         

        //this we simply inherit wrom BayesianGameIdenticalPayoffSolver: 
        //virtual boost::shared_ptr<JointPolicyPureVector> GetNewJpol() const;

        boost::shared_ptr<const BayesianGameCollaborativeGraphical> GetBGCG() const
            { return _m_bgcg; }
};


#endif /* !_BGCG_SOLVER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
