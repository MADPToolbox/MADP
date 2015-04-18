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
