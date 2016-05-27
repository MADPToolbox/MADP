/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
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
#ifndef _FG_SOLVERNDP_H_
#define _FG_SOLVERNDP_H_ 1

/* the include directives */
#include "Globals.h"
#include "FG_Solver.h"

/** \brief FG_SolverNDP optimizes (maximizes) a factor graph
 *  using non-serial dynamic programming. 
 *
 *  I.e., the factor graph represents the sum of the factors and 
 *  we look for the maximizing configuration.
 *
 *
 *  NOTE: NDP is an exact method, but FG_SolverNDP may return 
 *  an incorrect value. This happens when there are factors
 *  to which no variable is connected (in this case, NDP
 *  will return the sum of the values for the connected factors)
 *
 **/


class FG_SolverNDP :   
    public FG_Solver
//    , public NDPSolver
{
    private:    
        int _m_verbosity;
        size_t _m_nrSolutions;
        double _m_deadlineInSeconds;

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        FG_SolverNDP(const libDAI::FactorGraph& f, 
                     //ndp parameters: 
                     int verbosity = 2,
                     size_t nrSolutions = 1,
                     double deadlineInSeconds=0);
        /// Copy constructor.
        //FG_SolverNDP(const FG_SolverNDP& a);
        /// Destructor.
        //~FG_SolverNDP();
        /// Copy assignment operator
        //FG_SolverNDP& operator= (const FG_SolverNDP& o);

        //operators:

        //data manipulation (set) functions:
        double Solve();
        
        //get (data) functions:

};


#endif /* !_FG_SOLVERNDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
