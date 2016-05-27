/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _FG_SOLVERMAXPLUS_H_
#define _FG_SOLVERMAXPLUS_H_ 1

/* the include directives */
#include "Globals.h"
#include "FG_Solver.h"
#include "MaxPlusSolver.h"
#include "maxplus.h"

/** \brief FG_SolverMaxPlus optimizes (maximizes) a factor graph
 *  using max plus. 
 *
 *  I.e., the factor graph represents the sum of the factors and 
 *  we look for the maximizing configuration.
 **/


class FG_SolverMaxPlus :   
    public FG_Solver,
    public MaxPlusSolver
{
    private:    
    
    double _m_deadlineInSeconds;

    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        FG_SolverMaxPlus(const libDAI::FactorGraph& f, 
                         //maxplus parameters: 
                         size_t maxiter = 1000,
                         std::string updateType=std::string("PARALL"),
                         int verbosity = 2,
                         double damping = 0.0,
                         size_t nrSolutions = 1,
                         size_t nrRestarts = 1,
                         double deadlineInSeconds=0);
        /// Copy constructor.
        //FG_SolverMaxPlus(const FG_SolverMaxPlus& a);
        /// Destructor.
        //~FG_SolverMaxPlus();
        /// Copy assignment operator
        //FG_SolverMaxPlus& operator= (const FG_SolverMaxPlus& o);

        //operators:

        //data manipulation (set) functions:
        double Solve();
        
        //get (data) functions:

};


#endif /* !_FG_SOLVERMAXPLUS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
