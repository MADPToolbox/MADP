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
#ifndef _MAXPLUSSOLVER_H_
#define _MAXPLUSSOLVER_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief MaxPlusSolver is the base class for Max Plus methods, it stores
 * the parameters.
 **/
class MaxPlusSolver 
{
    private:      
    protected:
    ///stores the MaxPlus parameter for the max. number of iterations
    size_t _m_maxiter;
    ///stores the MaxPlus parameter for the update type
    std::string _m_updateType;
    ///stores the MaxPlus parameter for the verbosity level
    int _m_verbosity;
    ///stores the damping factor
    double _m_damping;
    ///stores the (desired) number of solutions
    size_t _m_nrSolutions;
    ///stores the number of restarts (for non-deterministic Max-Plus variants)
    size_t _m_nrRestarts;  
    
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        MaxPlusSolver(
            size_t maxiter = 1000,
            std::string updateType=std::string("PARALL"),
            int verbosity = 2,
            double damping = 0.0,
            size_t nrSolutions = 1,
            size_t nrRestarts = 1);

/*
        /// Copy constructor.
        MaxPlusSolver(const MaxPlusSolver& a);
        /// Destructor.
        ~MaxPlusSolver();
        /// Copy assignment operator
        MaxPlusSolver& operator= (const MaxPlusSolver& o);
*/
        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
};


#endif /* !_MAXPLUSSOLVER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
