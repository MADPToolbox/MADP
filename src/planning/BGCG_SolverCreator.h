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
#ifndef _BGCG_SOLVERCREATOR_H_
#define _BGCG_SOLVERCREATOR_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_SolverCreatorInterface.h"

#include "BGCG_Solver.h"
//class BGCG_Solver;
//compiler needs to know that BGCG_Solver isa BayesianGameIdenticalPayoffSolver_T

#include "BayesianGameCollaborativeGraphical.h"


/** \brief BGCG_SolverCreator is a class that represents an object that can
 * create a BGCG_Solver. 
 *
 * It is used to spawn new solvers inside GMAA*
 * */
class BGCG_SolverCreator 
    : public BGIP_SolverCreatorInterface
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
/*        BGCG_SolverCreator();
        /// Copy constructor.
        BGCG_SolverCreator(const BGCG_SolverCreator& a);
        /// Destructor.
        ~BGCG_SolverCreator();
        /// Copy assignment operator
        BGCG_SolverCreator& operator= (const BGCG_SolverCreator& o);
*/
        //operators:

        //virtual BGCG_Solver* operator() // <-leads to "call of overloaded ‘operator()(boost::shared_ptr<BGCGforStage>&)’ is ambiguous"
        virtual BGCG_Solver* Create_BGCG_Solver
            (const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg) const = 0;

        //implement BGIP_SolverCreator interface:
        virtual BayesianGameIdenticalPayoffSolver* operator()
        //virtual BGCG_Solver* operator()
            (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            boost::shared_ptr<const BayesianGameCollaborativeGraphical> bgcg =
                boost::dynamic_pointer_cast<const BayesianGameCollaborativeGraphical> (bg);
            //return this->operator()(bgcg);
            return this->Create_BGCG_Solver(bgcg);
        }

        //data manipulation (set) functions:
        
        //get (data) functions:
        virtual std::string SoftPrint() const = 0;
};


#endif /* !_BGCG_SOLVERCREATOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
