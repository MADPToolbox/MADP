/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGCG_SOLVERCREATOR_NDP_H_
#define _BGCG_SOLVERCREATOR_NDP_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_SolverCreator.h"

//We have to include this (otherwise compiler doesn't know that 
//BGCG_SolverMaxPlus is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGCG_SolverNonserialDynamicProgramming.h"

/** \brief BGCG_SolverCreator_NDP creates BGCG Solvers with
 * Non-serial Dynamic Programming. */
class BGCG_SolverCreator_NDP : public BGCG_SolverCreator
{
private:    

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGCG_SolverCreator_NDP()
        {}
/*        /// Copy constructor.
          BGCG_SolverCreator_NDP(const BGCG_SolverCreator_NDP& a);
        /// Destructor.
        ~BGCG_SolverCreator_NDP();
        /// Copy assignment operator
        BGCG_SolverCreator_NDP& operator= (const BGCG_SolverCreator_NDP& o);
*/
    //operators:
    //BGCG_SolverNonserialDynamicProgramming* operator()
    BGCG_SolverNonserialDynamicProgramming* Create_BGCG_Solver(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bg) const
        {
            return( 
                new BGCG_SolverNonserialDynamicProgramming(
                    bg)
                );
        };

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGCG_SolverCreator_NDP object";
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGCGSC_NDP";
            return (ss.str());
        }

    bool IsExactSolver() const { return(true); }

};


#endif /* !_BGCG_SOLVERCREATOR_NDP_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
