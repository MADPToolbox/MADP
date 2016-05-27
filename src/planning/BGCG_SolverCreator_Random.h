/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGCG_SOLVERCREATOR_RANDOM_H_
#define _BGCG_SOLVERCREATOR_RANDOM_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGCG_SolverCreator.h"

//We have to include this (otherwise compiler doesn't know that 
//BGCG_SolverRandom is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGCG_SolverRandom.h"

/** \brief BGCG_SolverCreator_Random creates a BGCG Solver that gives
 * random solutions.  */
class BGCG_SolverCreator_Random : public BGCG_SolverCreator
{
private:

    size_t _m_verbose;
    size_t _m_nrSolutions;

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGCG_SolverCreator_Random(size_t verbose=1,
                              size_t nrSolutions=1) :
                _m_verbose(verbose),
                _m_nrSolutions(nrSolutions)
    {}

    //operators:
    //BGCG_SolverRandom* operator()
    BGCG_SolverRandom* Create_BGCG_Solver(
        const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bg) const
    {
        return( 
            new BGCG_SolverRandom(
                bg,
                _m_nrSolutions));
    }

    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGCG_SolverCreator_Random object with _m_verbose="<<_m_verbose
               << ", _m_nrSolutions="<<_m_nrSolutions<< endl;
            return (ss.str());
        }


    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGCGSC_Random_nrSol" << _m_nrSolutions;
            return(ss.str());
        }

    bool IsExactSolver() const { return(false); }

};


#endif /* !_BGCG_SOLVERCREATOR_RANDOM_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
