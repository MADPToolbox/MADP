/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGIP_SOLVERCREATOR_BFS_H_
#define _BGIP_SOLVERCREATOR_BFS_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_IncrementalSolverCreatorInterface_T.h"

//We have to include this (otherwise compiler doesn't know that 
//BGIP_SolverBruteForceSearch is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGIP_SolverBruteForceSearch.h"
//class BGIP_SolverBruteForceSearch;

/** \brief BGIP_SolverCreator_BFS creates BGIP Solvers with Brute
 * Force Search.  */
template<class JP>
class BGIP_SolverCreator_BFS : public BGIP_IncrementalSolverCreatorInterface_T<JP>
{
private:    
    size_t _m_verbose;
    size_t _m_nrSolutions;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverCreator_BFS(size_t verbose=0,
                           size_t nrSolutions=INT_MAX)
        :
        _m_verbose(verbose),
        _m_nrSolutions(nrSolutions)
        {}
/*        /// Copy constructor.
        BGIP_SolverCreator_BFS(const BGIP_SolverCreator_BFS& a);
        /// Destructor.
        ~BGIP_SolverCreator_BFS();
        /// Copy assignment operator
        BGIP_SolverCreator_BFS& operator= (const BGIP_SolverCreator_BFS& o);
*/
    //operators:
    virtual BGIP_SolverBruteForceSearch<JP>* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            return( 
                new BGIP_SolverBruteForceSearch<JP>(
                    bg,
                    _m_verbose,
                    _m_nrSolutions
                    )
                );
        }
    
    //data manipulation (set) functions:
    
    //get (data) functions:
    std::string SoftPrint() const
        {
            std::stringstream ss;
            ss << "BGIP_SolverCreator_BFS object with "<<
                ", _m_verbose="<<_m_verbose <<
                ", _m_nrSolutions="<<_m_nrSolutions<<
                std::endl;
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGIPSC_BruteForce_nrSol" << _m_nrSolutions;
            return(ss.str());
        }


    bool IsExactSolver() const { return(true); }

};


#endif /* !_BGIP_SOLVERCREATOR_BFS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
