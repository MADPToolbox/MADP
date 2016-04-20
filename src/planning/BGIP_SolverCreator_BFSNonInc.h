/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGIP_SOLVERCREATOR_BFSNONINC_H_
#define _BGIP_SOLVERCREATOR_BFSNONINC_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_SolverCreatorInterface_T.h"

//We have to include this (otherwise compiler doesn't know that 
//BGIP_SolverBruteForceSearch is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGIP_SolverBFSNonIncremental.h"
//class BGIP_SolverBruteForceSearch;

/** \brief BGIP_SolverCreator_BFSNonInc creates BGIP Solvers with Brute
 * Force Search.  */
template<class JP>
class BGIP_SolverCreator_BFSNonInc : public BGIP_SolverCreatorInterface_T<JP>
{
private:    
    size_t _m_verbose;
    size_t _m_nrSolutions;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverCreator_BFSNonInc(size_t verbose=0,
                           size_t nrSolutions=INT_MAX)
        :
        _m_verbose(verbose),
        _m_nrSolutions(nrSolutions)
        {}
/*        /// Copy constructor.
        BGIP_SolverCreator_BFSNonInc(const BGIP_SolverCreator_BFSNonInc& a);
        /// Destructor.
        ~BGIP_SolverCreator_BFSNonInc();
        /// Copy assignment operator
        BGIP_SolverCreator_BFSNonInc& operator= (const BGIP_SolverCreator_BFSNonInc& o);
*/
    //operators:
    virtual BGIP_SolverBFSNonIncremental<JP>* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            return( 
                new BGIP_SolverBFSNonIncremental<JP>(
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
            ss << "BGIP_SolverCreator_BFSNonInc object with "<<
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


#endif /* !_BGIP_SOLVERCREATOR_BFSNONINC_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
