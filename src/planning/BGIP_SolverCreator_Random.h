/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _BGIP_SOLVERCREATOR_Random_H_
#define _BGIP_SOLVERCREATOR_Random_H_ 1

/* the include directives */
#include "Globals.h"
#include "BGIP_SolverCreatorInterface_T.h"

//We have to include this (otherwise compiler doesn't know that 
//BGIP_SolverRandom is-a BayesianGameIdenticalPayoffSolver_T
//and thus that the virtual function "operator()" is implemented...
#include "BGIP_SolverRandom.h"
//class BGIP_SolverRandom;

/** \brief BGIP_SolverCreator_Random creates a BGIP Solver that gives
 * random solutions.  */
class BGIP_SolverCreator_Random : public BGIP_SolverCreatorInterface_T<JointPolicyPureVector>
{
private:    
    
    int _m_verbose;
    size_t _m_nrSolutions;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    BGIP_SolverCreator_Random(int verbose=0,
                              size_t nrSolutions=1)
        :
        _m_verbose(verbose),
        _m_nrSolutions(nrSolutions)
        {}
    virtual ~BGIP_SolverCreator_Random(){};
    
    //operators:
    BGIP_SolverRandom* operator()
        (const boost::shared_ptr<const BayesianGameIdenticalPayoffInterface> &bg) const
        {
            if(_m_verbose >= 2)
            {
                std::cout << "BGIP_SolverCreator_Random:: creating a new BGIP_SolverRandom verbose=" << _m_verbose << ", nrSols="<<_m_nrSolutions << std::endl;
            }
            return( 
                new BGIP_SolverRandom(
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
            ss << "BGIP_SolverCreator_Random object with "<<
                ", _m_verbose="<<_m_verbose <<
                ", _m_nrSolutions="<<_m_nrSolutions<<
                std::endl;
            return (ss.str());
        }

    std::string SoftPrintBrief() const
        {
            std::stringstream ss;
            ss << "BGIPSC_Random_nrSol" << _m_nrSolutions;
            return(ss.str());
        }


    bool IsExactSolver() const { return(false); }

};


#endif /* !_BGIP_SOLVERCREATOR_Random_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
