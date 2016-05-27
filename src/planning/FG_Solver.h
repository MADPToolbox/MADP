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
#ifndef _FG_SOLVER_H_
#define _FG_SOLVER_H_ 1

/* the include directives */
#include "Globals.h"
#include "maxplus.h"

/** \brief FG_Solver is a base class for methods that 'solve' a factor graph
 *
 * Here we mean maximize a factor graph that represents the sum of its factors.
 * 
 *
 **/

namespace libDAI{
    class FactorGraph;
}

class FG_Solver 
{
    public:
        enum FG_Solver_t {FGSt_MaxPlus, FGSt_NDP};
    private:    
    protected:
        const libDAI::FactorGraph* _m_fg;
        
        //a list to store the best configurations
        std::list< libDAI::MADP_util::valConf > _m_bestConfs;
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        FG_Solver(const libDAI::FactorGraph& f) :
            _m_fg(&f)
        {}
/*        
        /// Copy constructor.
        FG_Solver(const FG_Solver& a);
*/
        /// Destructor.
        virtual ~FG_Solver(){}
/*
        /// Copy assignment operator
        FG_Solver& operator= (const FG_Solver& o);
*/
        //operators:

        //data manipulation (set) functions:
        virtual double Solve() = 0;
        
        //get (data) functions:
        std::list< libDAI::MADP_util::valConf >& GetBestConfigurations()
        { return _m_bestConfs;}

        static std::string SoftPrint(FG_Solver_t fgs)
        {
            switch(fgs)
            {
            case FGSt_MaxPlus:
                return("FGMaxPlus");
            case FGSt_NDP:
                return("FGNonSerDP");
            }
            throw(E("FG_Solver::SoftPrint() FG_Solver_t not handled"));
            return("");
        }
};


#endif /* !_FG_SOLVER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
