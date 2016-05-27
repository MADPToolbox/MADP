/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _LOCALBGVALUEFUNCTIONBGCGWRAPPER_H_
#define _LOCALBGVALUEFUNCTIONBGCGWRAPPER_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "LocalBGValueFunctionInterface.h"
#include <boost/shared_ptr.hpp>

class BayesianGameCollaborativeGraphical;


/**LocalBGValueFunctionBGCGWrapper is a class that represents a wrapper
 * for a BayesianGameCollaborativeGraphical such that it implements the
 * LocalBGValueFunctionInterface.
 *
 * When solving a BayesianGameCollaborativeGraphical using nonserial dynamic
 * programming (NDP), each LRF (i.e., each local payoff function of the BGCG)
 * is treated as a local value function (i.e. a LocalBGValueFunctionInterface).
 *
 * Note that not every local value function (LVF) is necessaririly a LRF: NDP
 * creates new LVFs (that do not correspond to any LRF) when eliminating agents.
 *
 *
 * This class creates a wrapper object, that can be queried
 * like an LocalBGValueFunctionInterface, but uses 
 * BayesianGameCollaborativeGraphical as its back end.
 *
 * */
class LocalBGValueFunctionBGCGWrapper 
    : public LocalBGValueFunctionInterface
{
    private:
        boost::shared_ptr<const BayesianGameCollaborativeGraphical> _m_bgcg;
        Index _m_LRF;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        LocalBGValueFunctionBGCGWrapper(
                const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg, Index LRF);
        /// Copy constructor.
/*        LocalBGValueFunctionBGCGWrapper(const 
                   LocalBGValueFunctionBGCGWrapper& a);
        /// Destructor.
        ~LocalBGValueFunctionBGCGWrapper();
        /// Copy assignment operator
        LocalBGValueFunctionBGCGWrapper& operator= 
                (const LocalBGValueFunctionBGCGWrapper& o);
*/                

        //this LocalBGValueFunction represents an LRF of the BGCG, and that
        //is fixed (or should be set using the normal BGCG interface).
        //SetValue should not be called here.
        void SetValue(Index jpolI, double value)
        {throw E("trying SetValue for LocalBGValueFunctionBGCGWrapper!");}
        void SetValue(std::vector<Index> indPol, double value)
        {throw E("trying SetValue for LocalBGValueFunctionBGCGWrapper!");}            
        double GetValue(Index jpolI) const;
        double GetValue(std::vector<Index> indPols) const;
        
        Scope GetAgentScope() const;
// in .cpp        {return _m_bgcg->GetScope(_m_LRF);}

        std::string SoftPrint() const;
        void Print() const { std::cout << SoftPrint() << std::endl; };

};


#endif /* !_LOCALBGVALUEFUNCTIONBGCGWRAPPER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
