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
#ifndef _LOCALBGVALUEFUNCTIONVECTOR_H_
#define _LOCALBGVALUEFUNCTIONVECTOR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "IndexTools.h"
#include "LocalBGValueFunctionInterface.h"
#include <boost/shared_ptr.hpp>

class BayesianGameCollaborativeGraphical;
//class Scope;
#include "Scope.h"



/**LocalBGValueFunctionVector is a vector implementation of
 * LocalBGValueFunctionInterface to represent  an \f$  u^e( \beta_e ) \f$.
 **/
class LocalBGValueFunctionVector : public LocalBGValueFunctionInterface
{
    private:    
        Scope _m_agentScope;
        boost::shared_ptr<const BayesianGameCollaborativeGraphical> _m_bgcg;
        std::vector<double> _m_values;
        /**a vector with the number of joint policies of each agent that
         * participates in this LocalBGValueFunction */
        std::vector<size_t> _m_nrIndivPols;
        /**the number of joint *local* policies*/
        size_t _m_nrJointPols;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        LocalBGValueFunctionVector(
                 const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &cgbg, 
                 Scope agentScope);
//        LocalBGValueFunctionVector(BayesianGameCollaborativeGraphical* cgbg, 
//                std::vector<Index> thisAgNeighbors);
        /// Copy constructor.
//        LocalBGValueFunctionVector(const LocalBGValueFunctionVector& a);
        /// Destructor.
        ~LocalBGValueFunctionVector();
        /// Copy assignment operator
//        LocalBGValueFunctionVector& operator= 
//              (const LocalBGValueFunctionVector& o);

        //operators:

        void SetValue(Index jpolI, double value)
        { _m_values.at(jpolI) = value; }
        void SetValue(std::vector<Index> indPols, double value)
        { _m_values.at(
            IndexTools::IndividualToJointIndices(indPols, _m_nrIndivPols)
                ) = value;}
        double GetValue(Index jpolI) const
        { return _m_values.at(jpolI); }
        double GetValue(std::vector<Index> indPols) const
        { return _m_values.at(
            IndexTools::IndividualToJointIndices(indPols, _m_nrIndivPols)
                ) ;}
        Scope GetAgentScope() const
        { return _m_agentScope; }

        std::string SoftPrint() const;
        void Print() const { std::cout << SoftPrint() << std::endl; };

};


#endif /* !_LOCALBGVALUEFUNCTIONVECTOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
