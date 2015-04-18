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
#ifndef _LOCALBGVALUEFUNCTIONINTERFACE_H_
#define _LOCALBGVALUEFUNCTIONINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Scope.h"



/**LocalBGValueFunctionInterface is a class that represents a local CGBG payoff function.
 * I.e., it represents an \f$  u^e( \beta_e ) \f$.
 */
class LocalBGValueFunctionInterface 
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        //LocalBGValueFunctionInterface();
        /// Copy constructor.
        //LocalBGValueFunctionInterface(const LocalBGValueFunctionInterface& a);
        /// Destructor.
        virtual ~LocalBGValueFunctionInterface(){};
        /// Copy assignment operator
        //LocalBGValueFunctionInterface& operator= (const LocalBGValueFunctionInterface& o);


        virtual void SetValue(Index jpolI, double best_response_value) = 0;
        virtual void SetValue(std::vector<Index> indPol, 
                    double best_response_value) = 0;
        virtual double GetValue(Index jpolI) const = 0;
        virtual double GetValue(std::vector<Index> indPols) const = 0;
        virtual Scope GetAgentScope() const = 0;

        virtual std::string SoftPrint() const = 0;
        virtual void Print() const = 0;
};


#endif /* !_LOCALBGVALUEFUNCTIONINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
