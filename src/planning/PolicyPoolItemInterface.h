/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _POLICYPOOLITEMINTERFACE_H_
#define _POLICYPOOLITEMINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "boost/shared_ptr.hpp"

class PolicyPoolItemInterface;
typedef boost::shared_ptr<PolicyPoolItemInterface> PolicyPoolItemInterface_sharedPtr;

class JointPolicyDiscretePure;

/**\brief PolicyPoolItemInterface is a class that gives the interface for a 
 * PolicyPoolItem. A PolicyPoolItem is a wrapper for a partial
 * joint policy (together with some properties) when placed in a PolicyPool.
 *
 **/
class PolicyPoolItemInterface 
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        //PolicyPoolItemInterface();
        /// Copy constructor.
        //PolicyPoolItemInterface(const PolicyPoolItemInterface& a);
        /// Destructor.
        virtual ~PolicyPoolItemInterface()
        {};
        /// Copy assignment operator
        //PolicyPoolItemInterface& operator= (const PolicyPoolItemInterface& o);

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        //
        /**\brief Returns a pointer to the wrapped (partial) joint policy*/
        virtual boost::shared_ptr<JointPolicyDiscretePure> GetJPol() const = 0;
        /**\brief Returns the heuristic value.*/
        virtual double GetValue() const=0;
        /**Softprint the PolicyPoolItemInterface*/
        virtual std::string SoftPrint() const = 0;
        /**Softprint the PolicyPoolItemInterface in brief*/
        virtual std::string SoftPrintBrief() const = 0;
        /**Prints a description to stdout.*/
        void Print() const 
        {std::cout << SoftPrint();}
        /**Prints a brief description to stdout.*/
        void PrintBrief() const 
        {std::cout << SoftPrint();}
};


#endif /* !_POLICYPOOLITEMINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
