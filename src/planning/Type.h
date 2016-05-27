/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TYPE_H_
#define _TYPE_H_ 1

/* the include directives */
#include "Globals.h"

/** \brief Type is an abstract class that represents a Type (e.g. for a 
 * Bayesian game). 
 *
 * So far Type_PointerTuple and Type_AOHIndex extend this class.
 **/
class Type 
{
    public:
        enum SubClass {UNDEFINED, AOHINDEX, POINTERTUPLE};
    private:    

    protected:
        SubClass _m_sc;
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor

        Type(SubClass s=AOHINDEX)
            : _m_sc(s) {};
        /// Destructor.
        virtual ~Type() {};
        /// Copy constructor.        
        Type(const Type& a)
            : _m_sc(a._m_sc) {};

        /// Copy assignment operator
        Type& operator= (const Type& o)
        {
            if (this == &o) return *this;   // Gracefully handle self assignment
            // Put the normal assignment duties here...
            _m_sc = o._m_sc;
            return *this;
        }



        virtual Type* Clone() const=0;
        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        SubClass GetSubClass() const
        {return _m_sc;};
        virtual std::string SoftPrint () const = 0;
};


#endif /* !_TYPE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
