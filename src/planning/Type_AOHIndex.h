/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TYPE_AOHINDEX_H_
#define _TYPE_AOHINDEX_H_ 1

/* the include directives */
#include "Globals.h"
#include "Type.h"

/** \brief Type_AOHIndex  is a implementation (extenstion) of Type and 
 * represents a type in e.g. a BG.
 * 
 * In particular this implementation of Type is a wrapper for an individual
 * action-observation history.
 **/
class Type_AOHIndex : public Type
{
    private:    
        Index _m_aohI;
        
    protected:
    
    public:
        
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        Type_AOHIndex(Index aohI) :
            Type(AOHINDEX)
            , _m_aohI(aohI) 
        {};
        /// Copy constructor.
        Type_AOHIndex(const Type_AOHIndex& a) : 
            Type(a)
            ,_m_aohI(a._m_aohI) 
        {};
        /// Destructor.
        virtual ~Type_AOHIndex(){};
        /// Copy assignment operator
        Type_AOHIndex& operator= (const Type_AOHIndex& o);
        Type* Clone() const;

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        Index GetAOHIndex() const
        {return _m_aohI; };
        
        std::string SoftPrint() const;
};


#endif /* !_TYPE_AOHINDEX_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
