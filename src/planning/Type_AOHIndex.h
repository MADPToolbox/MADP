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
