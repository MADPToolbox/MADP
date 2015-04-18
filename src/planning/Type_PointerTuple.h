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
#ifndef _TYPE_POINTERTUPLE_H_
#define _TYPE_POINTERTUPLE_H_ 1

/* the include directives */
#include "Globals.h"
#include "Type.h"

class TypeCluster;

/** \brief Type_PointerTuple is a implementation (extenstion) of Type and 
 * represents a type in e.g. a BG.
 * 
 * In particular this implementation of Type is a tuple <p, a, o>
 * where p is a pointer to a type of the (BG of the) previous stage, a is
 * the action taken and o the observation received since then.
 * */
class Type_PointerTuple : public Type
{
    private:   
        ///The previous TypeCluster
        const TypeCluster* _m_pred;
        ///The (index of) the individual action taken since previous_p
        Index _m_aI; 
        ///The (index of) the received individual observation since previous_p
        Index _m_oI;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        Type_PointerTuple(const TypeCluster* _m_pred, Index aI, Index oI);
        /// Copy constructor.
        Type_PointerTuple(const Type_PointerTuple& a);
        /// Destructor.
        virtual ~Type_PointerTuple();
        /// Copy assignment operator
        Type_PointerTuple& operator= (const Type_PointerTuple& o);
        Type* Clone() const;

        //operators:

        //data manipulation (set) functions:
        
        //get (data) functions:
        const TypeCluster* GetPredecessor() const
        {return _m_pred;};
        Index GetAction() const
        {return _m_aI;};
        Index GetObservation() const
        {return _m_oI;};
        
        ///SoftPrints all of the represented histories recursively 
        std::string SoftPrint() const;
};


#endif /* !_TYPE_POINTERTUPLE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
