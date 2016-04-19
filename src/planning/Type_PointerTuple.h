/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
