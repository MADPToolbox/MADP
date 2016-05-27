/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _TYPECLUSTER_H_
#define _TYPECLUSTER_H_ 1

/* the include directives */
#include "Globals.h"
#include "Type.h"

/** \brief TypeCluster is a class that represents a cluster of Types.
 * This is in particular used in a BayesianGameWithClusterInfo. */
class TypeCluster 
{
    public:
        typedef std::set< Type* >::const_iterator type_ci;
        typedef std::set< Type* >::iterator type_i;
    private:    
        /**\brief the set of tuples that form this cluster type.
         */
        std::set< Type* > _m_types;

        /**\brief The index of this TypeCluster within the TypeClusterList of 
         * the relevant agent
         */
        Index _m_i;

    
    protected:
    
    public:        
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        TypeCluster(Index i);
        /// Copy constructor.
        TypeCluster(const TypeCluster& a);
        /// Destructor.
        ~TypeCluster();
        /// Copy assignment operator
        TypeCluster& operator= (const TypeCluster& o);

        //operators:

        //data manipulation (set) functions:
        ///Add type t to this cluster.
        void AddType(Type* t)
        { _m_types.insert(t);}

        void Merge(TypeCluster* tc)
        { _m_types.insert(tc->begin(), tc->end() );}

        type_ci begin() const
        {return _m_types.begin();}
        type_i begin()
        {return _m_types.begin();}

        type_ci end() const
        {return _m_types.end();}
        type_i end()
        {return _m_types.end();}

        void clear()
        {_m_types.clear();}
        
        void SetIndex(Index i )
        { _m_i=i; };
        //get (data) functions:
        Index GetIndex() const
        {return _m_i;};
        //
        std::string SoftPrint () const;
};

typedef std::vector<TypeCluster* > TypeClusterList;
typedef std::vector<TypeCluster* >::iterator typeC_i;
typedef std::vector<TypeCluster* >::const_iterator typeC_ci;

#endif /* !_TYPECLUSTER_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
