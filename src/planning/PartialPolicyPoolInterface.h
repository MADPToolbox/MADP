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
#ifndef _PARTIALPOLICYPOOLINTERFACE_H_
#define _PARTIALPOLICYPOOLINTERFACE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PartialPolicyPoolItemInterface.h"
#include "boost/shared_ptr.hpp"

class PartialPolicyPoolInterface;
typedef boost::shared_ptr<PartialPolicyPoolInterface> PartialPolicyPoolInterface_sharedPtr;

class Interface_ProblemToPolicyDiscretePure;

/**\brief PartialPolicyPoolInterface is an interface for PolicyPools
 * containing Partial Joint Policies.  */
class PartialPolicyPoolInterface 
{
    private:    
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        //PartialPolicyPoolInterface();
        /// Copy constructor.
        //PartialPolicyPoolInterface(const PartialPolicyPoolInterface& a);
        /// Destructor.
        virtual ~PartialPolicyPoolInterface(){};
        /**\brief Copy assignment operator.
         *
         * This must be implemented by the derived class (with this prototype).
         * For an example, see PolicyPoolJPolValPair.
         *
         * For now, this function is purely abstract. Might there be some 
         * members added to this (base) class, then an implementation could
         * be made. This should then be called using
         *          PartialPolicyPoolInterface::operator=(o)
         * from the copy assignment operator of the derived class. See also
         * http://www.icu-project.org/docs/papers/cpp_report/the_assignment_operator_revisited.html.
         */        
        virtual PartialPolicyPoolInterface& operator= (const PartialPolicyPoolInterface& o)=0;

        //operators:

        //data manipulation (set) functions:

        /**\brief Initializes the Policy pool with the empty joint policy
         *
         * A pointer to a Interface_ProblemToPolicyDiscretePure is needed to create the
         * joint policy.
         */
        virtual void Init(const Interface_ProblemToPolicyDiscretePure* pu)=0; 

        /**\brief The 'Select' operator from #refGMAA.
         *
         * This returns a reference to the next PolicyPoolItem (a wrapper for a
         * partial joint policy, together with some properties).
         *
         * The returned PolicyPoolItem is not removed from the PolicyPool.
         */
        virtual PartialPolicyPoolItemInterface_sharedPtr Select()const=0;        
        /**\brief Removes the item returned by 'Select'.
         *
         * This removes the next PolicyPoolItem (a wrapper for a
         * partial joint policy, together with some properties), as would
         * be returned by 'Select'.
         */
         virtual void Pop(PartialPolicyPoolItemInterface_sharedPtr ppiToBeRemoved = 
                          PartialPolicyPoolItemInterface_sharedPtr())=0;
        /**\brief returns the contained item with the highest value.
         *
         * This function returns a pointer to the PartialPolicyPoolItemInterface
         * contained in this Policy pool with the highest (heuristic) value.
         *
         * Heuristic is between brackets, because this function is typically
         * used when we found lower bounds (i.e. full policies) and then select
         * the maximum lowerbound.
         */
        virtual PartialPolicyPoolItemInterface_sharedPtr GetBestRanked() const=0;
        /**\brief remove the GetBestRanked() item
         *
         * Removes the PartialPolicyPoolItemInterface contained
         * in this Policy pool with the highest (heuristic) value.
         * (as is returned by GetBestRanked() )
         */
        virtual void PopBestRanked()=0;

        /**\brief Add a PolicyPoolItem to the Pool.
         *
         * ...
         */
        virtual void Insert(PartialPolicyPoolItemInterface_sharedPtr  ppi)=0;


        /**\brief add all elements of pp to 'this'.
         *
         *...
         */
        virtual void Union(PartialPolicyPoolInterface_sharedPtr  pp)=0;
        /**\brief prune the items in the policy pool with exp.value < v
         *
         *...
         */
        virtual void Prune(double v)=0;       
        //get (data) functions:
        
        /**\brief return teh number of items in the policy pool
         *
         * ...
         */
        virtual size_t Size() const = 0;
        /**\brief return whether the number of items in the policy pool is 0
         *
         * ...
         */
        size_t Empty() const
        {return(Size()==0);};

        virtual std::string SoftPrint() const = 0;
};


#endif /* !_PARTIALPOLICYPOOLINTERFACE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
