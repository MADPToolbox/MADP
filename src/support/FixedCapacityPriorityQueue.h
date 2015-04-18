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
#ifndef _FIXEDCAPACITYPRIORITYQUEUE_H_
#define _FIXEDCAPACITYPRIORITYQUEUE_H_ 1

/* the include directives */
#include "Globals.h"
#include <list>

/** \brief FixedCapacityPriorityQueue is a class that represents 
 * a priority queue with a fixed size.
 *
 * -all overflowed items (i.e., items that do not fit) 
 *  are returned, such that the user can delete them
 *
 * 
 */
template <typename T, typename _Compare = std::less<T> >
class FixedCapacityPriorityQueue 
{
    private:    
        std::list <T> _m_l;
        size_t _m_capacity;
    
    protected:
    
    public:

        // Constructor, destructor and copy assignment.
        // 
        /// (default) Constructor
        FixedCapacityPriorityQueue(size_t capacity)
            : _m_capacity(capacity)
        {
        };
        //operators:

        //data manipulation (set) functions:
        /**\brief inserts a in the priority q. 
         *
         * returns true if the capacity overflows
         * in this case, overflown_T is set to the overflown value
         * (which can then be recovered or deleted by the user).
         */
        bool insert( T& a, T& overflown_T );
        
        //get (data) functions:
        bool empty() const   {return _m_l.empty();}
        const T& top() const { return _m_l.front(); } //highest priority (value) at the front
        void pop() { _m_l.pop_front(); }

        const T& back() const {return _m_l.back(); }

        size_t size() const {return _m_l.size();}

        std::string SoftPrint() const
        {
            std::stringstream ss;
            size_t i=0;
            typename std::list<T>::const_iterator it = _m_l.begin();
            while(it!=_m_l.end())
            {
                ss << "FixedCapacityPriorityQueue item number " << i << std::endl;
                ss << (*it)->SoftPrint() << std::endl;
                it++;
                i++;
            }
            return(ss.str());
        }
};

#define DEBUG_FCPQ 0
//template <class T>
template <typename T, typename _Compare >
bool FixedCapacityPriorityQueue<T,_Compare>::insert( T& a, T& overflown_T )
{
    bool overflow = ( _m_l.size() == _m_capacity);
    bool skip_insert = false;

    if(overflow) //check if we need to make space at the end
    {
        // overflown_Tp is the T* to which overflown_Tpp points
        // we set the former to point to the overflown element.
        //T* & overflown_Tp = *overflown_Tpp;

        T& last_in_queue = _m_l.back();
        //std::less< T > theLessOp;
        _Compare theLessOp;
        if( theLessOp( last_in_queue,  a) )
        //if( last_in_queue <  a ) //doesn't work automatically...
        {
            //This does not work with (or need) pointers, 
            //this simply points to last position in queue !!!
            //overflown_Tp = &last_in_queue;             
            
            //rather copy the value of the stuff we will throw out!
            overflown_T = last_in_queue; 
            _m_l.pop_back();
        }
        else
        {
            overflown_T = a;
            skip_insert = true;
        }
    }

    if(!skip_insert)
    {
        //insert a at the appropriate place
        typename std::list<T>::iterator it = _m_l.begin();
        typename std::list<T>::iterator last = _m_l.end();
        bool not_positioned = true; //so long as it does not point to the correct pos.
        //std::less< T > theLessOp;
        _Compare theLessOp;
        while(it != last && not_positioned)
        {
            if( theLessOp( *it, a ) )
                not_positioned = false;
            else
                it++;
        }
        _m_l.insert( it, a);
    }
#if 0 && DEBUG_FCPQ
    std::cout << "----AFTER POP AND INSERT\nthe overflown_Tp=" <<overflown_Tp << ", which means it points to..." <<std::endl ;
    if(overflown_Tp != NULL)
    {
        T& the_overflown_T = (*overflown_Tp);
        std::string typestr = " JPPVValPair* ";
        std::cout << "the_overflown_T [T="<< typestr <<
            "]=" << the_overflown_T << ", which points to..."<<std::endl;

        if(the_overflown_T != NULL)
            std::cout<< (*the_overflown_T).SoftPrintBrief() << std::endl;
        else
            std::cout << "nothing" <<std::endl;
    }
    else
        std::cout << " nothing." << std::endl;
    
    std::cout << "----" <<std::endl ;
#endif
    
    return(overflow);
}

#endif /* !_FIXEDCAPACITYPRIORITYQUEUE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
