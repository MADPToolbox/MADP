/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _SCOPE_H_
#define _SCOPE_H_ 1

#include "Globals.h"
#include <vector>
#include <iostream>
#include <string>


//typedef std::vector<Index> Scope;

//we only use this here, so in unnamed namespace:
namespace{
    //the scope data type
    typedef std::vector<Index> SDT;
}

class Scope : public SDT
{
    private:

    public:
    ///Default constructor
    Scope() : SDT()
    {}

    //construct from a vector Index
    Scope( SDT& o) : SDT(o)
    {}
    
    //construct from a range
    template<typename InputIterator> 
    Scope (InputIterator first, InputIterator last,
	       const allocator_type& a = allocator_type())
        : 
            SDT (first, last, a)
    {}

    //constuct from a string, e.g. Scope s("<1 54 2>")
    Scope(const std::string &s);

    //construct from a given size
    Scope( size_t size) : SDT(size)
    {}
    
    //copy assign
    Scope& operator= (const Scope& o)
    { this->SDT::operator=(o); return *this;}
    //copy assign from a vector<Index>
    Scope& operator= (const SDT& o)
    { this->SDT::operator=(o); return *this;}

    ///Insert an index into the scope
    /**Note this does *not* check for duplicates!
     */
    void Insert(Index i)
    {
        push_back(i);
    }
    ///Removes all indices in s from \em this.
    void Remove(const Scope& s);
    ///Merges all indices in s into \em this.
    void Insert(const Scope& s);
    ///Whether \em this contains Index i
    bool Contains(Index i) const;
    ///Whether this is a subset of s
    bool IsSubSetOf(const Scope& s) const;
    ///Whether two scopes are equal (i.e., with same ordering)
    bool Equals(const Scope& s) const;
    ///Returns a scope containing the intersection of a and b
    static Scope Intersection(const Scope& a, const Scope& b);
    ///Returns iterator to index \a i in this (or end() if i not contained)
    SDT::iterator GetIteratorForIndex(Index i) ;
    ///Returns the position of index i within \e this.
    /**or throws an exception if \e this does not contain i.
     */
    Index GetPositionForIndex(Index i) const;
    ///Sorts the indices.
    void Sort();

    std::string SoftPrint() const;


    friend std::ostream& operator<< (std::ostream& o, const Scope& s);
    friend std::istream& operator>> (std::istream& i, Scope& s);

    
};
typedef std::vector<Scope> Scopes;

#endif /* !_SCOPE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
