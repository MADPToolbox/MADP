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

#ifndef _PRINTTOOLS_H_
#define _PRINTTOOLS_H_ 1

#include <vector>
#include <set>
#include <sstream>
#include <iomanip>
#include "boost/numeric/ublas/vector.hpp"
#include "boost/numeric/ublas/vector_sparse.hpp"

/// PrintTools contains functionality for printing vectors etc.
/** The SoftPrint*() functions returns strings, the Print*() print to
 * standard out. */
namespace PrintTools {

template <class T>
static std::string SoftPrintVector(const T &v)
{
    std::stringstream ss;
    ss << v;
    return(ss.str());
}
    
template <class T>
static std::string SoftPrintVector(const std::vector<T> &v)
{
    std::stringstream ss;
    typename std::vector<T>::const_iterator it =  v.begin();
    typename std::vector<T>::const_iterator last =  v.end();
    ss << "< ";
    while(it != last)
    {
        if(it != v.begin())
            ss << ", ";

        ss << SoftPrintVector(*it);        
        it++;
    }
    ss << " >";
    return(ss.str());
}


template <class T>
static std::string SoftPrintSet(const T &v)
{
    std::stringstream ss;
    ss << v;
    return(ss.str());
}   

template <class T>
static std::string SoftPrintSet(const std::set<T> &v)
{
    std::stringstream ss;
    typename std::set<T>::const_iterator it =  v.begin();
    typename std::set<T>::const_iterator last =  v.end();
    ss << "{ ";
    while(it != last)
    {
        if(it != v.begin())
            ss << ", ";

        ss << SoftPrintSet(*it);        
        it++;
    }
    ss << " }";
    return(ss.str());
}


template <class T>
static std::string SoftPrintVector(const boost::numeric::ublas::mapped_vector<T> &v)
{
    std::stringstream ss;
    typename boost::numeric::ublas::mapped_vector<T>::const_iterator it =  v.begin();
    typename boost::numeric::ublas::mapped_vector<T>::const_iterator last =  v.end();
    ss << "< ";
    while(it != last)
    {
        if(it != v.begin())
            ss << ", ";

        ss << it.index() << ":" << *it;
        it++;
    }
    ss << " >";
    return(ss.str());
}


template <class T>
static std::string SoftPrintVector(const boost::numeric::ublas::compressed_vector<T> &v)
{
    std::stringstream ss;
    typename boost::numeric::ublas::mapped_vector<T>::const_iterator it =  v.begin();
    typename boost::numeric::ublas::mapped_vector<T>::const_iterator last =  v.end();
    ss << "< ";
    while(it != last)
    {
        if(it != v.begin())
            ss << ", ";

        ss << it.index() << ":" << *it;
        it++;
    }
    ss << " >";
    return(ss.str());
}


template <class T>
static std::string SoftPrintVector(const boost::numeric::ublas::coordinate_vector<T> &v)
{
    std::stringstream ss;
    typename boost::numeric::ublas::mapped_vector<T>::const_iterator it =  v.begin();
    typename boost::numeric::ublas::mapped_vector<T>::const_iterator last =  v.end();
    ss << "< ";
    while(it != last)
    {
        if(it != v.begin())
            ss << ", ";

        ss << it.index() << ":" << *it;
        it++;
    }
    ss << " >";
    return(ss.str());
}


/**Prints a vector using cout - i.e. the data type can be written to cout using
 * << (operator<< must be defined for T).*/
template <class T>
static void PrintVectorCout(const T &v)
{
    std::cout << v;
}
    
template <class T>
static void PrintVectorCout(const std::vector<T> &v)
{
    std::cout << SoftPrintVector(v) << std::endl;
}

template <class T>
static void PrintVectorCout(const boost::numeric::ublas::mapped_vector<T> &v)
{
    std::cout << SoftPrintVector(v) << std::endl;
}

template <class T>
static void PrintVectorCout(const boost::numeric::ublas::compressed_vector<T> &v)
{
    std::cout << SoftPrintVector(v) << std::endl;
}

template <class T>
static void PrintVectorCout(const boost::numeric::ublas::coordinate_vector<T> &v)
{
    std::cout << SoftPrintVector(v) << std::endl;
}


template <class T>
static void PrintCout(const T &v)
{
    std::cout << v;
}

template <class T>
static void PrintCout(const std::vector<T> &v)
{
    PrintVectorCout(v);
}

template <class T>
static void PrintCout(const std::set<T> &v)
{
    std::cout << SoftPrint(v);
}

template <class T>
static std::string SoftPrint(const T &v)
{
    std::stringstream ss;
    ss << v;
    return(ss.str());
}

template <class T>
static std::string SoftPrint(const std::vector<T> &v)
{
    return(SoftPrintVector(v));
}

template <class T>
static std::string SoftPrint(const std::set<T> &v)
{
    std::stringstream ss;
    typename std::set<T>::const_iterator it =  v.begin();
    typename std::set<T>::const_iterator last =  v.end();
    ss << "< ";
    while(it != last)
    {
        if(it != v.begin())
            ss << ", ";

        ss << SoftPrint(*it);
        it++;
    }
    ss << " >";
    return(ss.str());
}

template <class T>
static void PrintProgress(T prefix, LIndex i,
                   LIndex nr, size_t interval)
{
    if(i % interval == 0&& i > interval)
    {
        std::cout << prefix << " "<< i << " of " << nr << " - "
                  << std::setprecision(4)
                  << (CastLIndexToDouble(i) / CastLIndexToDouble(nr)) * 100
                  << "%" << std::endl;
    }
}

}

#endif /* !_PRINTTOOLS_H_ */
