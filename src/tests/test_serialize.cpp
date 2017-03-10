/* This file is part of the Multiagent Decision Process (MADP) Toolbox. 
 *
 * The majority of MADP is free software released under GNUP GPL v.3. However,
 * some of the included libraries are released under a different license. For 
 * more information, see the included COPYING file. For other information, 
 * please refer to the included README file.
 *
 * This file has been written and/or modified by the following people:
 *
 * Frans Oliehoek 
 * Bas Terwijn
 *
 * For contact information please see the included AUTHORS file.
 */


/*
Testing how to use boost::serialization to save and load TwoStatgeDynamicBayesianNetwork objects to disk
*/
#include <vector>
#include <iostream>
using namespace std;

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

#include "boost/numeric/ublas/matrix.hpp"
#include <boost/numeric/ublas/io.hpp> 

namespace{
    //the scope data type
    typedef vector<int> SDT;
}

template <typename T>
std::ostream& operator<< (std::ostream& os, const vector<T*>& v)
{
    os<<"[";
    for (typename vector<T*>::const_iterator it=v.begin();it!=v.end();it++)
    {
        if (it!=v.begin()) os<<";";
        os<<*(*it);
    }
    os<<"]";
    return os;
}

template <typename T>
std::ostream& operator<< (std::ostream& os, const vector<T>& v)
{
    os<<"[";
    for (typename vector<T>::const_iterator it=v.begin();it!=v.end();it++)
    {
        if (it!=v.begin()) os<<",";
        os<<*it;
    }
    os<<"]";
    return os;
}

class Scope : public SDT
{
    // boost serialization
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        // having to serialize base object as Scope oddly inherits from SDT instead of using composition
        ar & boost::serialization::make_nvp("Scope", boost::serialization::base_object<SDT>(*this) );
    }

public:
    Scope()
    {
        for (int i=0;i<10;i++) // some test data
            push_back(i); 
    }
    
};
BOOST_CLASS_VERSION(Scope, 1)

class CPDDiscreteInterface
{
    // boost serialization
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
    }

public:
    virtual void fill()
    {}

    virtual void print(std::ostream& os) const
    {
    }

friend std::ostream& operator<< (std::ostream& os, const CPDDiscreteInterface& cpd);

};

std::ostream& operator<< (std::ostream& os, const CPDDiscreteInterface& cpd)
{
    cpd.print(os);
    return os;
}

class CPT : public CPDDiscreteInterface
{
    typedef boost::numeric::ublas::matrix<double> Matrix;
    Matrix _m_probTable;

    // boost serialization
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(CPDDiscreteInterface)
            & BOOST_SERIALIZATION_NVP(_m_probTable);
    }

public:
    CPT()
        : _m_probTable(2,2)
    {
    }

    virtual void fill()
    {
        for (int x=0;x<2;x++)
            for (int y=0;y<2;y++)
                _m_probTable(x,y)=x+y;
    }

    virtual void print(std::ostream& os) const
    {
        os<<_m_probTable<<endl;
    }

};
BOOST_CLASS_EXPORT(CPT); // explicitly instantiated subclass for serialization


// Bayesian network with different vectors
class BN
{
    vector< Scope > v1;
    vector< Scope > v2;
    std::vector< std::vector<size_t> > vv;
    std::vector< CPDDiscreteInterface* > CPDs;

    // boost serialization
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version){
        ar & BOOST_SERIALIZATION_NVP(v1)
            & BOOST_SERIALIZATION_NVP(v2)
            & BOOST_SERIALIZATION_NVP(vv)
            & BOOST_SERIALIZATION_NVP(CPDs)
            ;
    }

public:
    
    BN()
    {
        for (int i=0;i<3;i++)
        {
            CPDs.push_back(new CPT());
            //CPDs.push_back(new CPDDiscreteInterface());
        }
    }

    ~BN()
    {
    }

    void fill() // some test data
    {
        Scope s;
        v1.push_back(s);
        v2.push_back(s);
        v2.push_back(s);
        vector<size_t> v;
        for (int i=0;i<5;i++)
        {
            v.push_back(i);
            vv.push_back(v);
        }
        for (vector< CPDDiscreteInterface * >::iterator it=CPDs.begin();it!=CPDs.end();it++)
            (*it)->fill();
        //for (vector<CPDDiscreteInterface>::iterator it=CPDs.begin();it!=CPDs.end();it++)
        //    it->fill();
    }

    friend std::ostream& operator<< (std::ostream& os, const BN& bn);
};
BOOST_CLASS_VERSION(BN, 1)

std::ostream& operator<< (std::ostream& os, const BN& bn)
{
    os<<bn.v1<<endl;
    os<<bn.v2<<endl;
    os<<bn.vv<<endl;
    os<<bn.CPDs<<endl;
    return os;
}

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <fstream>
#include <sstream>

int main()
{
    char* filename=(char*)"./test_serialize.xml";
        
    stringstream saved,loaded;
    {
        cout<<"saving:"<<endl;
        BN s;
        s.fill(); // add test data
        cout<<s<<endl;
        saved<<s;
        ofstream ofs(filename);
        boost::archive::xml_oarchive oa(ofs);
        oa << BOOST_SERIALIZATION_NVP(s);// save
    } 

    {
        cout<<"loading:"<<endl;
        BN s;
        ifstream ifs(filename);
        boost::archive::xml_iarchive ia(ifs);
        ia >> BOOST_SERIALIZATION_NVP(s);// load
        cout<<s<<endl;
        loaded<<s;
    }

    if (saved.str()!=loaded.str())
        return 1;

    return 0;
}
