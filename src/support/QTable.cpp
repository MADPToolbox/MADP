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
 * Matthijs Spaan 
 *
 * For contact information please see the included AUTHORS file.
 */

#include "QTable.h"
#include <fstream>

using namespace std;

void QTable::Load(const std::string &filename,
                  size_t nrRows,
                  size_t nrColumns,
                  QTable &Q)
{
    ifstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "QTable::Load: failed to "
             << "open file " << filename << endl;            
    }

    size_t a,s;
    double q;

//    QTable Q(nrRows,nrColumns);
    try { 
        Q.resize(nrRows,nrColumns);
    }
    catch(std::bad_alloc &e)
    {
        cout << "QTable::Load ran out of memory resizing QTable" << endl;
        throw(e);
    }

    s=0;
    string buffer;
    while(!getline(fp,buffer).eof())
    {
        istringstream is(buffer);
        a=0;
        while(is >> q)
            Q(s,a++)=q;

        if(a!=nrColumns)
            throw(E("QTable::Load wrong number of columns"));

        s++;
    }
    
    if(s!=nrRows)
        throw(E("QTable::Load wrong number of rows"));
}

void QTable::Load(const std::string &filename,
                  size_t nrRows,
                  size_t nrColumns,
                  size_t nrTables,
                  QTables &Qs)
{
    ifstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "QTable::Load: failed to "
             << "open file " << filename << endl;            
    }

    size_t a,s,i;
    double q;

    QTable Q(nrRows,nrColumns);
//    QTables Qs;
    Qs.clear();
    for(i=0;i!=nrTables;i++)
        Qs.push_back(Q);
    
    s=0;
    i=0;
    string buffer;
    while(!getline(fp,buffer).eof())
    {
        istringstream is(buffer);
        a=0;
        while(is >> q)
            Qs[i](s,a++)=q;

        if(a!=nrColumns)
            throw(E("QTable::Load wrong number of columns"));

        s++;
        if(s==nrRows)
        {
            i++;
            s=0;
        }
    }
    
    if(i!=nrTables)
        throw(E("QTable::Load wrong number of tables"));
}

void QTable::Save(const QTable &Q, const std::string &filename)
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "QTable::Save: failed to open file " << filename << endl;
        throw E(ss.str());
    }

    fp.precision(16);

    size_t nrRows=Q.size1(),
        nrColumns=Q.size2();
   
    for(size_t s=0;s!=nrRows;++s)
    {
        for(size_t a=0;a!=nrColumns;++a)
        {
            fp << Q(s,a);
            if(a!=nrColumns-1)
                fp << " ";
        }
        fp << endl;
    }
}

void QTable::Save(const QTables &Qs, const std::string &filename)
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "QTable::Save: failed to open file " << filename << endl;
        throw E(ss);
    }

    fp.precision(16);

    size_t nrRows=Qs[0].size1(),
        nrColumns=Qs[0].size2(),
        h=Qs.size();

    for(size_t k=0;k!=h;++k)
        for(size_t s=0;s!=nrRows;++s)
        {
            for(size_t a=0;a!=nrColumns;++a)
            {
                fp << Qs[k](s,a);
                if(a!=nrColumns-1)
                    fp << " ";
            }
            fp << endl;
        }
}
    
void QTable::SetToZero()
{
    size_t nrRows=this->size1(),
        nrColumns=this->size2();
   
    for(size_t s=0;s!=nrRows;++s)
        for(size_t a=0;a!=nrColumns;++a)
            Set(s,a,0.0);
}
