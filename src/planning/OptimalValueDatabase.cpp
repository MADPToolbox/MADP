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

#include "OptimalValueDatabase.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "directories.h"
#include <fstream>

using namespace std;

//Default constructor
OptimalValueDatabase::OptimalValueDatabase(
    const PlanningUnitDecPOMDPDiscrete* pu) :
    _m_problemName(pu->GetDPOMDPD()->GetUnixName()),
    _m_discount(pu->GetDiscount()),
    _m_horizon(pu->GetHorizon())
{
    Load();
}

OptimalValueDatabase::OptimalValueDatabase(
    const string &problemName,
    double discount,
    size_t horizon) :
                   _m_problemName(problemName),
                   _m_discount(discount),
                   _m_horizon(horizon)
{
    Load();
}

void OptimalValueDatabase::AddEntry(const std::string &problemName,
                                    double discount,
                                    size_t horizon,
                                    double value)
{
    nameDiscountT pairNameDiscount(problemName,discount);
    nameDiscountHorizonT nameDiscountHorizon(pairNameDiscount,
                                             horizon);
    _m_optimalValues.insert(
        pair<nameDiscountHorizonT, double>(nameDiscountHorizon,value));
}

void OptimalValueDatabase::SetOptimalValue(double value)
{
    AddEntry(_m_problemName,
             _m_discount,
             _m_horizon,
             value);
    Save();
}

double OptimalValueDatabase::GetEntry(const std::string &problemName,
                                      double discount,
                                      size_t horizon) const
{
    nameDiscountT pairNameDiscount(problemName,discount);
    nameDiscountHorizonT nameDiscountHorizon(pairNameDiscount,
                                             horizon);

    map<nameDiscountHorizonT,double>::const_iterator iter=
        _m_optimalValues.find(nameDiscountHorizon);
    if( iter != _m_optimalValues.end() )
        return(iter->second);
    else
    {
        stringstream ss;
        ss << "OptimalValueDatabase::GetEntry() no entry exists for "
           << problemName << " discount " << discount << " horizon "
           << horizon;
        throw(E(ss));
    }

    // should not get here
    return(0);
}
double
OptimalValueDatabase::GetOptimalValue() const
{
    return(GetEntry(_m_problemName,
                    _m_discount,
                    _m_horizon));
}

bool OptimalValueDatabase::IsOptimal(const std::string &problemName,
                                     double discount,
                                     size_t horizon,
                                     double value) const
{
    if(EqualReward(value,
                   GetEntry(problemName,discount,horizon)))
        return(true);
    else
        return(false);
}
bool OptimalValueDatabase::IsOptimal(double value) const
{
    return(IsOptimal(_m_problemName,
                     _m_discount,
                     _m_horizon,
                     value));
}


bool OptimalValueDatabase::IsInDatabase(const std::string &problemName,
                                        double discount,
                                        size_t horizon) const
{
    nameDiscountT pairNameDiscount(problemName,discount);
    nameDiscountHorizonT nameDiscountHorizon(pairNameDiscount,
                                             horizon);

    map<nameDiscountHorizonT,double>::const_iterator iter=
        _m_optimalValues.find(nameDiscountHorizon);
    if( iter == _m_optimalValues.end() )
        return(false);
    else
        return(true);
}
bool 
OptimalValueDatabase::IsInDatabase() const
{
    return(IsInDatabase(_m_problemName,
                        _m_discount,
                        _m_horizon));
}

string OptimalValueDatabase::SoftPrint() const
{
    stringstream ss;
    ss.precision(16);

    map<nameDiscountHorizonT,double>::const_iterator iter=
        _m_optimalValues.begin();
    while( iter != _m_optimalValues.end() )
    {
        double value=iter->second;
        nameDiscountHorizonT x=iter->first;
        size_t horizon=x.second;
        nameDiscountT y=x.first;
        string problemName=y.first;
        double discount=y.second;
        ss << problemName << " " << discount << " "
           << horizon << " " << value << endl;

        ++iter;
    }

    return(ss.str());
}

void OptimalValueDatabase::Load()
{
    _m_optimalValues.clear();
    ifstream fp;
    _filename="optimalValueDatabase"; // look in current directory first
    fp.open(_filename.c_str(),ios_base::in);
    if(!fp)
    {
        stringstream filename;
        filename << directories::MADPGetResultsDir() // otherwise look in results dir
                 << "/GMAA/optimalValueDatabase";
        _filename=filename.str();
        fp.open(_filename.c_str(),ios_base::in);
    }
    if(!fp)
        return;

    string problemName;
    double discount, value;
    size_t horizon;
    bool first=true;
    string buffer;
    while(!getline(fp,buffer).eof())
    {
        // first line is comment
        if(first)
        {
            first=false;
            continue;
        }
        istringstream is(buffer);
        is >> problemName;
        is >> discount;
        is >> horizon;
        is >> value;

        AddEntry(problemName,discount,horizon,value);
    }
}

void OptimalValueDatabase::Save() const
{
    ofstream fp(_filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "OptimalValueDatabase::Save() failed to open file "
           << _filename << endl;
        throw E(ss.str());
    }

    fp.precision(16);
    
    fp << "problem discount horizon value" << endl;
    fp << SoftPrint();
}

string OptimalValueDatabase::GetEntryName() const
{
    stringstream ss;
    ss.precision(16);
    ss<<_m_problemName<<" "<<_m_discount<<" "<<_m_horizon;
    return ss.str();
}
