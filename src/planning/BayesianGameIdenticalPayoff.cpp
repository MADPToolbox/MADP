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

#include "BayesianGameIdenticalPayoff.h"
#include <fstream>
#include "JointPolicyDiscretePure.h"
#include "RewardModelMapping.h"
#include "RewardModelMappingSparseMapped.h"

using namespace std;

//Default constructor
BayesianGameIdenticalPayoff::BayesianGameIdenticalPayoff()
{
    _m_initialized=false;
}

BayesianGameIdenticalPayoff::BayesianGameIdenticalPayoff(size_t nrAgents, 
                                                         const vector<size_t>& nrActions, 
                                                         const vector<size_t>& nrTypes,
                                                         bool useSparseRewardModel) :
    BayesianGameIdenticalPayoffInterface(nrAgents, nrActions, nrTypes),
    _m_utilFunction(0)
{
    _m_initialized=false;

    if(useSparseRewardModel)
        _m_utilFunction=new RewardModelMappingSparseMapped(_m_nrJTypes, _m_nrJA, "type", "ja");
    else
        _m_utilFunction=new RewardModelMapping(_m_nrJTypes, _m_nrJA, "type", "ja");
}

BayesianGameIdenticalPayoff::~BayesianGameIdenticalPayoff()
{
    delete _m_utilFunction;
}

BayesianGameIdenticalPayoff& BayesianGameIdenticalPayoff::operator= (const BayesianGameIdenticalPayoff& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment

    BayesianGameIdenticalPayoffInterface::operator=(o);
    _m_initialized=o._m_initialized;
    _m_utilFunction=o._m_utilFunction->Clone();

    return *this;
}

bool BayesianGameIdenticalPayoff::SetInitialized(bool b)
{
    _m_initialized = b;
    return(true);
}

double BayesianGameIdenticalPayoff::ComputeValueJPol(const JointPolicyDiscretePure & jpolBG) const
{
    double v=0.0;
    for(Index jtI=0; jtI<GetNrJointTypes(); jtI++)
    {
        Index jaI = jpolBG.GetJointActionIndex(jtI);
        double p = GetProbability(jtI);
        double u = GetUtility(jtI, jaI);
        v += p*u;
    }
    return(v);

}
string BayesianGameIdenticalPayoff::SoftPrintUtilForJointType(Index jtype) const
{
    stringstream ss;
    ss << "Utility function for jtype "<<jtype << 
        "with prob.="<< GetProbability(jtype) << endl;
    ss << "jtype\tja\tu(jtype,ja)"<<endl;
    for(Index jaI=0; jaI < _m_nrJA; jaI++)
        ss << jtype << "\t" << jaI << "\t" << GetUtility(jtype, jaI)<<endl;
    return(ss.str());
}

string BayesianGameIdenticalPayoff::SoftPrint() const
{
    stringstream ss;
    ss << BayesianGameBase::SoftPrint();    
    ss << "Utility function:"<<endl;
    for(Index jtype=0; jtype < _m_nrJTypes; jtype++)
    {
        for(Index jaI=0; jaI < _m_nrJA; jaI++)
            ss << GetUtility(jtype,jaI) << " ";
        ss << endl;
    }
//    ss << _m_utilFunction->SoftPrint();
    return(ss.str());
}

void BayesianGameIdenticalPayoff::Save(const BayesianGameIdenticalPayoff &bg,
                                       const string &filename)
{
    SaveTextFormat(bg,filename);
}

void BayesianGameIdenticalPayoff::SaveTextFormat(const BayesianGameIdenticalPayoff &bg,
                                                 const string &filename)
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "BayesianGameIdenticalPayoff::SaveTextFormat could not open "
           << filename;
        throw(E(ss.str()));
    }

    fp << "# BayesianGameIdenticalPayoff text format." << endl;
    fp << bg.GetNrAgents() << endl;
    for(Index i=0;i!=bg.GetNrAgents();++i)
    {
        fp << bg.GetNrActions(i);
        if(i!=(bg.GetNrAgents()-1))
            fp << " ";
    }
    fp << endl;
    for(Index i=0;i!=bg.GetNrAgents();++i)
    {
        fp << bg.GetNrTypes(i);
        if(i!=(bg.GetNrAgents()-1))
            fp << " ";
    }
    fp << endl;
    fp.precision(16);
    for(Index jtI=0;jtI!=bg.GetNrJointTypes();++jtI)
    {
        fp << bg.GetProbability(jtI);
        if(jtI!=(bg.GetNrJointTypes()-1))
            fp << " ";
    }
    fp << endl;
    for(Index jtI=0;jtI!=bg.GetNrJointTypes();++jtI)
    {
        for(Index jaI=0; jaI < bg.GetNrJointActions(); ++jaI)
        {
            fp << bg.GetUtility(jtI,jaI);
            if(jaI!=(bg.GetNrJointActions()-1))
                fp << " ";
        }
        fp << endl;
    }
}

BayesianGameIdenticalPayoff BayesianGameIdenticalPayoff::Load(const string &filename)
{
    return(LoadTextFormat(filename));
}

BayesianGameIdenticalPayoff BayesianGameIdenticalPayoff::LoadTextFormat(const string &filename)
{
    ifstream fp(filename.c_str());
    if(!fp)
    {
        cerr << "BayesianGameIdenticalPayoff::LoadTextFormat: failed to "
             << "open file " << filename << endl;            
    }

    size_t nrAgents=0;
    vector<size_t> nrActions, nrTypes;

    size_t tempInt=0;
    double tempDouble=0;
    size_t lineNumber=0;
    BayesianGameIdenticalPayoff bg;
    Index jtI=0,jaI=0;

    string buffer;
    while(!getline(fp,buffer).eof())
    {
        istringstream is(buffer);
        switch(lineNumber)
        {
        case 0:
            // skip first line (header)
            break;
        case 1:
            is >> nrAgents;
            break;
        case 2:
            while(is >> tempInt)
                nrActions.push_back(tempInt);
            if(nrActions.size()!=nrAgents)
                throw(E("BayesianGameIdenticalPayoff::LoadTextFormat wrong number of actions"));
            break;
        case 3:
            while(is >> tempInt)
                nrTypes.push_back(tempInt);
            if(nrTypes.size()!=nrAgents)
                throw(E("BayesianGameIdenticalPayoff::LoadTextFormat wrong number of types"));

            // now we have all the info to instantiate the BG
            bg=BayesianGameIdenticalPayoff(nrAgents,nrActions,nrTypes);
            break;
        case 4:
            jtI=0;
            while(is >> tempDouble)
            {
                bg.SetProbability(jtI,tempDouble);
                jtI++;
            }
            if(jtI!=bg.GetNrJointTypes())
                throw(E("BayesianGameIdenticalPayoff::LoadTextFormat wrong number of joint type probabilities"));
            jtI=0;
            break;
        default:
            jaI=0;
            while(is >> tempDouble)
            {
                bg.SetUtility(jtI,jaI,tempDouble);
                jaI++;
            }
            if(jaI!=bg.GetNrJointActions())
                throw(E("BayesianGameIdenticalPayoff::LoadTextFormat wrong number of actions in utility function"));
            jtI++;
            break;
        }

        lineNumber++;
    }
        
    if(jtI!=bg.GetNrJointTypes())
        throw(E("BayesianGameIdenticalPayoff::LoadTextFormat wrong number of joint types in utility function"));

    bg.SetInitialized(true);

    return(bg);
}

BayesianGameIdenticalPayoff 
BayesianGameIdenticalPayoff::GenerateRandomBG(
            size_t nrAgents,
            std::vector<size_t> acs,
            std::vector<size_t> obs
        )
{
    BayesianGameIdenticalPayoff bgip(nrAgents, acs, obs);
    for(Index jtype = 0; jtype < bgip.GetNrJointTypes(); jtype++)
        for(Index ja = 0; ja < bgip.GetNrJointActions(); ja++)
        {
            double rn = (rand() - (0.5 * RAND_MAX)) / (RAND_MAX / 20.0);
            bgip.SetUtility(jtype, ja, rn );
        }

    //create random, normalized prob distr.
    vector<double> typeProbs;
    double sum = 0.0;
    for(Index jtype = 0; jtype < bgip.GetNrJointTypes(); jtype++)
    {
        double r = ((double)rand()) / RAND_MAX;
        typeProbs.push_back(r);
    }
    for(Index jtype = 0; jtype < bgip.GetNrJointTypes(); jtype++)
        sum += typeProbs[jtype];
    for(Index jtype = 0; jtype < bgip.GetNrJointTypes(); jtype++)
        bgip.SetProbability(jtype, typeProbs[jtype] / sum );

    //bgip.Print();
    return(bgip);
}
