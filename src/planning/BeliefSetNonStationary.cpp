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

#include "BeliefSetNonStationary.h"
#include "JointBeliefSparse.h"

using namespace std;

BeliefSetNonStationary::BeliefSetNonStationary(size_t nrTimeSteps) :
    _m_beliefSets(nrTimeSteps+1)
{
    if(nrTimeSteps==MAXHORIZON)
        throw(E("BeliefSetNonStationary does not make sense in infinite-horizon setting"));
}

//Copy constructor.    
BeliefSetNonStationary::BeliefSetNonStationary(const BeliefSetNonStationary& o) 
{
    _m_beliefSets.clear();
    for(Index t=0;t!=o._m_beliefSets.size();++t)
    {
        BeliefSet St;
        for(Index j=0;j!=o._m_beliefSets[t].size();j++)
            St.push_back(new JointBeliefSparse(*o._m_beliefSets[t][j]));
        _m_beliefSets.push_back(St);
    }
}

//Destructor
BeliefSetNonStationary::~BeliefSetNonStationary()
{
    for(Index t=0;t!=_m_beliefSets.size();++t)
        for(Index j=0;j!=_m_beliefSets[t].size();j++)
            delete _m_beliefSets[t][j];
}

//Copy assignment operator
BeliefSetNonStationary& BeliefSetNonStationary::operator= (const BeliefSetNonStationary& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    _m_beliefSets.clear();
    for(Index t=0;t!=o._m_beliefSets.size();++t)
    {
        BeliefSet St;
        for(Index j=0;j!=o._m_beliefSets[t].size();j++)
            St.push_back(new JointBeliefSparse(*o._m_beliefSets[t][j]));
        _m_beliefSets.push_back(St);
    }

    return *this;
}

size_t BeliefSetNonStationary::Size(Index t) const
{
    return(_m_beliefSets.at(t).size());
}
    
size_t BeliefSetNonStationary::Size() const
{
    size_t size=0;
    for(Index t=0;t!=_m_beliefSets.size();++t)
        size+=Size(t);
    return(size);
}

bool BeliefSetNonStationary::Add(size_t t, const JointBeliefInterface &jb,
                                 bool uniquify)
{
    if(t>_m_beliefSets.size())
        throw(E("BeliefSetNonStationary::Add t is too high"));

    if(!jb.SanityCheck())
        throw(E("BeliefSetNonStationary::Add belief fails sanity check"));

    bool added=false, addBelief, equal;
    if(uniquify)
    {
        bool foundEqualBelief=false;
        // loop over all beliefs already in S
        for(Index j=0;j!=_m_beliefSets[t].size();j++)
        {
            equal=true;
            for(Index s=0;s!=jb.Size();s++)
            {
                // if one number differs we can move on to the next j
                if(_m_beliefSets[t][j]->Get(s)!=jb.Get(s))
                {
                    equal=false;
                    break;
                }
            }
            // if we found an equal belief we can stop
            if(equal)
            {
                foundEqualBelief=true;
                break;
            }
        }
        if(foundEqualBelief)
            addBelief=false;
        else
            addBelief=true;
    }
    else
        addBelief=true;

    if(addBelief)
    {
        _m_beliefSets[t].push_back(new JointBeliefSparse(jb));
        added=true;
    }
    else
        added=false;

    return(added);
}

string BeliefSetNonStationary::SoftPrint() const
{
    stringstream ss;
    for(Index t=0;t!=_m_beliefSets.size();++t)
    {
        ss << "Timestep " << t << ":" << endl;
        for(Index j=0;j!=_m_beliefSets[t].size();j++)
            ss << _m_beliefSets[t][j]->SoftPrint() << endl;
    }

    return(ss.str());
}
