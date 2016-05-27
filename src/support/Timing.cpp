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

#include "Timing.h"
#include <fstream>
#include <sys/times.h>
#include <limits.h>

using namespace std;

//Default constructor
Timing::Timing()
{
    _m_timeAtInitialization=GetCpuTime();
}

//Destructor
Timing::~Timing()
{
}

void Timing::Start(const string & id)
{
    Times newTimer;
    newTimer.start=GetCpuTime();
    newTimer.end=0;
    newTimer.hasEnded=false;
    _m_timesMap[id].push_back(newTimer);
}

void Timing::Stop(const string & id, clock_t endTime)
{
    map <string, vector<Times> >::iterator it=_m_timesMap.find(id);
    if(it==_m_timesMap.end())
    {
        stringstream ss;
        ss << "Timing::Stop Could not stop timing event called \""
           << id << "\", no such event ever started";
        throw(E(ss.str()));
    }
    else
    {
        it->second.size();
        it->second.back().end=endTime;
        it->second.back().hasEnded=true;
    }
}

void Timing::Stop(const string & id)
{
    Stop(id,GetCpuTime());
}

void Timing::Print() const
{
    map<string, vector<Times> >::const_iterator i;
    for(i=_m_timesMap.begin(); i!=_m_timesMap.end(); ++i)
    {
        const vector<Times> &times=i->second;
        const string &id=i->first;
        for(vector<Times>::const_iterator j=times.begin();
            j!=times.end();++j)
        {
            cout << id << ": ";
            if(j->hasEnded)
            {
                cout << j->end - j->start
                     << "   start "
                     << j->start << " end "
                     << j->end << endl;
            }
            else
                cout << "still running, started at " << j->start
                     << endl;
        }
    }
}

void Timing::PrintSummary() const
{
    map<string, vector<Times> >::const_iterator i;
    for(i=_m_timesMap.begin(); i!=_m_timesMap.end(); ++i)
    {
        const vector<Times> &times=i->second;
        const string &id=i->first;
        clock_t total=0,maxTime=0,minTime=INT_MAX,current;
        int nrSamples=0;
        for(vector<Times>::const_iterator j=times.begin();
            j!=times.end();++j)
        {
            if(j->hasEnded)
            {
                current=j->end - j->start;
                total+=current;
                if(current>maxTime)
                    maxTime=current;
                if(current<minTime)
                    minTime=current;

                nrSamples++;
            }
        }

        cout << id << ": ";
        if(nrSamples>0)            
            cout << ClockToSeconds(total)
                 << " s in " << nrSamples << " measurements"
                 << ", max " << ClockToSeconds(maxTime)
                 << ", avg " << ClockToSeconds(total)/nrSamples
                 << ", min " << ClockToSeconds(minTime)
                 << endl;
        else if((times.size()-nrSamples)>0)     // this will only
                                                // print the correct
                                                // "still running"
                                                // time if called from
                                                // the same process
            cout << (times.size()-nrSamples) << " still running, spent "
                 << (ClockToSeconds(GetCpuTime()) - 
                     ClockToSeconds(i->second[0].start)) << endl;
    }
}

void Timing::Save(const string & filename) const
{
    ofstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "Timing::Save: failed to open file " << filename;
        throw(E(ss.str()));
    }

    Save(fp);
}

void Timing::Save(ofstream &of) const
{
    of << _m_timeAtInitialization << endl;

    map<string, vector<Times> >::const_iterator i;
    for(i=_m_timesMap.begin(); i!=_m_timesMap.end(); ++i)
    {
        const vector<Times> &times=i->second;
        const string &id=i->first;

        for(vector<Times>::const_iterator j=times.begin();
            j!=times.end();++j)
        {
            if(j->hasEnded)
                of << id << " " << j->start << " " << j->end << endl;
        }
    }
}

void Timing::Load(const string & filename)
{
    bool first=true;

    ifstream fp(filename.c_str());
    if(!fp)
    {
        stringstream ss;
        ss << "Timing::Load: failed to open file " << filename;
        throw(E(ss));
    }

    _m_timesMap.clear();

    string buffer;
    while(!getline(fp,buffer).eof())
    {
        {
            string identification;
            clock_t start,end;
            Times times;

            istringstream is(buffer);
            if(first)
            {
                first=false;
                is >> start;
                _m_timeAtInitialization=start;
            }
            else
            {
                is >> identification;
                is >> start;
                is >> end;
                times.start=start;
                times.end=end;
                times.hasEnded=true;
                _m_timesMap[identification].push_back(times);
            }
        }
    }
}

clock_t Timing::GetCpuTime() const
{
    struct tms timeStruct;
    times(&timeStruct);
    return(timeStruct.tms_utime + timeStruct.tms_stime);
}

double Timing::ClockToSeconds(clock_t clockTicks) const
{
    return(static_cast<double>(clockTicks) / sysconf(_SC_CLK_TCK));
}

void Timing::AddEvent(const string & id, clock_t duration)
{
    Start(id);
    Stop(id,GetCpuTime()+duration);
}

vector<double> Timing::GetEventDurations(const string & id) const
{
    vector<double> durations;

    map <string, vector<Times> >::const_iterator it=_m_timesMap.find(id);
    if(it==_m_timesMap.end())
    {
        stringstream ss;
        ss << "Timing::GetEventDurations No events called \""
           << id << "\" have been stored";
        throw(E(ss.str()));
    }
    else
    {
        const vector<Times> &times=it->second;
        for(vector<Times>::const_iterator j=times.begin();
            j!=times.end();++j)
        {
            if(j->hasEnded)
                durations.push_back(ClockToSeconds(j->end - j->start));
        }
    }
    return(durations);
}

vector<double> Timing::GetRunningEventDurations(const std::string & id) const
{
    vector<double> durations;

    map <string, vector<Times> >::const_iterator it=_m_timesMap.find(id);
    if(it==_m_timesMap.end())
    {
        stringstream ss;
        ss << "Timing::GetRunningEventDurations No events called \""
           << id << "\" have been stored";
        throw(E(ss.str()));
    }
    else
    {
        const vector<Times> &times=it->second;
        for(vector<Times>::const_iterator j=times.begin();
            j!=times.end();++j)
        {
            // here we are only interested in events that are still running
            if(!j->hasEnded)
                durations.push_back(ClockToSeconds(GetCpuTime() - j->start));
        }
    }
    return(durations);
}
