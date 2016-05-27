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

#include "TimedAlgorithm.h"
#include <fstream>

using namespace std;

//Default constructor
TimedAlgorithm::TimedAlgorithm()
{
    _m_timer=new Timing();
}
//Destructor
TimedAlgorithm::~TimedAlgorithm()
{
    delete _m_timer;
}

void TimedAlgorithm::StartTimer(const string & id) const
{
    _m_timer->Start(id);
}

void TimedAlgorithm::StopTimer(const string & id) const
{
    _m_timer->Stop(id);
}

void TimedAlgorithm::PrintTimers() const
{
    _m_timer->Print();
}

void TimedAlgorithm::SaveTimers(const string & filename) const
{
    _m_timer->Save(filename);
}

void TimedAlgorithm::SaveTimers(ofstream &of) const
{
    _m_timer->Save(of);
}

void TimedAlgorithm::LoadTimers(const string & filename)
{
    _m_timer->Load(filename);
}

void TimedAlgorithm::PrintTimersSummary() const
{ 
    _m_timer->PrintSummary();
}

void TimedAlgorithm::AddTimedEvent(const string & id, clock_t duration)
{
    _m_timer->AddEvent(id,duration);
}

vector<double> TimedAlgorithm::GetTimedEventDurations(const string & id)
{
    return(_m_timer->GetEventDurations(id));
}
