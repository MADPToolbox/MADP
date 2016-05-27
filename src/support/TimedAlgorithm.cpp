/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
