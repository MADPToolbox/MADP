/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "POMDPDiscrete.h"

using namespace std;

//Default constructor
POMDPDiscrete::POMDPDiscrete(const std::string &name,
                             const std::string &descr,
                             const std::string &pf) :
    DecPOMDPDiscrete(name,descr,pf)
{
    this->SetNrAgents(1);
}
//Copy constructor.    
// POMDPDiscrete::POMDPDiscrete(const POMDPDiscrete& o) 
// {
// }
//Destructor
//POMDPDiscrete::~POMDPDiscrete()
//{
//}
//Copy assignment operator
// POMDPDiscrete& POMDPDiscrete::operator= (const POMDPDiscrete& o)
// {
//     if (this == &o) return *this;   // Gracefully handle self assignment
//     // Put the normal assignment duties here...

//     return *this;
// }
