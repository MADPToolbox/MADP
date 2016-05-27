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

// Very short example program to demonstrate how to solve the DecTiger problem
#include "ProblemDecTiger.h"
#include "JESPDynamicProgrammingPlanner.h"

using namespace std;

int main()
{
    // construct problem
    ProblemDecTiger dectiger;
    // initialize planner for horizon 3
    JESPDynamicProgrammingPlanner jesp(3,&dectiger);
    // run the planning algorithm
    jesp.Plan();

    cout << "Value computed for DecTiger horizon 3: "
         << jesp.GetExpectedReward() << endl;
    cout << "Policy computed:" << endl;
    cout << jesp.GetJointPolicy()->SoftPrint() << endl;

    return(0);
}
