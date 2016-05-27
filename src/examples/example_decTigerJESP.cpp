/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
