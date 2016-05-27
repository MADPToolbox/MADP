/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include "SimulationResult.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Use as follows: analyzeRewardResults "
             << "<resultsFile>" << endl;
        return(1);
    }

    string rewardsFile=string(argv[1]);

    try {
        SimulationResult result;
        result.Load(rewardsFile);
        result.PrintSummary();
    }
    catch(E& e){ e.Print(); }
}
