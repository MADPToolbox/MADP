/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include "Timing.h"
#include "directories.h"
#include "DecPOMDPDiscrete.h"
#include "BGBackupType.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Use as follows: analyzeTimings "
             << "<timingsFile>" << endl;
        return(1);
    }

    string timingsFile=string(argv[1]);

    try {
    Timing time;
    time.Load(timingsFile);
    time.PrintSummary();
    }
    catch(E& e){
        e.Print();
        return(1);
    }

    return(0);
}
