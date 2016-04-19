/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

// Very simple programs that only uses the Parser and Base libraries.

#include <iostream>
#include "DecPOMDPDiscrete.h"
#include "MADPParser.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Use as follows: printProblem "
             << "<problem>" << endl;
        return(1);
    }

    try {

        DecPOMDPDiscrete dpomdp("","",argv[1]);
        MADPParser parser(&dpomdp);
        dpomdp.Print();

    }
    catch(E& e){ e.Print(); }

    return(0);
}
