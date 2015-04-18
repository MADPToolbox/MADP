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
