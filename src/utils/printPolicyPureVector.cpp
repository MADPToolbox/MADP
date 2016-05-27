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


#include <iostream>
#include "DecPOMDPDiscrete.h"
#include "parser/MADPParser.h"
#include "PolicyPureVector.h"
#include "NullPlanner.h"
#include "directories.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc!=4)
    {
        cout << "Use as follows: printPolicyPureVector "
             << "<problem> <horizon> <index>" << endl;
        return(1);
    }

    // parse arguments
    string dpomdpFile=directories::MADPGetProblemFilename(argv[1]);
    int h=strtol(argv[2],0,10);
#if USE_ARBITRARY_PRECISION_INDEX
    LIndex index=*argv[3];
#else
    LIndex index=strtoull(argv[3],0,10);
#endif
    try {

    DecPOMDPDiscrete decpomdp("","",dpomdpFile);
    MADPParser parser(&decpomdp);
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointBeliefs(false);
    NullPlanner np(params,h,&decpomdp);
    PolicyPureVector p(&np,0, PolicyGlobals::TYPE_INDEX);
    p.SetIndex(index);

    p.Print();

    }
    catch(E& e){ e.Print(); }

    return(0);
}
