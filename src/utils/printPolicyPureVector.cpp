/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
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
