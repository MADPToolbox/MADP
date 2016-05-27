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
    if(argc!=5)
    {
        cout << "Use as follows: policyPureVectorToDotGraph "
             << "<problem> <horizon> <agentIndex> <policyIndex>" << endl;
        return(1);
    }

    // parse arguments
    string dpomdpFile=directories::MADPGetProblemFilename(argv[1]);
    int h=strtol(argv[2],0,10);
#if USE_ARBITRARY_PRECISION_INDEX
    LIndex agentIndex=*argv[3];
    LIndex policyIndex=*argv[4];
#else
    LIndex agentIndex=strtoull(argv[3],0,10);
    LIndex policyIndex=strtoull(argv[4],0,10);
#endif
    try {

    DecPOMDPDiscrete decpomdp("","",dpomdpFile);
    MADPParser parser(&decpomdp);
    PlanningUnitMADPDiscreteParameters params;
    params.SetComputeAll(true);
    params.SetComputeJointBeliefs(false);
    NullPlanner np(params,h,&decpomdp);
    PolicyPureVector p(&np,0, np.GetDefaultIndexDomCat());
    p.SetIndex(policyIndex);

    stringstream ss;
    ss << decpomdp.GetUnixName() << "_h"
       << h << "_agent" << agentIndex
       << "_policy" << policyIndex
       << ".dot";
    np.ExportDotGraph(ss.str(),p,CastLIndexToIndex(agentIndex));

    cout << "Exported policy to " << ss.str() << endl;

    }
    catch(E& e){ e.Print(); }

    return(0);
}
