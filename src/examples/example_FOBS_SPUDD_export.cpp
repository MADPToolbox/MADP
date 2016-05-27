/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Philipp Robbel 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "ProblemFOBSFireFightingGraph.h"
#include <iostream>

using namespace std;

// This is an example how a fully-observable problem can be exported to the SPUDD file format.
int main() {
    try {
        ProblemFOBSFireFightingGraph mmdp(10,3);
        
        // Debug prints
        cout << "#agents: " << mmdp.GetNrAgents() << endl;
        cout << "#LRFs: " << mmdp.GetNrLRFs() << endl;

        cout << "observation variables information (space equiv. to |S| for fully obs. problems):" << endl;
        const vector<size_t>& obs = mmdp.GetNrObservations();
        for(size_t i=0;i<obs.size();i++)
            cout << i << ":" << " " << obs[i] << " ";
        // Note: for fully-observable problems, JOI is likely to overflow for
        // realistic sizes of state space |S|
        cout << endl << "JOI valid: " << mmdp.JointOIndicesValid() << endl;
        
        cout << "Writing to .spudd file" << endl;
        mmdp.ExportSpuddFile("main.spudd");

    } catch(E& e){ e.Print(); }
    
    return 0;
}
