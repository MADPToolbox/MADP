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
 * Bas Terwijn
 *
 * For contact information please see the included AUTHORS file.
 */


#include <float.h>
#include "BayesianGameIdenticalPayoff.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Use as follows: printBayesianGameIdenticalPayoff "
             << "<filename>" << endl;
        return(1);
    }

    // parse arguments
    string filename(argv[1]);

    try {

        BayesianGameIdenticalPayoff bg(BayesianGameIdenticalPayoff::Load(filename));
        bg.Print();

        size_t nrJTypes=bg.GetNrJointTypes();
        vector<double> johProbs(nrJTypes);
        vector<double> maxUtils(nrJTypes,-DBL_MAX);
        vector<double> minUtils(nrJTypes,DBL_MAX);
        vector<double> maxError(nrJTypes);
        vector<bool> useJOH(nrJTypes);
        double util, threshold=0.01;
        for(Index jt = 0; jt < nrJTypes; jt++)
        {
            johProbs[jt]=bg.GetProbability(jt);
            for(Index jaI=0; jaI < bg.GetNrJointActions(); ++jaI)
            {
                util=bg.GetUtility(jt, jaI);
                maxUtils[jt]=max(maxUtils[jt],util);
                minUtils[jt]=min(minUtils[jt],util);
            }
            maxError[jt]=johProbs[jt]*max(abs(maxUtils[jt]),abs(minUtils[jt]));
            if(maxError[jt]>threshold)
                useJOH[jt]=true;
            else
                useJOH[jt]=false;
        }
        PrintVectorCout(johProbs);cout << endl;
        PrintVectorCout(maxUtils);cout << endl;
        PrintVectorCout(minUtils);cout << endl;
        PrintVectorCout(maxError);cout << endl;
        PrintVectorCout(useJOH);cout << endl;

    }
    catch(E& e){ e.Print(); }

    return(0);
}
