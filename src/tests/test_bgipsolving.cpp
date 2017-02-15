/**\file test_bgipsolving.cpp
 *
 * Authors:
 * Frans Oliehoek <faolieho@science.uva.nl>
 * Matthijs Spaan <mtjspaan@isr.ist.utl.pt>
 *
 * Copyright 2008 Universiteit van Amsterdam, Instituto Superior Tecnico
 *
 * This file is part of MultiAgentDecisionProcess.
 *
 * MultiAgentDecisionProcess is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * MultiAgentDecisionProcess is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MultiAgentDecisionProcess.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * $Id$
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
