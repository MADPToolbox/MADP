/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include "AlphaVectorPlanning.h"
#include "NullPlanner.h"
#include "ProblemDecTiger.h"
#include "MonahanBGPlanner.h"

using namespace std;

int main(int argc, char **argv)
{
    if(argc!=2)
    {
        cout << "Use as follows: " << argv[0] << "  "
             << "<QFunctionFile>" << endl;
        return(1);
    }

    string qFunctionFile=string(argv[1]);

    try {
        ValueFunctionPOMDPDiscrete V=AlphaVectorPlanning::ImportValueFunction(qFunctionFile);
        // guess number of actions and figure out number of states
        size_t nrStates=V.at(0).GetNrValues();
        size_t nrActions=0;
        for(Index k=0;k!=V.size();++k)
            if(V.at(k).GetAction()>nrActions)
                nrActions=V.at(k).GetAction();
        nrActions++; // the number is 1 more than the highest index
        QFunctionsDiscrete Q=AlphaVectorPlanning::ValueFunctionToQ(V,nrActions,nrStates);
        cout << "Original Q function contains";
        for(Index a=0;a!=nrActions;a++)
            cout << " " << Q.at(a).size();
        cout << " vectors (total " << V.size() << ")" << endl;
        

        // we need an AlphaVectorPlanning object for Pruning, so make
        // a ProblemDecTiger with a NullPlanner
        ProblemDecTiger *pdt=new ProblemDecTiger();
        pdt->SetNrStates(nrStates);
        NullPlanner *np=new NullPlanner(pdt);
        MonahanBGPlanner p(np);
        p.Initialize();

        QFunctionsDiscrete Q1=p.Prune(Q);

        cout << "Parsimonious Q function contains";
        size_t nrVec=0;
        for(Index a=0;a!=nrActions;a++)
        {
            nrVec+=Q1.at(a).size();
            cout << " " << Q1.at(a).size();
        }
        cout << " vectors (total " << nrVec << ")" << endl;

        string outFile=qFunctionFile + string("_parsimonious");
        AlphaVectorPlanning::ExportValueFunction(outFile, Q1);
        cout << "Stored resulting Q function to "
             << outFile << endl;
    }
    catch(E& e){ e.Print(); }
}
