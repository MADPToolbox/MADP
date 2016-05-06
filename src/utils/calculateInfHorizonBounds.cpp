/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include <iostream>
#include <float.h>
#include "OptimalValueDatabase.h"
#include "NullPlanner.h"
#include "argumentHandlers.h"
#include "argumentUtils.h"

using namespace std;
using namespace ArgumentUtils;

const char *argp_program_version = "calculateInfHorizonBounds";

// Program documentation
static char doc[] =
"calculateInfHorizonBounds - calculates bounds on the infinite horizon value  \
\v";

//NOTE: make sure that the below value (nrChildParsers) is correct!
const int nrChildParsers = 4;
const struct argp_child childVector[] = {
    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    ArgumentHandlers::solutionMethodOptions_child,
    ArgumentHandlers::modelOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

int main(int argc, char **argv)
{
    try {
        ArgumentHandlers::Arguments args;
        argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);

        DecPOMDPDiscreteInterface* decpomdp =
            GetDecPOMDPDiscreteInterfaceFromArgs(args);
        NullPlanner np(args.horizon,decpomdp);

        OptimalValueDatabase db(&np);
        double Vfinite=0;
        if(db.IsInDatabase())
            Vfinite=db.GetOptimalValue();
        else
        {
            cout << "Optimal finite-horizon value unknown" << endl;
            return(1);
        }


        double rMin=DBL_MAX, rMax=-DBL_MAX;
        for(Index sI=0;sI!=decpomdp->GetNrStates();++sI)
            for(Index jaI=0;jaI!=decpomdp->GetNrJointActions();++jaI)
            {
                double r=decpomdp->GetReward(sI,jaI);
                rMin=min(rMin,r);
                rMax=max(rMax,r);
            }
        
        double gamma=decpomdp->GetDiscount();
        double vMaxInf=rMax/(1-gamma);
        double vMinInf=rMin/(1-gamma);
        double discountFiniteSteps=
            (1-pow(gamma,static_cast<double>(args.horizon)))/(1-gamma);
        double vMaxFinite=rMax*discountFiniteSteps;
        double vMinFinite=rMin*discountFiniteSteps;

        if(args.verbose>=1)
            cout << "Vfinite " << Vfinite << " discountFiniteSteps "
                 << discountFiniteSteps << endl
                 << "rMax " << rMax << " rMin " << rMin << endl
                 << "vMaxInf " << vMaxInf << " vMinInf " << vMinInf << endl
                 << "vMaxFinite " << vMaxFinite << " vMinFinite " << vMinFinite 
                 << endl;
        
        // we calculate the bounds by knowing the absolute max/min
        // that we could get in this problem (vMaxInf,vMinInf),
        // compensating for the value that we actually achieved during
        // the finite horizon solution (Vfinite) - the value that we
        // maximally/minimally could have achived during these first
        // time step (vMaxFinite,vMinFinite)
        double upperBoundInfHorizon=vMaxInf+(Vfinite-vMaxFinite);
        double lowerBoundInfHorizon=vMinInf+(Vfinite-vMinFinite);

        double upperBoundInfHorizon_alt=Vfinite+pow(gamma,static_cast<double>(args.horizon))*vMaxInf;
        double lowerBoundInfHorizon_alt=Vfinite+pow(gamma,static_cast<double>(args.horizon))*vMinInf;
        
        if(!EqualReward(upperBoundInfHorizon,upperBoundInfHorizon_alt))
            throw(E("upper bound does not match"));
        if(!EqualReward(lowerBoundInfHorizon,lowerBoundInfHorizon_alt))
            throw(E("lower bound does not match"));

        cout << decpomdp->GetUnixName() << "(g=" << gamma << ", h="
             << args.horizon <<")" << setprecision(8)
             << " inf horizon bounds [ " << lowerBoundInfHorizon
             << ", " << upperBoundInfHorizon << " ]" << endl;

    }
    catch(E& e){ e.Print(); }
}
