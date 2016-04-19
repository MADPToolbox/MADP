/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "BGCG_SolverRandom.h"

#include <float.h>
#include <iostream>
#include <fstream>
#include <boost/make_shared.hpp>

#include "BayesianGameCollaborativeGraphical.h"

using namespace std;

//Default constructor
BGCG_SolverRandom::BGCG_SolverRandom(
    const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bgcg,
    size_t nrSolutions):
        BGCG_Solver(bgcg,  nrSolutions)
{
}

double BGCG_SolverRandom::Solve()
{
    double value = 0.0;
    for(Index i=0;i!=GetNrDesiredSolutions();++i)
    {
        //translate found configuration to jpol
        JPPV_sharedPtr temp = boost::dynamic_pointer_cast<JointPolicyPureVector>( GetNewJpol() );
        JPPV_sharedPtr jpolBG=boost::make_shared<JointPolicyPureVector>(*temp);
//        delete temp;
        jpolBG->RandomInitialization();

        //store the solution
        AddSolution(*jpolBG,value);
    }

    return(value);
}
