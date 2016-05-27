/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "FG_SolverNDP.h"
#include "ndp.h"
#include "EDeadline.h"

using namespace std;

//Default constructor
FG_SolverNDP::FG_SolverNDP(
            const libDAI::FactorGraph& f, 
            //ndp parameters: 
            int verbosity,
            size_t nrSolutions,
            double deadlineInSeconds
            )  
    :
    FG_Solver(f),
    //NDPSolver(maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts)
    _m_verbosity(verbosity),
    _m_nrSolutions(nrSolutions),
    _m_deadlineInSeconds(deadlineInSeconds)
{
}
/*
//Copy constructor.    
FG_SolverNDP::FG_SolverNDP(const FG_SolverNDP& o) 
{
}
//Destructor
FG_SolverNDP::~FG_SolverNDP()
{
}
//Copy assignment operator
FG_SolverNDP& FG_SolverNDP::operator= (const FG_SolverNDP& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/


double FG_SolverNDP::Solve()
{
    libDAI::Properties props;
    //props.Set("maxiter",_m_maxiter);
    //cout <<"FG_SolverNDP::Solve() - _m_verbosity="<<_m_verbosity<<endl;
    size_t verb = (size_t) std::max( (_m_verbosity-3), 0 );
    props.Set("verbose", verb);
    //props.Set("updates",_m_updateType);
    //props.Set("damping",_m_damping);
    //double  tol = 1e-2;
    //props.Set("tol",tol);    
    libDAI::NDP ndp (*_m_fg, props, _m_nrSolutions);
    //for(Index restI=0; restI < _m_nrRestarts; restI++)
    //{
    ndp.init();
    ndp.setDeadline(_m_deadlineInSeconds);
    double value_rep=-DBL_MAX;
    try {
        value_rep = ndp.run();
    } catch (libDAI::Exception &e)
    {
#if 0 // our version of libDAI doesn't have this Code() function yet, so we need to hack around that
        if(e.Code()==libDAI::INTERNAL_ERROR)
#else
        if(string(e.what()).find("Internal error")!=string::npos)
#endif
            throw(EDeadline("NDP deadline exceeded"));

        else
            throw(e);
    }
    //if(_m_verbosity >= 2)
            //cout << "Found solution with value="<<value_rep<<endl;
    //}

    //save the best configurations found by max-plus...
    //we make a copy, when ndp dies, its configs go with it...
    _m_bestConfs = ndp.GetBestConfigurations();



    size_t nrConfsFound = _m_bestConfs.size();
    if(_m_verbosity >= 1)
        cout << "found "<< nrConfsFound <<" configurations"<<endl;

    double value = 0.0;
    if(nrConfsFound > 0)
    {
        libDAI::MADP_util::valConf& vc = _m_bestConfs.front();
        value = vc.first;
        std::vector<size_t>& sol_vec = vc.second;
        if(_m_verbosity >= 2)
            cout << "best value:"<<value<<endl;
        if(_m_verbosity >= 3)
            cout << "best configuration:\n"<<
                PrintTools::SoftPrintVector(sol_vec)<<endl;
        return (value);
    }
    else
        return(0.0);

}
