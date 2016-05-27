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

#include "FG_SolverMaxPlus.h"
#include "maxplus.h"
#include "EDeadline.h"

using namespace std;

//Default constructor
FG_SolverMaxPlus::FG_SolverMaxPlus(const libDAI::FactorGraph& f, 
                                   //maxplus parameters: 
                                   size_t maxiter,
                                   std::string updateType,
                                   int verbosity,
                                   double damping,
                                   size_t nrSolutions,
                                   size_t nrRestarts,
                                   double deadlineInSeconds)  
    :
        FG_Solver(f),
        MaxPlusSolver(maxiter, updateType, verbosity, damping, nrSolutions, nrRestarts),
        _m_deadlineInSeconds(deadlineInSeconds)
{
}
/*
//Copy constructor.    
FG_SolverMaxPlus::FG_SolverMaxPlus(const FG_SolverMaxPlus& o) 
{
}
//Destructor
FG_SolverMaxPlus::~FG_SolverMaxPlus()
{
}
//Copy assignment operator
FG_SolverMaxPlus& FG_SolverMaxPlus::operator= (const FG_SolverMaxPlus& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}
*/


double FG_SolverMaxPlus::Solve()
{
    tms ts_before, ts_after;
    times(&ts_before);

    libDAI::Properties props;
    props.Set("maxiter",_m_maxiter);
    //cout <<"FG_SolverMaxPlus::Solve() - _m_verbosity="<<_m_verbosity<<endl;
    size_t verb = (size_t) std::max( (_m_verbosity-3), 0 );
    props.Set("verbose", verb);
    props.Set("updates",_m_updateType);
    props.Set("damping",_m_damping);
    double  tol = 1e-2;
    props.Set("tol",tol);    
    libDAI::MaxPlus mp (*_m_fg, props, _m_nrSolutions);
    for(Index restI=0; restI < _m_nrRestarts; restI++)
    {
        if(_m_verbosity >= 4)
            cout << "New MaxPlus restart...";

        mp.init();
        /* GetWriteAnyTimeResults, GetResultsOFStream, GetTimingsOFStream
           are defined by BayesianGameIdenticalPayoffSolver_T< JointPolicyPureVector >
           we do not really need this (at this point at least) so let's not bother for now
           ----------
           if(GetWriteAnyTimeResults())
            mp.SetAnyTimeResults(true,
                                 GetResultsOFStream(),
                                 GetTimingsOFStream());*/
        double value = mp.run();

        if(_m_deadlineInSeconds>0)
        {
            times(&ts_after);
            clock_t utime = ts_after.tms_utime -
                ts_before.tms_utime;
            double timeSpentInS=static_cast<double>(utime) / sysconf(_SC_CLK_TCK);
            if(timeSpentInS>_m_deadlineInSeconds)
                throw(EDeadline("MaxPlus deadline exceeded"));
        }
        if(_m_verbosity >= 4)
            cout << "Found solution with value="<<value<<endl;
    }

    //save the best configurations found by max-plus...
    //const std::vector<size_t> & config = mp.GetBestConfiguration();
    //we make a copy, when mp dies, its configs go with it...
    _m_bestConfs = mp.GetBestConfigurations();



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
