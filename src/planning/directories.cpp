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

#include "directories.h"
#include <wordexp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>
#include "PlanningUnit.h"
#include "MultiAgentDecisionProcessInterface.h"
#include "argumentHandlers.h"

using namespace std;

string directories::MADPGetResultsDir()
{
    wordexp_t p;
    wordexp("~/.madp/results",&p,0);
    string dir(p.we_wordv[0]);
    wordfree(&p);
    return(dir);
}

string directories::MADPGetResultsDir(const string & method,
                                      const string & problem)
{
    string dir=MADPGetResultsDir() + "/" + method + "/" + problem;
    return(dir);
}

string directories::MADPGetResultsDir(const string & method,
                                      const PlanningUnit &pu)
{
    return(MADPGetResultsDir(method, pu.GetProblem()->GetUnixName()));
}

string
directories::MADPGetResultsDir(const string & method,
                               const MultiAgentDecisionProcessInterface
                               &problem)
{
    return(MADPGetResultsDir(method, problem.GetUnixName()));
}

string directories::MADPGetResultsDir(const string & method,
                                      const PlanningUnit *pu)
{
    return(MADPGetResultsDir(method,pu->GetProblem()->GetUnixName()));
}

string
directories::MADPGetResultsDir(const string & method,
                               const MultiAgentDecisionProcessInterface
                               *problem)
{
    return(MADPGetResultsDir(method,problem->GetUnixName()));
}

void directories::MADPCreateResultsDir(const string & method,
                                       const string & problem)
{
    string dir=MADPGetResultsDir() + "/" + method + "/" + problem;
    
    struct stat statInfo;
    if(stat(dir.c_str(),&statInfo)==0 &&
       S_ISDIR(statInfo.st_mode))
    {
#if 0
        cout << "Results dir " << dir << " already exists" << endl;
#endif
    }
    else
        if(mkdir(dir.c_str(),0777)!=0)
        {
            stringstream ss;
            ss << "mkdir error for " << dir;
            throw(E(ss));
        }
}

void directories::MADPCreateResultsDir(const string & method,
                                      const PlanningUnit &pu)
{
    return(MADPCreateResultsDir(method,pu.GetProblem()->GetUnixName()));
}

void
directories::MADPCreateResultsDir(const string & method,
                                  const MultiAgentDecisionProcessInterface
                                  &problem)
{
    return(MADPCreateResultsDir(method,problem.GetUnixName()));
}

void directories::MADPCreateResultsDir(const string & method,
                                      const PlanningUnit *pu)
{
    return(MADPCreateResultsDir(method,pu->GetProblem()->GetUnixName()));
}

void
directories::MADPCreateResultsDir(const string & method,
                                  const MultiAgentDecisionProcessInterface
                                  *problem)
{
    return(MADPCreateResultsDir(method,problem->GetUnixName()));
}

string
directories::MADPGetResultsBaseFilename(const string & method, const string & problem, 
                                        const ArgumentHandlers::Arguments &args)
{
    string descr;
    if(args.description == NULL)
        descr = problem;
    else
        descr = string(args.description);

    string basename = method + "_" + descr + "_";
    if(args.prefix != NULL )
        basename = basename + args.prefix + "_";

    if(args.discount!=-1)
    {
        stringstream ss;
        ss << basename << "g" << args.discount << "_";
        basename=ss.str();
    }
    return(basename);
}

string directories::MADPGetResultsFilename(const string & method,
                                           const string & problem,
                                           const
                                           ArgumentHandlers::Arguments &args)
{
    return(MADPGetResultsDir(method,problem) + "/" +
           MADPGetResultsBaseFilename(method, problem, args));
}

string directories::MADPGetResultsFilename(const string & method,
                                           const PlanningUnit &pu,
                                           const
                                           ArgumentHandlers::Arguments &args)
{
    return(MADPGetResultsDir(method,pu) + "/" +
           MADPGetResultsBaseFilename(method,pu.GetProblem()->GetUnixName(),
                                      args));
}

string
directories::MADPGetResultsFilename(const string & method,
                                    const 
                                    MultiAgentDecisionProcessInterface &problem,
                                    const ArgumentHandlers::Arguments &args)
{
    return(MADPGetResultsDir(method,problem) + "/" +
           MADPGetResultsBaseFilename(method,problem.GetUnixName(),
                                      args));
}

string directories::MADPGetProblemsDir()
{
    wordexp_t p;
    wordexp("~/.madp/problems",&p,0);
    string dir(p.we_wordv[0]);
    wordfree(&p);
    return(dir);
}

string directories::MADPGetProblemFilename(const string & problem,
                                           const string & extension)
{
    // check whether the problem string ends in the extension
    if(extension.size() < problem.size() && // otherwise the compare
                                            // cannot be done
       problem.compare(problem.size()-extension.size(),
                       extension.size(),
                       extension)==0)
        return(problem);
    else
        return(MADPGetProblemsDir() + "/" + problem + "." + extension);
}

string directories::MADPGetProblemFilename(const string & problem)
{
    if (problem.empty())
        throw(E("No problem file specified."));

    size_t pos = problem.find_last_of(".");

    if(pos < problem.length()-1 && pos > 0){ //non-empty extension
        return(problem);
    }else{
        DIR *d = opendir (MADPGetProblemsDir().c_str());
	if (d == NULL)
	    throw(E("Could not open problems directory. Please create a symbolic link from ~/.madp/problems to the problems directory in your MADP tree (see README for details)."));

	string problemName;
        struct dirent *ep;
        vector<string> extensions; //default extensions. Add more here if needed.
        extensions.push_back("pgmx"); 
        extensions.push_back("dpomdp");
        extensions.push_back("POMDP");
        problemName = problem;
        while(extensions.size() > 0)
        {
            string s = problemName + "." + extensions.back();
            while ((ep = readdir (d)))
            {
                if(!s.compare(ep->d_name))
	        {
                    closedir (d);
                    return(MADPGetProblemsDir() + "/" + s);
	        }
            }
            rewinddir (d);
            extensions.pop_back();
        }
	closedir (d);
    }

    stringstream ss;
    ss << "Problem file not found (problem = " << problem << ")";
    throw(E(ss));
    return(problem);
}

string directories::MADPGetProblemFilename(const 
                                           ArgumentHandlers::Arguments &args)
{
    if(args.isTOI)
        return(MADPGetProblemFilename(args.dpf,"toi-dpomdp"));
    else
        return(MADPGetProblemFilename(args.dpf));
}
