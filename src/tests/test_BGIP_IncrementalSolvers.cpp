/**\file test_BGIP_IncrementalSolvers.cpp
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

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <string>

#include "Globals.h"
#include "PolicyGlobals.h"
#include "ProblemDecTiger.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoff.h"
#include "BGIP_SolverBruteForceSearch.h"
#include "BGIP_SolverBranchAndBound.h"
#include "Timing.h"

#define BEEP 1

#include "argumentHandlers.h"

const char *argp_program_version = "test_BGIP_IncrementalSolvers 0.1";

// Program documentation
static char doc[] =
"This program generates a random BG and solves it testing the incremental solvers. \
\v";

const struct argp_child childVector[] = {
//    ArgumentHandlers::problemFile_child,
    ArgumentHandlers::globalOptions_child,
    { 0 }
};

#include "argumentHandlersPostChild.h"

using namespace std;
using namespace BGIP_BnB;

void testBGIP_Solvers();
void testSolver(const BayesianGameIdenticalPayoff& bgip);

//the structure in whiuch the options are put
ArgumentHandlers::Arguments args;

int main(int argc, char **argv)
{

    argp_parse (&ArgumentHandlers::theArgpStruc, argc, argv, 0, 0, &args);
    try
    {
        testBGIP_Solvers();
    }catch(E& e)
    {
        e.Print();
    }
    return(0);
}//end of main


void testBGIP_Solvers()
{
    srand(time(NULL));

    for(Index k=0;k!=100;++k)
    {
        {
            size_t nrAgents = 2;
            Index acs_ar[] = {2,2};
            vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
            Index obs_ar[] = {2,2};
            vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
            
            BayesianGameIdenticalPayoff bgip = 
                BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
            
            testSolver(bgip);
        }

        {
            size_t nrAgents = 2;
            Index acs_ar[] = {2,2};
            vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
            Index obs_ar[] = {1,2};
            vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
            
            BayesianGameIdenticalPayoff bgip = 
                BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
            
            testSolver(bgip);
        }
        
        {
            size_t nrAgents = 3;
            Index acs_ar[] = {2,3,2};
            vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
            Index obs_ar[] = {3,2,2};
            vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
            
            BayesianGameIdenticalPayoff bgip = 
                BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
            
            testSolver(bgip);
        }

        {
            size_t nrAgents = 4;
            Index acs_ar[] = {3,1,2,3};
            vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
            Index obs_ar[] = {3,2,1,2};
            vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
            
            BayesianGameIdenticalPayoff bgip = 
                BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
            
            testSolver(bgip);
        }

        {
            size_t nrAgents = 4;
            Index acs_ar[] = {2,2,2,2};
            vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
            Index obs_ar[] = {2,2,2,2};
            vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
            
            BayesianGameIdenticalPayoff bgip = 
                BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
            
            testSolver(bgip);
        }

    }
}

void testSolver(const BayesianGameIdenticalPayoff &bgip1)
{
    BGIP_sharedPtr bgip=BGIP_sharedPtr(new BayesianGameIdenticalPayoff(bgip1));
    BGIP_SolverBruteForceSearch<JointPolicyPureVector> bfs(bgip);

    vector<double> bfsValues;
    vector<LIndex> bfsJPols;
    while(!bfs.AllSolutionsHaveBeenReturned())
    {
        double value;
        boost::shared_ptr<JointPolicyDiscretePure> jp;
        if (!bfs.GetNextJointPolicyAndValue(jp,value))
            break; // not more found because of _m_CBGlowerBound in BGIP_SolverBruteForceSearch::GetNextJointPolicyAndValueSpecific()
        boost::shared_ptr<JointPolicyPureVector> jppv = boost::dynamic_pointer_cast<JointPolicyPureVector>(jp);
        bfsValues.push_back(value);
        bfsJPols.push_back(jppv->GetIndex());
    }
    if(bfsValues.size()!=bgip->GetNrJointPolicies())
    {
        cerr << "BFS only returned " << bfsValues.size() << " solutions, should be "
             << bgip->GetNrJointPolicies() << endl;
        exit(1);
    } 
    cout << "BFS jpols: " << endl << SoftPrintVector(bfsJPols) << endl;
    cout << "BFS values: " << endl << SoftPrintVector(bfsValues) << endl;

    for(Index i=0;i!=NUMBER_OF_BNB_JOINT_TYPE_ORDERINGS;++i)
//    for(Index i=0;i!=1;++i)
    {
        vector<double> bnbValues;
        vector<LIndex> bnbJPols;
        stringstream ss;
        BnB_JointTypeOrdering jto=static_cast<BnB_JointTypeOrdering>(i);
        ss << "BGIP_SolverBranchAndBound_" << SoftPrint(jto);

        try { // catch #ERROR: ReOrderJointTypes: agents have different number of types, fix implementation"
            BGIP_SolverBranchAndBound<JointPolicyPureVector> bnb(bgip,args.verbose,1,false,jto);

            while(!bnb.AllSolutionsHaveBeenReturned())
            {
                double value;
                boost::shared_ptr<JointPolicyDiscretePure> jpdp;
                if (!bnb.GetNextJointPolicyAndValue(jpdp,value))
                    break;
                boost::shared_ptr<JointPolicyPureVector> jp = boost::dynamic_pointer_cast<JointPolicyPureVector>(jpdp);
                bnbValues.push_back(value);
                bnbJPols.push_back(jp->GetIndex());
            }
        } catch(E& e)
        {
            continue;
        }
        if(bnbValues.size()!=bgip->GetNrJointPolicies())
        {
            cerr << ss.str() << " only returned " << bfsValues.size()
                 << " solutions, should be "
                 << bgip->GetNrJointPolicies() << endl;
            exit(1);
        }
        for(Index j=0;j!=bfsValues.size();++j)
        {
            if(!EqualProbability(bfsValues[j], bnbValues[j]))
            {
                cerr << "ERROR: The value of the "
                     << j << "th solution of " << ss.str()
                     << " does not match with BFS (" << bnbValues[j] 
                     << " vs " << bfsValues[j] << ")" << endl;
//                cerr << SoftPrintVector(bnbValues) << endl;
                exit(1);
            }
            if(bfsJPols[j]!=bnbJPols[j])
            {
                cerr << "ERROR: The jpol of the "
                     << j << "th solution of " << ss.str()
                     << " does not match with BFS (" << bnbJPols[j] 
                     << " vs " << bfsJPols[j] << ")" << endl;
                exit(1);
            }
        }
            
        cout << ss.str() << " jpols:" << endl << SoftPrintVector(bnbJPols) << endl;
        cout << ss.str() << " values:" << endl << SoftPrintVector(bnbValues) << endl;
    }

    cout << "no discrepancy between BFS and BNB detected" << endl;
}
