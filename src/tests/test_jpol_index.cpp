#include "Globals.h"
#include "JointPolicyPureVector.h"
#include "BayesianGameIdenticalPayoff.h"

using namespace std;

void testJpolIndex(const BayesianGameIdenticalPayoff& bgip)
{
    JointPolicyPureVector jp(&bgip);
    LIndex nrJpol=bgip.GetNrJointPolicies();
    for(Index i=0;i!=nrJpol;++i)
    {
        if(jp.GetIndex()!=i)
        {
            cerr << "ERROR: Following joint policy should have index " << i
                 << jp.SoftPrint() << endl;
            exit(1);
        }
        ++jp;
    }

    for(Index i=0;i!=nrJpol;++i)
    {
        jp.SetIndex(i);
        if(jp.GetIndex()!=i)
        {
            cerr << "ERROR: Following joint policy should have index " << i
                 << jp.SoftPrint() << endl;
            exit(1);
        }
    }

}

int main()
{
    {
        size_t nrAgents = 2;
        Index acs_ar[] = {2,2};
        vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
        Index obs_ar[] = {1,2};
        vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
        
        BayesianGameIdenticalPayoff bgip = 
            BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
        
        testJpolIndex(bgip);
    }

    {
        size_t nrAgents = 3;
        Index acs_ar[] = {2,3,1};
        vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
        Index obs_ar[] = {1,2,3};
        vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
        
        BayesianGameIdenticalPayoff bgip = 
            BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
        
        testJpolIndex(bgip);
    }

    {
        size_t nrAgents = 4;
        Index acs_ar[] = {2,2,3,2};
        vector<size_t> acs (acs_ar, acs_ar + sizeof(acs_ar) / sizeof(Index));
        Index obs_ar[] = {2,2,2,3};
        vector<size_t> obs (obs_ar, obs_ar + sizeof(obs_ar) / sizeof(Index));
        
        BayesianGameIdenticalPayoff bgip = 
            BayesianGameIdenticalPayoff::GenerateRandomBG(nrAgents, acs, obs);
        
        testJpolIndex(bgip);
    }
}
