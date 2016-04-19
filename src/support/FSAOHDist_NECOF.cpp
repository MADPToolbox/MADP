/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

#include "FSAOHDist_NECOF.h"
#include "FactoredStateDistribution.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"
#include "Scope.h"
#include "IndexTools.h"
#include "VectorTools.h"
#include "CPDDiscreteInterface.h"
#include "CPT.h"
#include "JointPolicyDiscretePure.h"
#include "TwoStageDynamicBayesianNetwork.h"
#include "MADPComponentFactoredStates.h"

using namespace std;

#define DEBUG_UPDATE 0
#define DEBUG_CONSTRUCTOR 0 
FSAOHDist_NECOF::FSAOHDist_NECOF() :
    _m_puf(0),
    _m_stage(0),
    _m_sfacMarginals(0),
    _m_oHistConditional( 0 ),
    _m_oHistMarginals( 0 ),
    _m_sfacSoI(0),
    _m_sfacSoI_ii_size(0),
    _m_stepsize(0)
{
}


FSAOHDist_NECOF::FSAOHDist_NECOF(const PlanningUnitFactoredDecPOMDPDiscrete* p)
    :
        _m_puf(p)
        ,_m_stage(0)
        ,_m_sfacMarginals( new FSDist_COF() )
        ,_m_oHistConditional( p->GetNrAgents(), 0 )
        ,_m_oHistMarginals( vector< vector<double> >( 
                    p->GetNrAgents(),
                    vector<double>(1, 1.0) ) 
                )
        ,_m_sfacSoI( p->GetFDPOMDPD()->GetNrAgents() )
        ,_m_sfacSoI_ii_size( p->GetFDPOMDPD()->GetNrAgents() )
        ,_m_stepsize( p->GetFDPOMDPD()->GetNrAgents(), 0)
{
    const FactoredDecPOMDPDiscreteInterface* fd =  _m_puf->GetFDPOMDPD();
    const vector<size_t>& nrValsPerSF = fd->GetNrValuesPerFactor();
    //first fill the scope of influence for all agent's (observations)
    for(Index agI=0; agI < _m_puf->GetNrAgents(); agI++)
    {
        Scope sfSc_dummy;
        Scope agSc;
        agSc.Insert(agI);
        _m_sfacSoI.at(agI) = fd->Get2DBN()->GetYSoI_O(agI);
#if DEBUG_CONSTRUCTOR        
        cout << "_m_sfacSoI.at("<<agI<<") is "<< _m_sfacSoI.at(agI)<<endl;
#endif                 
        //compute the stepsize
        vector<size_t> nrValsForScope(_m_sfacSoI.at(agI).size());
        IndexTools::RestrictIndividualIndicesToScope(nrValsPerSF,_m_sfacSoI.at(agI),
                                                     nrValsForScope);
        _m_sfacSoI_ii_size.at(agI) = VectorTools::VectorProduct(nrValsForScope);
        _m_stepsize.at(agI) = IndexTools::CalculateStepSize(nrValsForScope);
    }

    //now fill the probabilities of the (initial=empty) observation
    //history for each agent
    for(Index agI=0; agI < _m_puf->GetNrAgents(); agI++)
    {
        size_t nrXs = _m_sfacSoI_ii_size.at(agI);
        size_t nrOHs = 1;
        CPT* cpt =  new CPT(nrOHs, nrXs );
        for(Index xI=0; xI < nrXs; xI++)
            cpt->Set(0, xI, 1.0);

        _m_oHistConditional.at(agI) =  cpt;
    }
}

//Copy constructor.    
FSAOHDist_NECOF::FSAOHDist_NECOF(const FSAOHDist_NECOF& o)     :
        _m_puf(o._m_puf)
        ,_m_stage(o._m_stage)
        ,_m_sfacMarginals( o._m_sfacMarginals )
        ,_m_oHistConditional( o._m_oHistConditional )
        ,_m_oHistMarginals( o._m_oHistMarginals )
        ,_m_sfacSoI(o._m_sfacSoI)
        ,_m_sfacSoI_ii_size( o._m_sfacSoI_ii_size )
//        ,_m_stepsize(o._m_stepsize)
{
    size_t nrAg = _m_puf->GetNrAgents();
    //make a deep copy of the stepsize arrays
    _m_stepsize.resize(nrAg, 0);
    for( Index agI=0; agI < nrAg; agI++)    
    {
        //check how many factors in the SoI of this agent's (agI's) observation
        size_t nrSFacsInSoI = _m_sfacSoI.at(agI).size();
        size_t * sz_arr = new size_t [nrSFacsInSoI];
        for(Index sfI=0; sfI < nrSFacsInSoI; sfI++)
            sz_arr[sfI] = o._m_stepsize.at(agI)[sfI];
        _m_stepsize.at(agI) = sz_arr;
    }


}
//Destructor
FSAOHDist_NECOF::~FSAOHDist_NECOF()
{
    size_t nrAg = _m_oHistConditional.size();
    if(_m_stepsize.size()== nrAg)
    {
        for( Index agI=0; agI < nrAg; agI++)    
        {
            size_t * sz_arr = _m_stepsize[agI];
            if( sz_arr != 0)
                delete [] sz_arr;
        }
        _m_stepsize.clear();
    }

    for(Index agI=0; agI < nrAg; agI++)
        delete _m_oHistConditional.at(agI);

    delete _m_sfacMarginals;
}
//Copy assignment operator
FSAOHDist_NECOF& FSAOHDist_NECOF::operator= (const FSAOHDist_NECOF& o)
{
    if (this == &o) return *this;   // Gracefully handle self assignment
    // Put the normal assignment duties here...

    return *this;
}

double FSAOHDist_NECOF::GetXProb(
        const Scope& sfSc, 
        const std::vector<Index>& sfValI)
    const
{
    return _m_sfacMarginals->GetProbability(sfSc, sfValI);
}

double FSAOHDist_NECOF::GetOHProb_relT(
        const Scope& agSc, 
        const std::vector<Index>& ohI)
    const
{
    double p = 1.0;
    for(Index i=0; i < agSc.size(); i++)
    {
        Index agI = agSc.at(i);
        Index ohI_agI = ohI.at(i);
        double p_this_agent = _m_oHistMarginals.at(agI).at(ohI_agI);
        p *= p_this_agent;
    }
    return p;
}
double FSAOHDist_NECOF::GetXOHProb_relT(
        const Scope& sfSc, const std::vector<Index>& sfacIndices,
        const Scope& agSc, const std::vector<Index>& ohIndices
        ) const
{
    //we use a running example to illustrate the code:
    // ->we want to compute P(o1o2x1x2)
    //However, o1 additionally depends on x3 and o2 on x4

    //first check if sfSc contains all state factors that can influence
    //the observation (and thus the OHs) of the agents in agentSc, if
    //not, we will need to marginalize over the state factors that are not
    //in sfSc (but are in the SoI).
    
    Scope all_relevant_sfacIs(sfSc);
    for(Index i=0; i < agSc.size(); i++)
        all_relevant_sfacIs.Insert( _m_puf->GetFDPOMDPD()->
                                    Get2DBN()->GetYSoI_O( agSc[i] ) );

    Scope to_marginalize_over_sfacIs(all_relevant_sfacIs);
    to_marginalize_over_sfacIs.Remove(sfSc);

    //p(x1x2) = P(x1)*P(x2)
    double p1 = GetXProb(sfSc, sfacIndices);

    vector<Index> marg_vec(to_marginalize_over_sfacIs.size(), 0);
    vector<size_t> marg_nrSFvals( to_marginalize_over_sfacIs.size());
    IndexTools::RestrictIndividualIndicesToScope(
        _m_puf->GetFDPOMDPD()->GetNrValuesPerFactor(),
        to_marginalize_over_sfacIs,
        marg_nrSFvals);

    //this will hold P(o1o2|x1x2)
    double p_ohIndices_given_sfacIndices = 0.0;
    do {
        //the probability of this instantiation of marg_vec 
        //P(x3x4)
        double p2 = GetXProb(to_marginalize_over_sfacIs, marg_vec);

        if(p2>0) // optimalization: if p2 is zero the rest will be zero as well
        {
            //construct the 'full' state vector (i.e., containing all_relevant_sfacIs)
            //the vector x1x2x3x4
            vector<Index> all_relevant_vals(sfacIndices);
            all_relevant_vals.insert(all_relevant_vals.end(),
                                     marg_vec.begin(),
                                     marg_vec.end());
            
            //P(o1o2|x1x2x3x4)
            double p_ohIndices_given_all_relevant_vals = 1.0;
            for(Index i=0; i < agSc.size(); i++)
            {
                Index agI=agSc.at(i);
                Index ohI=ohIndices.at(i);

                vector<Index> sfacIrestr(_m_sfacSoI.at(agI).size());
                try{
                    IndexTools::RestrictIndividualIndicesToNarrowerScope(
                        all_relevant_vals,
                        all_relevant_sfacIs, 
                        _m_sfacSoI.at(agI),
                        sfacIrestr);
                }catch(ENoSubScope& e)
                { throw ENoSubScope("bah"); }

                Index x = IndexTools::IndividualToJointIndicesStepSize(
                    sfacIrestr, _m_stepsize.at(agI) );

                double p_OHi_given_all_relevant_vals = 
                    _m_oHistConditional.at(agI)->Get(ohI, x);
                //P(o1o2|x1x2x3x4) = P(o1|x1x2x3)P(o2|x1x2x4)
                p_ohIndices_given_all_relevant_vals *=
                    p_OHi_given_all_relevant_vals;
            }
            //P(o1o2x3x4|x1x2) = P(o1o2|x1x2x3x4)P(x3x4)
            double p_ohIndices_marg_vec_given_sfacIndices = 
                p2 * p_ohIndices_given_all_relevant_vals;
            //P(o1o2|x1x2) = sum_{x3x4} P(o1o2x3x4|x1x2) 
            p_ohIndices_given_sfacIndices += 
                p_ohIndices_marg_vec_given_sfacIndices;
        }
    }while (! IndexTools::Increment( marg_vec, marg_nrSFvals ) );

    //finally P(o1o2x1x2):
    double result = p1 * p_ohIndices_given_sfacIndices;
    return(result);
}


double FSAOHDist_NECOF::GetXOHProb_relT_SufficientSFscope(
        const Scope& sfSc, const std::vector<Index>& sfacIndices,
        const Scope& agSc, const std::vector<Index>& ohIndices
        ) const
{    
    double p = GetXProb(sfSc, sfacIndices);


    for(Index i=0; i < agSc.size(); i++)
    {
        Index agI=agSc.at(i);
        Index ohI=ohIndices.at(i);

        vector<Index> sfacIrestr(_m_sfacSoI.at(agI).size());
        try{
            IndexTools::RestrictIndividualIndicesToNarrowerScope(
                sfacIndices, sfSc, _m_sfacSoI.at(agI), sfacIrestr );
        }catch(ENoSubScope& e)
        {
            stringstream ss;
            ss <<"Error FSAOHDist_NECOF::GetXOHProb_relT should be called with all state factors on which the observation histories (of the agents in agSc) depend\n";
            ss <<"sfSc="<<sfSc<<", agSc="<<agSc<<endl;
            string s(ss.str());
            cerr << s;
            throw ENoSubScope(s);
        }
        Index x = IndexTools::IndividualToJointIndicesStepSize(sfacIrestr, 
                _m_stepsize.at(agI) );

        double p_i = _m_oHistConditional.at(agI)->Get(ohI, x);
        p *= p_i;
    }

    return p;

}
void FSAOHDist_NECOF::Update(const JointPolicyDiscretePure& jpol)
{
#if DEBUG_UPDATE    
    cout << "Started update of ..."<<  this->SoftPrint() << endl;
    cout << "for jpol" << jpol.SoftPrint() << endl;
#endif                 
    //make temp copies of old distr.
    FSDist_COF prev_sfacMarginals = *_m_sfacMarginals;
    vector< vector<double> > prev_oHistMarginals = _m_oHistMarginals;
    Index prev_stage = _m_stage;
    //update stage
    _m_stage++;
    //set all sfac probs to 0
    _m_sfacMarginals->SetZero();
    //grow vectors representing OH probs
    for(Index agI=0; agI < _m_puf->GetNrAgents(); agI++)
    {
        _m_oHistMarginals[agI].clear();
        size_t nrOHs = _m_puf->GetNrObservationHistories(agI, _m_stage);
        _m_oHistMarginals[agI] = vector<double>( nrOHs, 0.0 );
        delete _m_oHistConditional[agI];
        
        size_t nrXs = _m_sfacSoI_ii_size.at(agI);
        CPT* cpt =  new CPT(nrOHs, nrXs );
        _m_oHistConditional.at(agI) =  cpt;
    }

    //compute the action probabilities for each agent
    //(note that this is possible because we update from the marginals
    // of the observation histories, otherwise this should be done inside
    // the state-prob update loop)
    vector < vector<double> > p_AgAc( _m_puf->GetNrAgents() );//p_AgAc[agI][acI]
    for(Index agI=0; agI < _m_puf->GetNrAgents(); agI++)        
    {
        p_AgAc[agI].resize(_m_puf->GetNrActions(agI) );
        for(Index ohIwithinPrevStage=0; ohIwithinPrevStage < 
                 _m_puf->GetNrObservationHistories(agI, prev_stage) ;
                 ohIwithinPrevStage++)
        {
            Index ohI = ohIwithinPrevStage + 
                CastLIndexToIndex(_m_puf->GetFirstObservationHistoryIndex(agI, prev_stage));
            Index actionI = jpol.GetActionIndex(agI,ohI);
            p_AgAc[agI][actionI] += 
                prev_oHistMarginals[agI][ohIwithinPrevStage];
        }
    }
    
    const FactoredDecPOMDPDiscreteInterface* fd = _m_puf->GetFDPOMDPD();
    const TwoStageDynamicBayesianNetwork* dbn = fd->Get2DBN();

//update state factor probs:
    //yI is the state factor index in the right side of the 2DBN
    for(Index yI=0; yI < fd->GetNrStateFactors(); yI++)
    {
        //compute probabilities of all values of xI at new stage _m_stage
        const Scope& sfSc = dbn->GetXSoI_Y(yI);
        vector<Index> xIs(sfSc.size(), 0);
        vector<size_t> nr_xIs(sfSc.size());
        IndexTools::RestrictIndividualIndicesToScope(
            fd->GetNrValuesPerFactor(), sfSc, nr_xIs);
        const Scope& agSc = dbn->GetASoI_Y(yI);
        vector<Index> actions(agSc.size(), 0);
        vector<size_t> nr_actions(agSc.size()); 
        IndexTools::RestrictIndividualIndicesToScope(
            _m_puf->GetNrActions(), agSc, nr_actions);
        //NOTE: currently intra-stage Y dependencies are not yet supported. 
        //This is a workaround.
        vector<Index> Yii_dummy;

        do {
            //compute probability of xIs:
            double pX = prev_sfacMarginals.GetProbability(sfSc, xIs);

            do {
                //compute probability of actions:
                double pA = 1.0;
                for(Index i=0; i < agSc.size(); i++)
                {
                    Index agI = agSc[i];
                    pA *= p_AgAc[agI][ actions[i] ];
                }

                vector<double> probs = 
                    fd->Get2DBN()->GetYProbabilitiesExactScopes(
                        xIs, actions, Yii_dummy, yI);
                for(Index yVal=0; yVal < fd->GetNrValuesForFactor(yI); yVal++)
                {
                    _m_sfacMarginals->GetReferrence(yI, yVal) += 
                        probs[yVal] * pX * pA;
                }

            } while ( ! IndexTools::Increment(actions, nr_actions) );
        } while ( ! IndexTools::Increment(xIs, nr_xIs) );
    }
    
//update the OH conditionals and marginals
    for(Index agI=0; agI < _m_puf->GetNrAgents(); agI++)        
    {
        const Scope& aSc = dbn->GetASoI_O(agI);
        const Scope& ySc = dbn->GetYSoI_O(agI);
        const Scope& oSc = dbn->GetOSoI_O(agI);
        //is agI in its own scope of influence?
        //(i.e., does its action affect its observation?)
        bool agI_in_ASoI_O_agI = aSc.Contains(agI);
        // nr Other Influencing Agents
        size_t nrOIA = aSc.size();
        if(agI_in_ASoI_O_agI)
            nrOIA--;
#if 0
        if(nrOIA >= 1)
            throw E("FSAOHDist_NECOF:: influence of other agents j on agent i's observation not (yet?) supported,");
#endif
        if(oSc.size() != 0)
            throw E("FSAOHDist_NECOF:: correlated observations not (yet?) supported,");

        vector<Index> aIs(aSc.size(), 0);
        vector<size_t> nr_aIs(aSc.size());
        IndexTools::RestrictIndividualIndicesToScope(
            _m_puf->GetNrActions(), aSc, nr_aIs);
        vector<Index> yIs(ySc.size(), 0);
        vector<size_t> nr_yIs(ySc.size());
        IndexTools::RestrictIndividualIndicesToScope(
            fd->GetNrValuesPerFactor(), ySc, nr_yIs);
        //compute the probability of all previous OH histories of this agent
        //extended by an observation.
        for(Index ohIwithinPrevStage=0; ohIwithinPrevStage < 
                 _m_puf->GetNrObservationHistories(agI, prev_stage) ;
                 ohIwithinPrevStage++)
        {
            double PprevOHI = prev_oHistMarginals[agI][ohIwithinPrevStage];
            Index prevOHI = ohIwithinPrevStage + 
                CastLIndexToIndex(_m_puf->GetFirstObservationHistoryIndex(agI, prev_stage));
            Index agI_s_actionI = jpol.GetActionIndex(agI, prevOHI);
#if 0
            vector<Index> aIs_dummy; //this should be constructed 
            if(agI_in_ASoI_O_agI)
                aIs_dummy.push_back( agI_s_actionI );
#endif
            //when we want to model observation dependence, this
            //should be constructed to contain the observations that
            //can influence the observation of this agent (agI)
            vector<Index> oIs_dummy;

            do{
                double Py = _m_sfacMarginals->GetProbability(ySc, yIs);
                // we loop over all joint actions for this scope, but
                // only consider the ones in which agI's action is
                // consistent with the policy
                do{
                    if(aIs[aSc.GetPositionForIndex(agI)]==agI_s_actionI)
                    {
                        vector<double> O_probs = 
                            fd->Get2DBN()->GetOProbabilitiesExactScopes(
                                aIs, yIs, oIs_dummy, agI);

                        // marginalize the other influencing agents' actions
                        double pOtherAgents=1;
                        for(Index i=0; i < aSc.size(); i++)
                        {
                            if(aSc[i]!=agI)
                            {
                                Index otherAgent=aSc[i];
                                Index otherAgentAction=aIs[i];
                                pOtherAgents *= 
                                    p_AgAc[otherAgent][otherAgentAction];
                            }
                        }

                        for(Index oI=0; oI < _m_puf->GetNrObservations(agI); oI++)
                        {
                            //P( oI| agI_s_actionI, yIs ) = P( oI| prevOHI, yIs)
                            //Po_given_pOH_aIs_y = P(o|prev.OH, aIs, y) 
                            double Po_given_pOH_aIs_y = O_probs[oI];
                            // P(oI|..) = P(OH | y) = \Sum_aG  - i.e., the probability of actions are 
                            // marginalized out.
                            double P_OH_given_y = Po_given_pOH_aIs_y * PprevOHI * pOtherAgents;
                            // compute indices for the new OH, and y
                            Index sucOHI =_m_puf->GetSuccessorOHI(agI, prevOHI, oI);
                            Index sucOHIwithinStage = sucOHI -
                                CastLIndexToIndex(_m_puf->GetFirstObservationHistoryIndex(agI, _m_stage));
                            Index y = IndexTools::IndividualToJointIndicesStepSize(
                                yIs, _m_stepsize.at(agI) );
                            
                            //add the probability of oI (i.e. OH) for this configuration of actions aIs
                            double pCond=_m_oHistConditional.at(agI)->Get(sucOHIwithinStage, y)
                                + P_OH_given_y;
                            _m_oHistConditional.at(agI)->Set(sucOHIwithinStage, y, pCond);
                            _m_oHistMarginals.at(agI).at(sucOHIwithinStage) += 
                                P_OH_given_y * Py;
                        }
                    }            
                } while ( ! IndexTools::Increment(aIs, nr_aIs) );
            } while ( ! IndexTools::Increment(yIs, nr_yIs) );
        } //end for ohIwithinPrevStage
    }//end for agI  
#if DEBUG_UPDATE    
    cout << "Ended update. Result is ..."<<  this->SoftPrint() << endl;
#endif                 
    this->SanityCheck();
}

void FSAOHDist_NECOF::InitializeFromISD(const FactoredStateDistribution * d)
{
    FactoredStateDistribution *p1=d->Clone();
    FSDist_COF* p =
        dynamic_cast<FSDist_COF*>(p1);
    if(p == 0)
        throw E("FSAOHDist_NECOF::InitializeFromISD cast failed");
    //copy the distribution into this object
    delete _m_sfacMarginals;
    _m_sfacMarginals = p;
}


string FSAOHDist_NECOF::SoftPrint() const
{
    stringstream ss;
    ss << "Nearly completely factored (S,OH)-distribution:"<<endl
        << "state factor marginals="<<endl;
    ss << _m_sfacMarginals->SoftPrint() 
       << "oHist marginals="<< SoftPrintVector(_m_oHistMarginals) << endl;
    ss << "oHist conditionals=" << endl;
//     <<     "Not printed"<< endl <<
    for(Index agI=0; agI < _m_oHistConditional.size(); agI++)        
        ss <<  _m_oHistConditional.at(agI)->SoftPrint() << endl;
    return (ss.str() );

}

void FSAOHDist_NECOF::SanityCheck()
{
    //cout << "Starting FSAOHDist_NECOF::SanityCheck()"<<endl;
    _m_sfacMarginals->SanityCheck();    
    
    for(Index agentI=0; agentI < _m_oHistMarginals.size(); agentI++)
    {
        double psum = 0.0;
        for(Index ohI=0; ohI < _m_oHistMarginals.at(agentI).size(); ohI++)
            psum += _m_oHistMarginals.at(agentI).at(ohI);

        if(!Globals::EqualProbability(psum, 1.0))
        {
            stringstream ss;
            ss << "Warning! - FSDist_COF::SanityCheck agentI="<<agentI<<" does not sum to 1.0, but to " << psum;
            throw(E(ss));
        }
        if(  abs(psum - 1.0) > 1e-12  )
            cout << "Warning -  - FSDist_COF::SanityCheck agentI="<<agentI<<
                "could perhaps use renormalization?"<<endl;
    }
    for(Index agentI=0; agentI < _m_oHistConditional.size(); agentI++)
        _m_oHistConditional.at(agentI)->SanityCheck();


    
}
