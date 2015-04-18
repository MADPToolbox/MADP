/* This file is part of the Multiagent Decision Process (MADP) Toolbox v0.3. 
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

/* Only include this header file once. */
#ifndef _PERSEUS_H_
#define _PERSEUS_H_ 1

/* the include directives */
#include <iostream>
#include <iomanip>
#include "Globals.h"

#include "BeliefValue.h"
#include "AlphaVectorPlanning.h"
#include "directories.h"
#include "PlanningUnitDecPOMDPDiscrete.h"
#include "QAVParameters.h"
#include "argumentHandlers.h"

class BeliefSetNonStationary;

/**Perseus contains basic functionality for the Perseus planner. */
class Perseus :
    virtual public AlphaVectorPlanning
{
private:

    int _m_minimumNumberOfIterations;
    int _m_maximumNumberOfIterations;

    int _m_verbose;

    bool _m_initializeWithImmediateReward;
    bool _m_initializeWithZero;

    size_t GetSize(const ValueFunctionPOMDPDiscrete &V) const
        { return(V.size()); }
    size_t GetSize(const QFunctionsDiscrete &Q) const
        {
            size_t nr=0;
            for(QFDcit i=Q.begin();i!=Q.end();++i)
                nr+=i->size();
            return(nr);
        }
    size_t GetSize(const QFunctionsDiscreteNonStationary &Q) const
        {
            size_t nr=0;
            for(QFDNScit i=Q.begin();i!=Q.end();++i)
                nr+=GetSize(*i);
            return(nr);
        }

protected:

    double _m_bestValue;

    /// Whether or not the belief set has been set.
    bool _m_beliefsInitialized;

    std::string _m_identification, _m_valueFunctionFilename, _m_resultsFilename;

    ValueFunctionPOMDPDiscrete GetInitialValueFunction() const;

    QFunctionsDiscrete GetInitialQFunctions() const;

    QFunctionsDiscreteNonStationary GetInitialNonStationaryQFunctions() const;

    bool _m_storeIntermediateValueFunctions;

    bool _m_storeTimings;

    bool _m_computeVectorForEachBelief;

    bool _m_dryrun;

    template <class VF>
    void PlanStartOfIteration(int iter,
                              const std::vector<double> &VB,
                              const VF &V) const
        {
            double x=0;
            for(unsigned int i=0;i!=VB.size();i++)
                x+=VB[i];
            JointBeliefInterface* jb0 = GetPU()->GetNewJointBeliefFromISD();
            if(GetVerbose() >= -1)
                std::cout 
                    << GetIdentification() << ": iteration " << std::setw(6)
                    << iter 
                    << " |V| " << GetSize(V)
                    << " sumV/nrB " << x/VB.size() << " V0 " 
                    << BeliefValue::GetValue( *jb0, V)
                    << " (best " << _m_bestValue << ")" << std::endl;
            delete jb0;

            if(_m_storeIntermediateValueFunctions && !_m_dryrun)
            {
                std::string resultsDir=directories::MADPGetResultsDir("POMDP",
                                                                 GetPU());
                std::stringstream valueFunctionFilename;
                valueFunctionFilename << resultsDir << "/intermediate/"
                                      << GetIdentification()
                                      << "ValueFunction_h"
                                      << GetPU()->GetHorizon() << "_iter_" 
                                      << std::setw(4) << std::setfill('0')
                                      << iter;
                AlphaVectorPlanning::
                    ExportValueFunction(valueFunctionFilename.str(),V);
            }
        }
    
    template <class VF>
    void PlanEndOfIteration(const VF &V)
        {
            PlanEndOfIteration();
            
            JointBeliefInterface* jb0 = GetPU()->GetNewJointBeliefFromISD();
            double value=BeliefValue::GetValue(*jb0, V);
            delete jb0;

            if(value>_m_bestValue)
            {
                _m_bestValue=value;
                StoreValueFunction(V);
                if(!_m_dryrun)
                    ExportValueFunction(_m_valueFunctionFilename);
            }
        }

    void PlanEndOfIteration() const;

    /// Prints the maximum immediate reward present in the belief set.
    void PrintMaxRewardInBeliefSet() const;

    void PlanLeadIn();

    void PlanLeadOut();

    GaoVectorSet BackupStageLeadIn(const ValueFunctionPOMDPDiscrete &V) const;

    void BackupStageLeadOut(GaoVectorSet Gao) const;

    void UpdateValueFunctionName();

    virtual std::vector<double> GetImmediateRewardBeliefSet() const = 0;

    virtual void InitializeBeliefs(int nrB, bool uniquify) = 0;

    virtual void StoreValueFunction(const ValueFunctionPOMDPDiscrete &V);
    virtual void StoreValueFunction(const QFunctionsDiscrete &Q);
    virtual void StoreValueFunction(const QFunctionsDiscreteNonStationary &Q);
    
public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    Perseus(const PlanningUnitDecPOMDPDiscrete* pu);
    Perseus(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    /// Destructor.
    virtual ~Perseus();

    void Initialize();

    /** Exports the computed value function in a simple text format
     * used by Tony Cassandra for .alpha files. */
    virtual void ExportValueFunction(const std::string &filename) const = 0;

    virtual double GetQ(const JointBeliefInterface &b, Index jaI) const = 0;

    virtual double GetQ(const JointBeliefInterface &b, Index jaI,
                        AlphaVector::BGPolicyIndex &betaMaxI) const = 0;
    
    virtual double GetQ(const JointBeliefInterface &b, Index t,
                        Index jaI) const = 0;

    virtual double GetQ(const JointBeliefInterface &b, Index t, Index jaI,
                        AlphaVector::BGPolicyIndex &betaMaxI) const = 0;

    virtual void SetValueFunction(const std::string &filename) = 0;

    /// Sample an index of a belief in the set which has not been improved.
    int SampleNotImprovedBeliefIndex(std::vector<bool> stillNeedToBeImproved,
                                     int nrNotImproved) const;

    bool CheckConvergence(const std::vector<double> &VB,
                          const std::vector<double> &VBnew,
                          int iter) const;

    virtual void Plan() = 0;

    void SetSaveIntermediateValueFunctions(bool save)
        { _m_storeIntermediateValueFunctions=save; }

    void SetSaveTimings(bool save)
        { _m_storeTimings=save; }

    void SetIdentification(const std::string &identification);
    
    void SetResultsFilename(const std::string &filename);

    std::string GetIdentification() const { return(_m_identification); }

    static std::string BackupTypeToString(const QAVParameters &params);

    static QAVParameters ProcessArguments(const ArgumentHandlers::Arguments
                                          &args);

    void SetComputeVectorForEachBelief(bool compute)
        { _m_computeVectorForEachBelief = compute; }

    void SetMinimumNumberOfIterations(int nr)
        { _m_minimumNumberOfIterations=nr; }
    void SetMaximumNumberOfIterations(int nr)
        { _m_maximumNumberOfIterations=nr; }

    void SetInitializeWithImmediateReward(bool initReward)
        { _m_initializeWithImmediateReward=initReward; }

    void SetInitializeWithZero(bool initZero)
        { _m_initializeWithZero=initZero; }

    void SetDryrun(bool dryrun)
        { _m_dryrun=dryrun; }
    void SetVerbose(int verbose) { _m_verbose=verbose; }
    int GetVerbose() const { return(_m_verbose); }

};

#endif /* !_PERSEUS_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
