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
#ifndef _ALPHAVECTORPLANNING_H_
#define _ALPHAVECTORPLANNING_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"

#include "boost/multi_array.hpp"

#include "VectorSet.h"
#include "BeliefSet.h"
#include "BeliefSetNonStationary.h"
#include "TimedAlgorithm.h"
#include "ValueFunctionPOMDPDiscrete.h"

// for their matrix classes
#include "TransitionModelMapping.h"
#include "TransitionModelMappingSparse.h"
#include "ObservationModelMapping.h"
#include "ObservationModelMappingSparse.h"
#include "EventObservationModelMapping.h"
#include "EventObservationModelMappingSparse.h"

#include "boost/shared_ptr.hpp"

class PlanningUnitDecPOMDPDiscrete;
class PlanningUnitFactoredDecPOMDPDiscrete;
class DecPOMDPDiscreteInterface;
class FactoredDecPOMDPDiscreteInterface;
class AlphaVector;
namespace ArgumentHandlers {
    class Arguments;
};

/// Represents back-projected copies of alpha vectors.
typedef boost::multi_array<VectorSet*,2> GaoVectorSet;
/// An index for a GaoVectorSet.
typedef GaoVectorSet::index GaoVectorSetIndex;

#if USE_POMDPSOLVE_LIBRARY
// we should not include any of Tony Cassandra's header files in one
// of our headers, otherwise the macros in that code (such as Equal())
// mess up our functions
namespace pomdpsolve {

    struct PomdpSolveParamStruct;
    struct AlphaListType;
    typedef struct AlphaListType *AlphaList;


}
#endif

/**AlphaVectorPlanning provides base functionality for alpha-vector based
 * POMDP or BG techniques.  */
class AlphaVectorPlanning : public TimedAlgorithm
{
private:

    /**A pointer to the PlanningUnit (which can only be a 
     * #PlanningUnitDecPOMDPDiscrete or derived type).*/
    const PlanningUnitDecPOMDPDiscrete* _m_pu;
    boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> _m_puShared;

    std::vector<const TransitionModelMapping::Matrix* > _m_T;
    std::vector<const ObservationModelMapping::Matrix* > _m_O;

    std::vector<const TransitionModelMappingSparse::SparseMatrix* > _m_Ts;
    std::vector<const ObservationModelMappingSparse::SparseMatrix* > _m_Os;

    std::vector< std::vector <const EventObservationModelMapping::Matrix* > > _m_Oe;
    std::vector< std::vector <const EventObservationModelMappingSparse::SparseMatrix* > > _m_Oes;
    
    typedef boost::numeric::ublas::compressed_vector<double> SparseVector;
    std::vector<std::vector<SparseVector* > > _m_TsForBackup;
    std::vector<std::vector<SparseVector* > > _m_OsForBackup;
    std::vector<std::vector<std::vector<SparseVector* > > > _m_eventOsForBackup;
    std::vector<std::vector<std::vector<SparseVector* > > > _m_TsOsForBackup;

    bool _m_useSparse;

    GaoVectorSet BackProjectFull(const VectorSet &v) const;
    GaoVectorSet BackProjectSparse(const VectorSet &v) const;

    bool _m_initialized;

    void DeInitialize();

    ValueFunctionPOMDPDiscrete PruneValueFunctionPOMDPSolve(const ValueFunctionPOMDPDiscrete &V) const;


protected:

#if USE_POMDPSOLVE_LIBRARY
    ///Tony's magical parameter structure...
    pomdpsolve::PomdpSolveParamStruct *_m_solve_params;

    ///Convert a AlphaVector to a double * as used by pomdp-solve
    static double* AlphaVectorToDoubleP(const AlphaVector & v);

    ///Convert a vector of AlphaVectors to a AlphaList  as used by pomdp-solve 
    //An AlphaList is a pointer, so no problem to return by value.
    static pomdpsolve::AlphaList AlphaVectorsToAlphaList(const ValueFunctionPOMDPDiscrete &V);
#endif

public:
    // Constructor, destructor and copy assignment.
    /// (default) Constructor
    AlphaVectorPlanning(const PlanningUnitDecPOMDPDiscrete* pu);
    AlphaVectorPlanning(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);
    AlphaVectorPlanning(const PlanningUnitFactoredDecPOMDPDiscrete* pu);
    AlphaVectorPlanning(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &pu);
    /// Destructor.
    virtual ~AlphaVectorPlanning();

    void Initialize();

    /**Returns a ref to the PlanningUnit.*/
    const PlanningUnitDecPOMDPDiscrete* GetPU() const
        {
            if(_m_pu)
                return(_m_pu);
            else
                return(_m_puShared.get());
        }
    
    /// Back projects a value function.
    GaoVectorSet BackProject(const ValueFunctionPOMDPDiscrete &v) const;

    /// Back projects a value function, represented as a VectorSet.
    GaoVectorSet BackProject(const VectorSet &v) const;

    /** Sample a belief set according to the arguments. */
    BeliefSet SampleBeliefs(
        const ArgumentHandlers::Arguments &args) const;

    /** Sample a non-stationary belief set according to the arguments. */
    BeliefSetNonStationary SampleBeliefsNonStationary(
        const ArgumentHandlers::Arguments &args) const;

    /** Sample a non-stationary belief set specifying each argument. */
    BeliefSetNonStationary SampleBeliefsNonStationary(
        int nrBeliefs=10,
        int uniqueBeliefs=0,
        int resetAfter=0,
        int useQMDPforSamplingBeliefs=0,
        double QMDPexploreProb=0.1) const;

    /// Compute the cross-sum of two vector sets.
    static void CrossSum(  const std::vector< AlphaVector > &A,
                    const std::vector< AlphaVector > &B,
                    std::vector< AlphaVector > & output);
    /// Compute the cross-sum of two vector sets.
    VectorSet CrossSum(const VectorSet &A, const VectorSet &B) const;

    /// Computes the union of two vector sets.
    VectorSet Union(const VectorSet &A, const VectorSet &B) const;

    /// Prune a Q-valuefunction.
    QFunctionsDiscrete Prune(const QFunctionsDiscrete &Q) const;

    /// Prune a POMDP valuefunction.
    ValueFunctionPOMDPDiscrete Prune(const ValueFunctionPOMDPDiscrete &V) const;

    /// Prune a VectorSet.
    VectorSet Prune(const VectorSet &V) const;

    /// Exports a value function \a V to file named \a filename.
    static void ExportValueFunction(const std::string & filename,
                                    const ValueFunctionPOMDPDiscrete &V,
                                    bool includeBGindices=true);

    /// Exports a Q functions \a Q to file named \a filename.
    static void ExportValueFunction(const std::string & filename,
                                    const QFunctionsDiscrete &Q,
                                    bool includeBGindices=true);

    /// Exports a Q functions \a Q to file named \a filename.
    static void ExportValueFunction(const std::string & filename,
                                    const QFunctionsDiscreteNonStationary &Q,
                                    bool includeBGindices=true);

    /// Imports a value function from a file named \a filename.
    static ValueFunctionPOMDPDiscrete 
    ImportValueFunction(const std::string & filename);

    static QFunctionsDiscreteNonStationary
    ImportValueFunction(const std::string & filename, size_t nr,
                        size_t nrA, size_t nrS);

    /// Returns the value function induced by the reward model.
    ValueFunctionPOMDPDiscrete GetImmediateRewardValueFunction() const; 

    /// Returns the value function induced by the reward model of \a pu.
    static ValueFunctionPOMDPDiscrete
    GetImmediateRewardValueFunction(const PlanningUnitDecPOMDPDiscrete *pu);

    /**\brief Takes all the vectors from the Q-function Q and throws them together as a 'V' function.
     *
     * Note: this function does NOT perform any maximization (or pruning)
     */
    static ValueFunctionPOMDPDiscrete
    QFunctionsToValueFunction(const QFunctionsDiscrete &Q);

    /// Exports the POMDP represented by \a pu to file named \a filename.
    static void ExportPOMDPFile(const std::string & filename,
                                const DecPOMDPDiscreteInterface *decpomdp);

    /// Exports the POMDP to file named \a filename.
    void ExportPOMDPFile(const std::string & filename) const;

    /// Exports a belief set to a file.
    static void ExportBeliefSet(const BeliefSet &B,
                                const std::string & filename);

    QFunctionsDiscrete
    ValueFunctionToQ(const ValueFunctionPOMDPDiscrete &V) const;

    static QFunctionsDiscrete 
    ValueFunctionToQ(const ValueFunctionPOMDPDiscrete &V,
                     size_t nrA, size_t nrS);

    static std::vector<int> GetDuplicateIndices(const VectorSet &V);

    static bool VectorIsInValueFunction(const AlphaVector &alpha,
                                        const ValueFunctionPOMDPDiscrete &V);

    static bool VectorIsDominated(const AlphaVector &alpha,
                                  const ValueFunctionPOMDPDiscrete &V);

    static bool VectorIsDominated(Index i,
                                  const VectorSet &V,
                                  const std::vector<bool> &vectorsInVtoConsider);

    static VectorSet ValueFunctionToVectorSet(const ValueFunctionPOMDPDiscrete& V);
    static ValueFunctionPOMDPDiscrete VectorSetToValueFunction(const VectorSet& VS,
                                                               Index a=0,
                                                               AlphaVector::BGPolicyIndex betaI=-1);
    static VectorSet* VectorOfVectorsToVectorSet(
        const std::vector<std::vector<double> > &vectors);
    
    static std::string SoftPrint(const VectorSet &VS);

    static bool EqualVS(const VectorSet &VS1, const VectorSet &VS2);

};

#endif /* !_ALPHAVECTORPLANNING_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
