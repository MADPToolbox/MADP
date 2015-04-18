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
#ifndef _PLANNINGUNITMADPDISCRETE_H_
#define _PLANNINGUNITMADPDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include <queue>
#include <cmath>
#include "Globals.h"
#include "PlanningUnit.h"
#include "MultiAgentDecisionProcessDiscreteInterface.h"

#include "Interface_ProblemToPolicyDiscretePure.h"

#include "PlanningUnitMADPDiscreteParameters.h"
//#include "Referrer.h"

// forward declarations, very important in this class, as it will be
// included by many many others
class JointBelief;
class JointBeliefSparse;
class JointBeliefInterface;
class JointPolicyDiscrete;
class PolicyDiscretePure;

class ActionHistory;
class JointActionHistory; 
class ObservationHistory;
class ObservationHistoryTree;
class JointObservationHistory;
class JointObservationHistoryTree;
class ActionObservationHistory;
class ActionObservationHistoryTree;
class JointActionObservationHistory;
class JointActionObservationHistoryTree;

#include "ActionHistoryTree.h"
#include "JointActionHistoryTree.h"

/** 
 * \brief PlanningUnitMADPDiscrete represents a Planning unit for a
 * discrete MADP (discrete actions, observations and states).
 */
class PlanningUnitMADPDiscrete : 
    public PlanningUnit,
    public Interface_ProblemToPolicyDiscretePure
    //public Referrer<MultiAgentDecisionProcessDiscreteInterface>
{
private:    
    MultiAgentDecisionProcessDiscreteInterface* _m_madp;

    /// Calls functions to initialize the planning unit.
    void Initialize();

    /// Cleans up and releases allocated memory. 
    void Deinitialize();

    ///A bool indicating whether this Planning unit has been initialized.
    bool _m_initialized;

    /// The parameters for this planning unit.
    PlanningUnitMADPDiscreteParameters _m_params;
        
    /** \brief Constructs all possible observation histories and puts
     * them in a tree per agent.
     *
     * Pointers to (the root nodes of) these trees are stored in
     * _m_observationHistoryTreeRootPointers.
     */
    void InitializeObservationHistories();

    /** \brief Breadth-first construction of the observation histories
     * and obs. history tree for agent agentI.
     * 
     * Returns a pointer to the root of the tree.  The function should
     * only be called from InitializeObservationHistories()!  (should be
     * called in the order of the agents). */
    ObservationHistoryTree* CreateObservationHistoryTree(Index agentI);

    ///Deletes all ObservationHistories(Trees) - used on reinitialization.
    void DeInitializeObservationHistories();
    /** \brief Creates all the joint observation histories.
     * 
     * The root is stored as _m_jointObservationHistoryTreeRoot and
     * all joint obs. histories are accessible by index through
     * _m_jointObservationHistoryTreeVector. */
    void InitializeJointObservationHistories();

    ///Deletes all joint observation histories.
    void DeInitializeJointObservationHistories();

    /**\brief Constructs all possible action histories and puts them
     * in a tree per agent.
     *
     * Pointers to (the root nodes of) these trees are stored in
     * _m_actionHistoryTreeRootPointers.
     */
    void InitializeActionHistories();

    /** \brief Breadth-first construction of the action histories
     * and action history tree for agent agentI.
     * 
     * Returns a pointer to the root of the tree.  The function should
     * only be called from InitializeActionHistories()!  (should be
     * called in the order of the agents). */
    ActionHistoryTree* CreateActionHistoryTree(Index agentI);
    ///Deletes all ActionHistories(Trees) - used on reinitialization.
    void DeInitializeActionHistories();
    /** \brief Creates all the joint action histories.
     * 
     * The root is stored as _m_jointActionHistoryTreeRoot and
     * all joint action histories are accessible by index through
     * _m_jointActionHistoryTreeVector. */
    void InitializeJointActionHistories();

    ///Deletes all joint action histories.
    void DeInitializeJointActionHistories();

    /** \brief Constructs all possible action-observation histories
     * and puts them in a tree per agent.
     *
     * Pointers to (the root nodes of) these trees are stored in
     * _m_observationHistoryTreeRootPointers.
     */
    void InitializeActionObservationHistories();
    /** \brief Breadth-first construction of the action-observation
     * histories and action-obs. history tree for agent agentI.
     * 
     * Returns a pointer to the root of the tree.  The function should
     * only be called from InitializeActionObservationHistories()!
     * (should be called in the order of the agents). */
    ActionObservationHistoryTree* CreateActionObservationHistoryTree(Index 
                agentI);
    /** \brief Deletes all ActionObservationHistories(Trees) - used on
     * reinitialization. */
    void DeInitializeActionObservationHistories();

    /** \brief Creates all the joint action-observation histories.
     *
     * The root is stored as _m_jointActionObservationHistoryTreeRoot
     * and all joint act-obs. histories are accessible by index
     * through _m_jointActionObservationHistoryTreeVector. Depending
     * on _m_params, the joint beliefs should are calculated and
     * stored (and their conditional probabilities).
     */
    void InitializeJointActionObservationHistories();

    ///Deletes all joint action-observation histories.
    void DeInitializeJointActionObservationHistories();

protected:

    /// Do some sanity checks, can be overridden.
    virtual bool SanityCheck() const;

    //Counters that keep track of the number of histories 

    /** \brief A vector that keeps track of the number of observation
     * histories per agent. */
    std::vector<size_t> _m_nrObservationHistories;
    ///The number of joint observation histories.
    size_t _m_nrJointObservationHistories;        
    /** \brief Keeps track of the number of observation histories per agent 
     * per time step. _m_nrObservationHistoriesT[agentI][t] */
    std::vector< std::vector<size_t> > _m_nrObservationHistoriesT;
    ///The number of joint observation histories per time-step.
    std::vector<size_t> _m_nrJointObservationHistoriesT;
    /**\brief The _m_firstOHIforT[aI][t] contains the first observation history
     * for time-step t of agent aI.*/
    std::vector<std::vector<LIndex> > _m_firstOHIforT;
    /**\brief The _m_firstJOHIforT[t] contains the first joint observation 
     * history for time-step t.*/
    std::vector<LIndex> _m_firstJOHIforT;

    /** \brief A vector that keeps track of the number of action
     * histories per agent. */
    std::vector<size_t> _m_nrActionHistories;
    ///The number of joint action histories.
    size_t _m_nrJointActionHistories;
    /**\brief A vector that keeps track of the number of action histories per 
     * agent per time step. _m_nrActionHistoriesT[agentI][t] */
    std::vector< std::vector<size_t> > _m_nrActionHistoriesT;
    ///The number of joint action histories per time-step.
    std::vector<size_t> _m_nrJointActionHistoriesT;
    /**\brief The _m_firstAHIforT[aI][t] contains the first action history
     * for time-step t of agent aI.*/
    std::vector<std::vector<LIndex> > _m_firstAHIforT;
    /**\brief The _m_firstJAHIforT[t] contains the first joint action history
     * for time-step t.*/
    std::vector<LIndex> _m_firstJAHIforT;
        
    /** \brief A vector that keeps track of the number of
     * action-obs. histories per agent. */
    std::vector<size_t> _m_nrActionObservationHistories;
    ///The number of joint actionAction histories.
    size_t _m_nrJointActionObservationHistories;
    /**\brief Keeps track of the number of action-obs. histories per 
     * agent per time step. _m_nrActionObservationHistoriesT[agentI][t] */
    std::vector< std::vector<size_t> > _m_nrActionObservationHistoriesT;
    ///The number of joint actionObservation histories per time-step.
    std::vector<size_t> _m_nrJointActionObservationHistoriesT;
    /**\brief The _m_firstAOHIforT[aI][t] contains the first actionObservation 
     * history for time-step t of agent aI.*/
    std::vector<std::vector<LIndex> > _m_firstAOHIforT;
    /**\brief _m_firstJAOHIforT[t] contains the first joint actionObservation 
     * history for time-step t.*/
    std::vector<LIndex> _m_firstJAOHIforT;

    //Storage of the actual histories:
    
    /** \brief A vector that stores pointers to the roots of the
     * observation history trees of each agent.*/
    std::vector<ObservationHistoryTree *> _m_observationHistoryTreeRootPointers;
    /**A vector which, for each agents, stores a vector with all 
     * ObservationHistoryTree pointers. Used to give access to each
     * ObservationHistory(Tree) by index. */
    std::vector<std::vector<ObservationHistoryTree*> > _m_observationHistoryTreeVectors;
    ///The root node of the joint observation histories tree
    JointObservationHistoryTree* _m_jointObservationHistoryTreeRoot;
    /** \brief A vector which stores a JointObservationHistoryTree pointer.
     *
     * Used to give access to the all JointObservationHistoryTree by
     * index.*/
    std::vector<JointObservationHistoryTree*> _m_jointObservationHistoryTreeVector;

    /**\brief A vector that stores pointers to the roots of the action
     * history trees of each agent.*/
    std::vector<ActionHistoryTree *> _m_actionHistoryTreeRootPointers;
    /**\brief A vector which, for each agents, stores a vector with all
     * ActionHistoryTree pointers. 
     *
     * Used to give access to each ActionHistory(Tree) by index. */
    std::vector<std::vector<ActionHistoryTree*> > _m_actionHistoryTreeVectors;
    ///The root node of the joint action histories tree
    JointActionHistoryTree* _m_jointActionHistoryTreeRoot;
    /**\brief A vector which stores a JointActionHistoryTree pointer.
     *
     * Used to give access to the all JointActionHistoryTree by
     * index.*/
    std::vector<JointActionHistoryTree*> _m_jointActionHistoryTreeVector;
  
    /** \brief A vector that stores pointers to the roots of the
     * action-observation history trees of each agent.*/
    std::vector< ActionObservationHistoryTree *> 
    _m_actionObservationHistoryTreeRootPointers;
    /** \brief A vector which, for each agents, stores a vector with all 
     * ActionObservationHistoryTree pointers. 
     *
     * Used to give access to each ActionObservationHistory(Tree) by
     * index. */
    std::vector<std::vector<ActionObservationHistoryTree*> > 
    _m_actionObservationHistoryTreeVectors;
    
    ///The root node of the joint actionObservation histories tree
    JointActionObservationHistoryTree* 
    _m_jointActionObservationHistoryTreeRoot;
    /** \brief A vector which stores JointActionObservationHistoryTree pointer.
     *
     * Used to give access to the all
     * JointActionObservationHistoryTree by index.*/
    std::vector<JointActionObservationHistoryTree*> 
    _m_jointActionObservationHistoryTreeVector;

    /**A map which is used instead of
     * _m_jointActionObservationHistoryTreeVector when we don't
     * cache all JointActionObservationHistoryTree's.*/
    std::map< LIndex, JointActionObservationHistoryTree*> 
    _m_jointActionObservationHistoryTreeMap;

    //Storage of joint beliefs:
    /**_m_jBeliefCache[i] stores a pointer to the joint belief 
     * corresponding to the i-th JointActionObservationHistory (assuming
     * b^0 is as specified by the problem and that a pure joint policy
     * consistent with the  i-th JointActionObservationHistory is followed)
     */
    std::vector<const JointBeliefInterface*> _m_jBeliefCache;
    /// Stores the _conditional_ probability of this joint belief.
    std::vector<double> _m_jaohConditionalProbs;
    /**Caches the probabilities of JointActionObservationHistory's
     * (assuming b^0 is as specified by the problem and that a pure
     * joint policy consistent with the i-th
     * JointActionObservationHistory is followed).  I.e.,
     * _m_jaohProbsCache[k] = P( jaohI=k | prevJPol, b^0 )
     */
    std::vector<double> _m_jaohProbs;
        
public:
    // Constructor, destructor and copy assignment.
    /// Constructor with specified parameters.
    /** Allows for specification of the planning horizon and a pointer
     * to the problem instance. */
    PlanningUnitMADPDiscrete(
        const PlanningUnitMADPDiscreteParameters &params,
        size_t horizon=3, 
        MultiAgentDecisionProcessDiscreteInterface* p=0
        );
    /// Constructor with default parameters.
    /** Allows for specification of the planning horizon and a pointer
     * to the problem instance.  See
     * PlanningUnitMADPDiscreteParameters for the default settings. */
    PlanningUnitMADPDiscrete(
        size_t horizon=3, 
        MultiAgentDecisionProcessDiscreteInterface* p=0
        );

    /// Destructor.
    ~PlanningUnitMADPDiscrete();

    //operators:

    MultiAgentDecisionProcessDiscreteInterface* GetMADPDI()
    { return _m_madp; }
    const MultiAgentDecisionProcessDiscreteInterface* GetMADPDI() const
    { return _m_madp; }
    
    //data manipulation (set) functions:
        
    ///Sets the problem for which to plan, using a pointer.
    /** Also reinitializes the planning unit. */
    void SetProblem(MultiAgentDecisionProcessDiscreteInterface* madp);
    ///Sets the horizon for the planning problem.
    /** Also reinitializes the planning unit. */
    void SetHorizon(size_t h);

    //get (data) functions:

    //general
        
    /// Gets the number of agents.
    size_t GetNrAgents() const { return(PlanningUnit::GetNrAgents()); }

    ///Returns a reference to the problem of the PlanningUnitMADPDiscrete
    const MultiAgentDecisionProcessDiscreteInterface* GetProblem() const
        {return(GetMADPDI());};
    MultiAgentDecisionProcessDiscreteInterface* GetProblem() 
        {return(GetMADPDI());};
    /// Returns the number of states.
    size_t GetNrStates() const;

    /// Get a pointer to a State by index.
    const State* GetState(Index i) const
        { return(GetMADPDI()->GetState(i)); }

    /// Returns the probability of a state in the initial state distribution.
    double GetInitialStateProbability(Index sI) const
        {return(GetMADPDI()->GetInitialStateProbability(sI));};
    /**\brief Returns a _new_ joint belief with the value of the
     * initial state distribution.
     */
    JointBeliefInterface* GetNewJointBeliefFromISD() const;

    /// Get the parameters for this planning unit.
    const PlanningUnitMADPDiscreteParameters& GetParams() const 
        { return(_m_params); }

    /// Sets the parameters for this planning unit.
    /** Also reinitializes the planning unit. */
    void SetParams(const PlanningUnitMADPDiscreteParameters &params);

    //related to getting (info of) actions
         
    /// Returns the number of actions vector.
    const std::vector<size_t>& GetNrActions() const
        {return (GetMADPDI()->GetNrActions());}
    /// Returns the number of actions of agent agentI.
    size_t GetNrActions(Index agentI) const
        {return (GetMADPDI()->GetNrActions(agentI));}
    ///return the number of joint actions.
    size_t GetNrJointActions() const
        {return (GetMADPDI()->GetNrJointActions());}

    /// Returns a ref to the a-th action of agent agentI.
    const Action* GetAction(Index agentI, Index a) const
        {return(GetMADPDI()->GetAction(agentI, a));}
    /// Returns a ref to the i-th joint action.
    const JointAction* GetJointAction(Index jaI) const
        {return (GetMADPDI()->GetJointAction(jaI));}
    /**\brief Returns the joint action index that corresponds to the array of
     * specified individual action indices.*/
    Index IndividualToJointActionIndices(const Index* 
                                          indivActionIndices) const
        {return (GetMADPDI()->IndividualToJointActionIndices(
                     indivActionIndices));}
    /**\brief Returns the joint action index that corresponds to the vector of
     * specified individual action indices.*/
    Index IndividualToJointActionIndices(const std::vector<Index>& 
                                          indivActionIndices) const
        {return (GetMADPDI()->IndividualToJointActionIndices(
                     indivActionIndices));}

    /** \brief Returns a vector containing the indices of the
     * indiv. actions corresponding to the joint action jaI.*/
    std::vector<Index> JointToIndividualActionIndices(Index jaI) const;
 

    //related to getting (info of) observations
  

    /// Returns the number of observations vector.    
    const std::vector<size_t>& GetNrObservations() const
        {return (GetMADPDI()->GetNrObservations());}
    /// Returns the number of observations of agent agentI.
    size_t GetNrObservations(Index agentI) const
        {return (GetMADPDI()->GetNrObservations(agentI));};
    /// Returns the number of joint observations.
    size_t GetNrJointObservations() const
        {return (GetMADPDI()->GetNrJointObservations());};
    /// Returns a ref to the o-th observation of agent agentI.
    const Observation* GetObservation(Index agentI, Index o) const
        {return(GetMADPDI()->GetObservation(agentI, o));}
    /// Returns a ref to the joI-th joint observation.
    const JointObservation* GetJointObservation(Index joI) 
        const {return (GetMADPDI()->GetJointObservation(joI));}
    /** \brief Returns the joint observation index that corresponds to
     * the vector of specified individual observation indices.*/
    Index IndividualToJointObservationIndices(const std::vector<Index>& inObs) const
        {return(GetMADPDI()->IndividualToJointObservationIndices(inObs));}
    /** \brief Returns a vector containing the indices of the
     * indiv. observations corresponding to the joint observation
     * joI.*/
    std::vector<Index> JointToIndividualObservationIndices(Index joI) const
        {return(GetMADPDI()->JointToIndividualObservationIndices(joI));}
   
    //related to getting histories: 
    /**This function computes the index of a history.
     * It can be a joint or individual history of actions, observations
     * or actions-observations.
     *
     * \li  Index t - the stage
     * \li  Index t_offset - for each stage t, there is an offset
     * \li  const Index indices[] - array of size t containing the actual 
     *      indices of the (J)(A/O)s.
     *      E.g., a vector with joint observation indices.
     * \li  size_t indexDomainSize - the number of indices. I.e., the 
     *      indices range from 0...(indexDomainSize-1)
     */
    Index ComputeHistoryIndex( Index t,  Index t_offset, 
            //const std::vector<Index>& indices,
            const Index indices [], 
            size_t indexDomainSize 
            ) const;

    /**This function computes the indices of the sequence corr. to a history.
     * I.e., it is the reverse of ComputeHistoryIndex.
     *
     * It can be a joint or individual history of actions, observations
     * or actions-observations.
     *
     * \li  Index hI - index of the history
     * \li  Index t - the stage
     * \li  Index t_offset - for each stage t, there is an offset
     * \li  const Index indices[] - array of size t. The function will write
     *      the indices of the (J)(A/O)s corresponding to hI to this array.
     * \li  size_t indexDomainSize - the number of indices. I.e., the 
     *      indices range from 0...(indexDomainSize-1)
     */
    void ComputeHistoryArrays(Index hI, Index t, Index t_offset,
            Index Indices[], //output
            size_t indexDomainSize 
        ) const;


    
    //related to getting (info of) observation histories

    /**Returns a vector containing the indices of the
     * indiv. obs.histories corresponding to the joint observation
     * hist. joI.*/
    const std::vector<Index>& JointToIndividualObservationHistoryIndicesRef(
        Index johI) const;    
    /**Returns a vector containing the indices of the indiv. obs. histories
     * corresponding to the joint observation hist. joI.*/
    std::vector<Index> JointToIndividualObservationHistoryIndices(Index johI)
        const;
    ///converts individual history indices to a joint index
    /**This converts a vector of  individual observation history indices to a 
     * joint observation history index.
     * \li \a t     -   the stage of the individual histories 
     *                  (must be the same, otherwise there is no corresponding
     *                  joint index!)
     * \li \a indivIs - the vector containing the individual indices.
     */
    Index IndividualToJointObservationHistoryIndex(Index t, 
           const std::vector<Index>& indivIs) const;

    /** Returns the number of observation histories for agentI. The
     * planning unit (specifically _m_nrObservationHistories) must be
     * initialized when calling this function. */
    size_t GetNrObservationHistories(Index agentI) const;
    /** Returns a vector with the number of OHs for each agent. This 
     * calls GetNrObservationHistories(Index agentI) */
    const std::vector<size_t> GetNrObservationHistoriesVector() const;    
    /** Returns the number of observation histories for agentI for
     * time step ts.  The planning unit (specifically
     * _m_nrObservationHistories) must be initialized when calling
     * this function. */
    size_t GetNrObservationHistories(Index agentI, Index ts) const
        {return _m_nrObservationHistoriesT.at(agentI).at(ts);}
    /** Returns a vector with the number of OHs in stage ts for each agent. This 
     * calls GetNrObservationHistories(Index agentI, Index ts) */
    const std::vector<size_t> GetNrObservationHistoriesVector(Index ts) const;    
    ///Returns the number of joint observation histories.
    size_t GetNrJointObservationHistories() const
        {return _m_nrJointObservationHistories;}
    ///Returns the number of joint observation histories for time step ts.
    size_t GetNrJointObservationHistories(Index ts) const
        {return _m_nrJointObservationHistoriesT.at(ts);}
    ///Returns the index of the first ts observation history of agent agI.
    LIndex GetFirstObservationHistoryIndex(Index agI, Index ts) const
        {return _m_firstOHIforT.at(agI).at(ts);}
    /**Returns the index of the first joint observation history of time
     * step ts*/
    LIndex GetFirstJointObservationHistoryIndex(Index ts) const
        {return _m_firstJOHIforT.at(ts);}
    ///Returns the time step of observation history ohI.
    Index GetTimeStepForOHI(Index agentI, Index ohI) const;       
    ///Returns the time step of joint observation history johI.
    Index GetTimeStepForJOHI(Index johI) const;       
    /**Returns the index of the successor of observation history johI via
     * joint observation joI. I.e., johIsuc = (johI, joI).
     */
    Index GetSuccessorJOHI(Index johI, Index joI) const;
    /**Returns the index of the successor of observation history ohI 
     * of agentI via observation joI. I.e., johIsuc = (johI, joI).
     */
    Index GetSuccessorOHI(Index agentI, Index ohI, Index oI) const;
    

    ///Returns a pointer to observation history# ohI of agent# agentI.
    ObservationHistoryTree* GetObservationHistoryTree(Index agentI,
                                                      Index ohI) const;
    ///Returns a pointer to joint observation history#.
    JointObservationHistoryTree* GetJointObservationHistoryTree(
        Index johI) const;        
    /// Returns the index of a JointObservationHistoryTree pointer.
    Index GetJointObservationHistoryIndex(JointObservationHistoryTree* joh)
        const;
    /**\brief converts the vector observations of length t to a
     * (individual) ObservationHistory Index for agentI.
     *
     * observation needs to be a vector of length > t.  The index of
     * the history formed by the first t observations is returned.
     */
    Index GetObservationHistoryIndex(Index agentI, Index t, 
                                     const std::vector<Index>& observations) const;
    /**\brief converts the vector joint observations of length t
     * to a JointObservationHistory Index.
     *
     * jointObservation needs to be a vector of length > t.  The index
     * of the history formed by the first t joint observations is
     * returned.
     */
    Index GetJointObservationHistoryIndex(Index t, 
            const std::vector<Index>& jointObservations) const;
    /**\brief converts the vector joint observations of length t to a
     * JointObservationHistory Index.
     *
     * jointObservation needs to be a vector of length > t.  The index
     * of the history formed by the first t joint observations is
     * returned.
     */
    Index GetJointObservationHistoryIndex( Index t, 
                                const Index jointObservations[] ) const;

    /**\brief Computes the joint observations for johI.
     *
     * This function _computes_ (i.e., does no look up).
     * \li \a johI     -   the JointObservationHistory index
     * \li \a t        -   the stage of jaohI
     * \li \a joIs     -   an array of size t, which will be filled with 
     *                     observ. o^1,...,o^t
     *
     * This function is called by JointToIndividualObservationHistoryIndices
     * (used when obs-histories are not cached).     */
    void GetJointObservationHistoryArrays
    (Index johI, Index t, Index joIs[]) const;

    /**
     * \brief Computes the observations for ohI.
     *
     * This function _computes_ (i.e., does no look up).
     * \li \a agentI   -   the index of the relevant agent 
     * \li \a ohI      -   the ObservationHistory index
     * \li \a t        -   the stage of jaohI
     * \li \a oIs      -   an array of size t, which will be filled with 
     *                     observations o^1,...,o^t
     *
     * This function is called by JointToIndividualObservationHistoryIndices
     * (used when obs-histories are not cached).*/
    void GetObservationHistoryArrays
    (Index agentI, Index ohI, Index t, Index oIs[]) const;    




    //related to getting (info of) action histories
        
    /**Returns a vector containing the indices of the individual
     * ObservationHistory s corresponding to the JointActionHistory
     * index JAHistI. This method does not depend on a cached vector.
     * (and thus also works if JointActionHistory s are not
     * generated.*/
    std::vector<Index> JointToIndividualActionHistoryIndices(Index JAHistI)
        const;
    /**Returns a reference to a cached vector containing the indices of the
     * indiv. action histories corresponding to the joint action history
     * index JAHistI.  NOTE: The cached vector (of which the reference is
     * returned) is stored in the generated JointActionHistory. If joint
     * action histories are not generated (see PlanningUnitMADPDiscrete
     * constructor), this method will fail.  */
    const std::vector<Index>& JointToIndividualActionHistoryIndicesRef(Index
                                                              JAHistI) const;
    ///converts individual history indices to a joint index
    /**This converts a vector of  individual observation history indices to a 
     * joint observation history index.
     * \li \a t     -   the stage of the individual histories 
     *                  (must be the same, otherwise there is no corresponding
     *                  joint index!)
     * \li \a indivIs - the vector containing the individual indices.
     */
    Index IndividualToJointActionHistoryIndex(Index t, 
            const std::vector<Index>& indivIs) const;
    /**\brief Computes the joint actions for jahI.
     *
     * This function _computes_ (i.e., does no look up).
     * \li \a jahI     -   the JointActionHistory index
     * \li \a t        -   the stage of jaohI
     * \li \a jaIs     -   an array of size t, which will be filled with 
     *                     actions a^1,...,a^t
     */
    void GetActionHistoryArrays
        (Index agentI, Index ahI, Index t,  Index aIs[]) const;
    /**\brief converts the vector joint observations of length t
     * to a JointObservationHistory Index.
     *
     * jointObservation needs to be a vector of length > t.  The index
     * of the history formed by the first t joint observations is
     * returned.
     */
    Index GetJointActionHistoryIndex(Index t, 
            const std::vector<Index>& jointActions) const;
    /**\brief converts the vector joint observations of length t to a
     * JointObservationHistory Index.
     *
     * jointObservation needs to be a vector of length > t.  The index
     * of the history formed by the first t joint observations is
     * returned.
     */
    Index GetJointActionHistoryIndex( Index t, 
                                const Index jointActions[] ) const;
    /** Returns the number of action histories for agentI. The
     * planning unit (specifically _m_nrActionHistories) must be
     * initialized when calling this function. */
    size_t GetNrActionHistories(Index agentI) const;
    /** Returns the number of action histories for agentI for
     * time step ts.  The planning unit (specifically
     * _m_nrActionHistories) must be initialized when calling
     * this function. */
    size_t GetNrActionHistories(Index agentI, Index ts) const
        {return _m_nrActionHistoriesT.at(agentI).at(ts);}
    ///Returns a pointer to action history# ohI of agent# agentI.
    ActionHistoryTree* GetActionHistoryTree(Index agentI,
                                            Index ohI) const;
    ///Returns a pointer to joint action history#.
    JointActionHistoryTree* GetJointActionHistoryTree(
        Index jahI) const;        
    /// Returns the index of a JointActionHistoryTree pointer.
    Index GetJointActionHistoryIndex(JointActionHistoryTree* joh) const;
    ///Returns the number of joint action histories.
    size_t GetNrJointActionHistories() const;
    /**Returns the index of the first joint action observation
     * history of time step ts*/
    LIndex GetFirstJointActionObservationHistoryIndex(Index ts) const
        {return _m_firstJAOHIforT.at(ts);}
    ///Returns the time step of observation history ohI.
    Index GetTimeStepForAHI(Index agentI, Index ohI) const;       
    ///Returns the time step of joint observation history johI.
    Index GetTimeStepForJAHI(Index johI) const;       
    Index GetSuccessorJAHI(Index johI, Index joI) const;    
    Index GetSuccessorAHI(Index agentI, Index ohI, Index oI) const;

    //related to getting (info of) action-observation histories
        

    /**Returns a vector containing the indices of the indiv.
     * act.-obs.histories corresponding to the joint observation hist.
     * joI.*/
    const std::vector<Index>&
    JointToIndividualActionObservationHistoryIndicesRef(LIndex jaohI)
        const;    
    /**Returns a vector containing the indices of the indiv.
     * act.-obs.history indices corresponding to the joint observation hist.
     * joI.*/
    std::vector<Index> JointToIndividualActionObservationHistoryIndices(
        LIndex jaohI) const;

    /**\brief computes the vectors of actions and obs. corresponding to jaohI
     * indivO_vec[agentI][t] = oI
     */
    void JointAOHIndexToIndividualActionObservationVectors(
            LIndex jaohI,
            std::vector< std::vector<Index> >& indivO_vec,
            std::vector< std::vector<Index> >& indivA_vec
            ) const;
    /**\brief computes the vector of action-observations corresponding to jaohI
     * indivAO_vec[agentI][t] = aoI
     */
    void JointAOHIndexToIndividualActionObservationVectors(
            LIndex jaohI,
            std::vector< std::vector<Index> >& indivAO_vec
            ) const;
    ///converts individual history indices to a joint index
    /**This converts a vector of  individual observation history indices to a 
     * joint observation history index.
     * \li \a t     -   the stage of the individual histories 
     *                  (must be the same, otherwise there is no corresponding
     *                  joint index!)
     * \li \a indivIs - the vector containing the individual indices.
     */
    LIndex IndividualToJointActionObservationHistoryIndex(Index t, 
            const std::vector<Index>& indivIs) const;

    /** Returns the number of action observation histories for agentI. 
     * The planning unit (specifically _m_nrObservationHistories) must be 
     * initialized and action-observation histories must have been created
     * when calling this function. */
    size_t GetNrActionObservationHistories(Index agentI) const
        {return _m_nrActionObservationHistories.at(agentI);}
    ///Returns a pointer to observation history# ohI of agent# agentI.
    ActionObservationHistoryTree* GetActionObservationHistoryTree(
        Index agentI, Index aohI) const
        {return _m_actionObservationHistoryTreeVectors.at(agentI).at(aohI);}

    ///Returns the number of jointActionObservation histories.
    size_t GetNrJointActionObservationHistories() const
        {return _m_nrJointActionObservationHistories;}
    ///Returns a pointer to JointActionObservation history#.
    JointActionObservationHistoryTree* 
    GetJointActionObservationHistoryTree(LIndex jaohI) const;
        
    /**\brief converts the vectors of actions and observations of length t
     * to a (individual) ObservationHistory Index for agentI.
     */
    Index GetActionObservationHistoryIndex(
        Index agentI, 
        Index t, 
        const std::vector<Index>& actions,
        const std::vector<Index>& observations
        ) const;
    
    ///Returns the time step of joint action-observation history\a aohI.
    Index GetTimeStepForAOHI(Index agentI, Index aohI) const;       
    ///Returns the time step of joint action-observation history\a jaohI.
    Index GetTimeStepForJAOHI(LIndex jaohI) const;       
    /**Returns the index of the successor of joint
     * action-observation history \a jaohI via joint action \a jaI
     * and joint observation \a joI.
     */
    Index GetSuccessorAOHI(Index agI, Index aohI, Index aI, Index oI) const; 
    /**Returns the index of the successor of agent agI's
     * action-observation history \a aohI via action \a aI
     * and observation \a oI.
     */
    LIndex GetSuccessorJAOHI(LIndex jaohI, Index jaI, Index joI) const; 

    /// Register a new \a jaoht in the vector of indices. 
    void RegisterJointActionObservationHistoryTree(
        JointActionObservationHistoryTree* jaoht);

    /**\brief Computes the joint actions and observations for aohI.
     *
     * This function _computes_ (i.e., does no look up).
     * \li \a agentI    -   the agent index
     * \li \a aohI      -   the JointActionObservationHistory index
     * \li \a t         -   the stage of jaohI
     * \li \a aIs       -   an array of size t, which will be filled with 
     *                      actions a^0,...,a^(t-1)
     * \li \a oIs      -   an array of size t, which will be filled with 
     *                      observ. o^1,...,o^t
     *
     */
    void GetActionObservationHistoryArrays
    (Index agentI, Index aohI, Index t, Index aIs[], Index oIs[]) const;
    
    /// Returns the index of a JointActionObservationHistoryTree pointer.
    LIndex GetJointActionObservationHistoryIndex(
        JointActionObservationHistoryTree* jaoh) const;

    /**\brief converts the vectors of actions and observations of length t to a
     * joint ActionObservationHistory Index.
     *
     * Jactions and Jobservation needs to be a vector of length > t.  
     */
    Index GetJointActionObservationHistoryIndex(
        Index t, 
        const std::vector<Index>& Jactions,
        const std::vector<Index>& Jobservations
        ) const;

    /**\brief like GetJointActionObservationHistoryIndex, but works on a subset
     * of agents.
     */
    Index GetJointActionObservationHistoryIndex(
        Index t, 
        const std::vector<Index>& Jactions,
        const std::vector<Index>& Jobservations,
        const Scope& agentSope
        ) const;
    /**\brief Computes the joint actions and observations for jaohI.
     *
     * This function _computes_ (i.e., does no look up).
     * \li \a jaohI    -    the JointActionObservationHistory index
     * \li \a t        -    the stage of jaohI
     * \li \a jaIs     -    an array of size t, which will be filled with 
     *                      joint actions a^0,...,a^(t-1)
     * \li \a joIs     -    an array of size t, which will be filled with 
     *                      joint observ. o^1,...,o^t
     *
     * This function is called by GetJAOHProbs() and 
     * JointToIndividualActionObservationHistoryIndices()
     * (when the aoh's are not cached). 
     */
    void GetJointActionObservationHistoryArrays
    (LIndex jaohI, Index t, Index jaIs[], Index joIs[]) const;
    /**\brief returns the vectors with joint actions and observations
     * for JointActionObservationHistory jaohi.
     *
     * This function first tries to find jaohI in the cache. If it doesn't
     * find it there, it computes the desired result using 
     * GetJointActionObservationHistoryArrays.
     */
    void GetJointActionObservationHistoryVectors (LIndex jaohI,
                                                  std::vector<Index> &jaIs,
                                                  std::vector<Index> &joIs) const;

    //related to joint beliefs
    //
    /**\brief a function that forces derives classes to specify which 
     * types of joint beliefs are used.
     *
     * 
     * The function returns a pointer to a joint belief that is allocated with 
     * *new*. Therefore it should also be *delete*d.
     *
     * The function is virtual, so it can be overriden in derived classes.
     * The behavior implemented here is as follows:
     * 
     * When _m_params._m_useSparseBeliefs 
     * -> return a JointBeliefSparse 
     * otherwise return a JointBelief.
     */
    virtual JointBeliefInterface * GetNewJointBeliefInterface() const;
    virtual JointBeliefInterface * GetNewJointBeliefInterface(size_t size) 
        const;
        
    /**brief Returns a pointer to a _new_ joint belief.
     *
     * When joint beliefs are cached, this function returns a new joint
     * belief, which is a copy of the one stored in the joint belief cache.
     *
     * When joint beliefs are not cached, this function simple calculates
     * and returns a new belief.
     *
     * In all cases, the programming is responsible to clean up! (i.e.,
     * 'delete' the ptr).
     *
     *
     */
    JointBeliefInterface* GetJointBeliefInterface(LIndex jaohI) const;

    /**Gives the _conditional_ probability of the realization of the joint
     * action-observation history jaohI (and thus of the joint belief
     * corresponding to JointActionObservationHistory jaohI).  This
     * probability is conditional GIVEN the previous joint
     * action-observation history (joint belief) AND joint action.  I.e.
     * P(h'|h,a) = P(h,a,h') / P(h,a) */
    double GetJAOHProbGivenPred(LIndex jaohI) const;
    
    /**\brief returns the probability of jaohI.
     *
     * 
     */
    double GetJAOHProb(
        LIndex jaohI, // the jaohI for which we want to know the probs
        Index p_jaohI = 0, // the GIVEN predecessor
        const JointBeliefInterface* p_jb = NULL,// the corresponding GIVEN
        // jb of p
        const JointPolicyDiscrete * jpol = NULL // the policy followed in
        ) const;
    /**\brief returns the probability of jaohI AND the corresponding joint
     * belief given the predecessor p_jaohI (and its corresponding belief)
     *
     *  -output args:
     *      JointBeliefInterface jb -the joint belief at jaohI
     *  -input args:
     *      Index jaohI             -the jaohI for which we want to know 
     *                              the probs.
     *      Index p_jaohI = 0       -the GIVEN predecessor, when none is
     *                              specified, the initial (empty) jaoh
     *                              is assumed.
     *      JointBelief* p_jb = NULL-the corresponding GIVEN jb of p
     *      JointPolicyDiscrete * jpol = NULL 
     *                              -the policy followed starting from
     *                              p_jaohI. (see remark below)
     *  -returns    
     *      the probability of jaohI, given p_jaohI.
     *
     * jpol is the joint policy that is followed for the stages between
     * p_jaohI and jaohI. It doesn't matter if given jpol, the prob.
     * of p_jaohI is 0; this function returns P(jaohI) GIVEN p_jaohI and 
     * following jpol consequently.
     *
     * \sa the manual documentation.
     * 
     */
    double GetJAOHProbs(
        //output args:
        JointBeliefInterface* jb, 
        //input args:
        LIndex jaohI, // the jaohI for which we want to know the probs
        LIndex p_jaohI = 0, // the GIVEN predecessor
        const JointBeliefInterface* p_jb = NULL,// the corresponding GIVEN
        // jb of p
        const JointPolicyDiscrete * jpol = NULL // the policy followed in
        ) const;
    /**the function that perfoms most of the work, called by GetJAOHProbs.
     *
     * given the joint belief jb, held for joint action observation history
     * p_joahI, this function performs the joint belief updates 
     * corresponding to the sequence
     *
     * jaIs[t_p], joIs[t_p], ..., jaIs[t], joIs[t]
     *
     * the returned value is the probability of this sequence, given
     * the start p_jaohI, and given the policy followed jpol.
     *
     * The joint belief corresponding to this sequence is returned through
     * jb (the belief jb is simply updated.)
     *
     */
    double GetJAOHProbsRecursively(
        //input/output args:
        JointBeliefInterface* jb, 
        //input args:
        const Index jaIs[],
        const Index joIs[],
        Index t_p,
        Index t,
//            const Index jaohI, //the jaohI for which we want to know the probs
        LIndex p_jaohI = 0, // the GIVEN predecessor
//            const JointBelief* p_jb = NULL, // the corresponding GIVEN jb of p
        const JointPolicyDiscrete * jpol = NULL // the policy followed in
        ) const;

    //related to policies
    /// Returns the number of policies for agentI.
    LIndex GetNrPolicies(Index agentI) const;

    /// Returns the number of joint policies.
    LIndex GetNrJointPolicies() const;

    //Getting probabilities and rewards
    
    /// Returns the trans. prob for state, joint action, suc state indices
    double GetTransitionProbability(Index sI, Index jaI, Index sucSI) 
        const
        { return(GetMADPDI()->GetTransitionProbability(sI, jaI, sucSI));};
    /// Returns P(joI | jaI, sucSI ). Arguments are time-ordered.
    double GetObservationProbability(Index jaI, Index sucSI, Index joI) 
        const
        { return(GetMADPDI()->GetObservationProbability(jaI, sucSI, joI));};

    //other
    /// Prints the action histories for all agents
    void PrintActionHistories();
    /// Prints the observation histories for all agents
    void PrintObservationHistories();
    /// Prints the actionObservation histories for all agents
    void PrintActionObservationHistories();
    /// Prints info regarding the planning unit. 
    void Print();

    const TransitionModelDiscrete* GetTransitionModelDiscretePtr() const
    {   
#if 0
        std::cerr << 
        "GetTransitionModelDiscretePtr deprecated, use GetProblem->GetTGet()"
            <<std::endl;
#endif
        return(GetMADPDI()->GetTransitionModelDiscretePtr()); 
    }    
    const ObservationModelDiscrete* GetObservationModelDiscretePtr() const
    { return(GetMADPDI()->GetObservationModelDiscretePtr()); }

    //Interface_ProblemToPolicyDiscrete no longer implemented!
    //functions for Interface_ProblemToPolicyDiscrete

    /// soft prints ObservationHistory ohIndex of agent agentI.
    std::string SoftPrintObservationHistory(Index agentI, Index ohIndex) const;
    /// soft prints action actionI of agent agentI.
    std::string SoftPrintAction(Index agentI, Index actionI) const;
    
    virtual bool AreCachedJointToIndivIndices(
        PolicyGlobals::PolicyDomainCategory pdc) const;
    
    std::vector<Index> JointToIndividualPolicyDomainIndices(Index jdI,
                                                       PolicyGlobals::PolicyDomainCategory cat) const;
    const std::vector<Index>& JointToIndividualPolicyDomainIndicesRef( 
        Index jdI, PolicyGlobals::PolicyDomainCategory cat) const;
    std::string SoftPrintPolicyDomainElement(Index agentI, Index dI,
                                        PolicyGlobals::PolicyDomainCategory cat ) const;
    size_t GetNrPolicyDomainElements(Index agentI,
                                     PolicyGlobals::PolicyDomainCategory cat,
                                     size_t depth=MAXHORIZON) const;
    
    /*\brief the default PolicyDomainCategory for the planning unit.
     *
     * as specified by Interface_ProblemToPolicyDiscrete. This can be 
     * overriden in derived classes.
     */
    virtual PolicyGlobals::PolicyDomainCategory GetDefaultIndexDomCat() const;

    /// Export a policy in dot format (from the GraphViz tools).
    void ExportDotGraph(const std::string &filename,
                        const PolicyDiscretePure &policy,
                        Index agentI,
                        bool labelEdges=true) const;

    /// Convert a policy to dot format (from the GraphViz tools).
    std::string PolicyToDotGraph(const PolicyDiscretePure &policy,
                                 Index agentI,
                                 bool labelEdges=true) const;

};


#endif /* !_PLANNINGUNITMADPDISCRETE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
