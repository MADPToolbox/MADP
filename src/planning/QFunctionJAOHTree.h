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
#ifndef _QFUNCTIONJOINTHISTORYTREE_H_
#define _QFUNCTIONJOINTHISTORYTREE_H_ 1

/* the include directives */
#include "Globals.h"
#include "QFunctionJAOH.h"

/** \brief QFunctionJAOHTree is represents
 * QFunctionJAOH which store Qvalues in a tree.  */
class QFunctionJAOHTree : public QFunctionJAOH
{
private:    

    bool _m_initialized; 
    
    void Initialize();
    void DeInitialize();

    /// Recursively compute Qvalues based on a particular JOAHTree.
    /** Function that should be reimplemented by derived classes that
     * use ComputeQ() (e.g., QPOMDP, QBG). */
#if QFunctionJAOH_useIndices
    virtual double ComputeRecursively(size_t time_step, 
                                      LIndex jaohI, 
                                      Index lastJAI) = 0;
#else
    virtual double ComputeRecursively(size_t time_step, 
                                      JointActionObservationHistoryTree* jaoht, 
                                      Index lastJAI) = 0;
#endif
    
    /// This function starts the recursive computing of Qvalues.
    /** A lot of Q-value functions are computed in a similar, recursive way.
     * (In particular QPOMDP and QBG)
     *
     * This function starts this calculation.
     *
     * Classes that use this function have to define (reimplement) 
     * ComputeRecursively.
     * */
    void ComputeQ();
    

protected:
    
public:
    // Constructor, destructor and copy assignment.
    /// Default constructor, requires a planning unit.
    QFunctionJAOHTree(const PlanningUnitDecPOMDPDiscrete *pu);
    QFunctionJAOHTree(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    /// Destructor.
    virtual ~QFunctionJAOHTree();

    //get (data) funct        
    /**Return the Qvalue for JointActionObservationHistory Index jaohI and
     * JointAction index jaI.*/
    double GetQ(Index jaohI, Index jaI) const
        {   return (_m_QValues(jaohI,jaI)); }
    
    /**Compute the heuristic. (after associated with an initialized 
     * PlanningUnitDecPOMDPDiscrete)*/
    void Compute();

    void SetPU(const PlanningUnitDecPOMDPDiscrete* pu);
    void SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu);

    void Load(const std::string &filename);
    void Save(const std::string &filename) const;

};


#endif /* !_QFUNCTIONJOINTHISTORYTREE_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
