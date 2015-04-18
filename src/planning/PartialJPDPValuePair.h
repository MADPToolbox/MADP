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
#ifndef _PARTIALJPDPVALUEPAIR_H_
#define _PARTIALJPDPVALUEPAIR_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "PartialJointPolicyValuePair.h"
#include "BayesianGameIdenticalPayoffSolver_T.h"
#include "boost/shared_ptr.hpp"

class PartialJPDPValuePair;
typedef boost::shared_ptr<PartialJPDPValuePair> PartialJPDPValuePair_sharedPtr;

class PartialJointPolicyDiscretePure;

/**\brief PartialJPDPValuePair represents a (PartialJointPolicyDiscretePure,Value) pair, which
 * stores the full PartialJointPolicyDiscretePure. */
class PartialJPDPValuePair : public PartialJointPolicyValuePair
{
private:    

    /// The pointer to the past joint policy associated with this ppi
    PJPDP_sharedPtr _m_jpol;

    boost::shared_ptr<BayesianGameIdenticalPayoffSolver> _m_bgip_solver;
    boost::shared_ptr<BGCG_Solver> _m_bgcg_solver;
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> >_m_bgip_solver_T;
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVectorForClusteredBG> > _m_bgip_solver_TC;
    
protected:
    
public:
    // Constructor, destructor and copy assignment.

    PartialJPDPValuePair(const PJPDP_sharedPtr &jp,
                         double val);

    /// Copy constructor.
    PartialJPDPValuePair(const PartialJPDPValuePair& a);

    /// Destructor.
    ~PartialJPDPValuePair();

    PJPDP_sharedPtr GetJPol() const
        {return(_m_jpol);}

    boost::shared_ptr<BayesianGameIdenticalPayoffSolver>
    GetBGIPSolverPointer() const { return(_m_bgip_solver); }
    boost::shared_ptr<BGCG_Solver>
    GetBGCGSolverPointer() const { return(_m_bgcg_solver); }
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> >
    GetBGIPSolver_T_PointerJPPV() const { return(_m_bgip_solver_T); }
    boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVectorForClusteredBG> >
    GetBGIPSolver_T_PointerCluster() const { return(_m_bgip_solver_TC); }
    void SetBGIPSolverPointer(
        const boost::shared_ptr<BayesianGameIdenticalPayoffSolver> &bgips)
        { _m_bgip_solver=bgips; }
    void SetBGCGSolverPointer(
        const boost::shared_ptr<BGCG_Solver> &bgcgs)
        { _m_bgcg_solver=bgcgs; }
    void SetBGIPSolver_T_Pointer(
        const boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVector> > &bgips)
        { _m_bgip_solver_T=bgips; }
    void SetBGIPSolver_T_Pointer(
        const boost::shared_ptr<BayesianGameIdenticalPayoffSolver_T<JointPolicyPureVectorForClusteredBG> > &bgips)
        { _m_bgip_solver_TC=bgips; }
    PartialJPDPValuePair* Clone() const;
    void CleanUpBGIPSolver();

    std::string SoftPrint() const;
    std::string SoftPrintBrief() const;
};


namespace std{
    /**\brief Overload the less<Type> template for JPolValPair* (we want less
     * to give an ordering according to values, not addresses...).*/
    template <> 
    struct less< PartialJPDPValuePair * > //struct, so operator() is public by def. 
    {
        bool operator()(const PartialJPDPValuePair* x, const PartialJPDPValuePair* y) const
        { 
            //cout << "specialized less<PartialJPDPValuePair> called!"<<endl;
            return( x->GetValue() < y->GetValue() );
        }

    };
    template <> 
    struct less< PartialJPDPValuePair_sharedPtr > //struct, so operator() is public by def. 
    {
        bool operator()(const PartialJPDPValuePair_sharedPtr x, 
                        const PartialJPDPValuePair_sharedPtr y) const
        { 
            //cout << "specialized less<PartialJPDPValuePair> called!"<<endl;
            return( x->GetValue() < y->GetValue() );
        }

    };
}


#endif /* !_PARTIALJPDPVALUEPAIR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
