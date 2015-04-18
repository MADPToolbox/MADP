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
#ifndef _POLICYDISCRETE_H_
#define _POLICYDISCRETE_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
#include "Policy.h"
#include "Interface_ProblemToPolicyDiscrete.h"
#include "Referrer.h"

using namespace PolicyGlobals;

/**\brief PolicyDiscrete is a class that represents a discrete 
 * policy.
 *
 * A `discrete joint policy' is a policy for a discrete problem. I.e., the 
 * problem specifies, for each agent, a discrete domain for the policy and 
 * discrete actions.
 *
 * A PolicyDiscrete from discrete indices over the domain (typically 
 * action-observation histories or types) to (degenerate) probability 
 * distributions over indices over (joint) actions.
 *
 * This means that this class includes both pure and stochastic pure policies.
 *
 * */
class PolicyDiscrete : 
//    public Referrer<Interface_ProblemToPolicyDiscrete>,
    public Policy
{
    private:    
        const Interface_ProblemToPolicyDiscrete* _m_I_PTPD;
        I_PtPD_constPtr _m_I_PTPDshared;
        /// Maintains the PolicyDomainCategory.
        PolicyGlobals::PolicyDomainCategory _m_indexDomCat;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        PolicyDiscrete( const Interface_ProblemToPolicyDiscrete* iptpd,
                        PolicyGlobals::PolicyDomainCategory idc,
                        Index agentI ) :
            Policy(agentI),
            _m_I_PTPD(iptpd),
            _m_I_PTPDshared(),
            _m_indexDomCat(idc){};
        PolicyDiscrete( const I_PtPD_constPtr &iptpd,
                        PolicyGlobals::PolicyDomainCategory idc,
                        Index agentI ) :
            Policy(agentI),
            _m_I_PTPD(0),
            _m_I_PTPDshared(iptpd),
            _m_indexDomCat(idc){};
            
        /// Copy constructor.
        PolicyDiscrete(const PolicyDiscrete& a) :
            Policy(a),
            _m_I_PTPD(a._m_I_PTPD),
            _m_I_PTPDshared(a._m_I_PTPDshared),
            _m_indexDomCat(a._m_indexDomCat)
        {};

        /// Destructor.
        virtual ~PolicyDiscrete()
        {};

        //operators:

        //data manipulation (set) functions:
        /**\brief Sets the category of the domain over which the indices of
         * this policy are specified.
         */
        void SetPolicyDomainCategory(PolicyDomainCategory idc)
        {_m_indexDomCat = idc;}
        
        //get (data) functions:
        /**\brief Returns the Category of the domain over which the indices of
         * this policy are specified.
         */
        PolicyDomainCategory GetPolicyDomainCategory() const
        {return (_m_indexDomCat);}

        /**\brief Returns the probability that the policy specifies action
         * for domain index i.
         *
         * Implementations for pure policies clearly should return 0 or 1.
         */
        virtual double GetActionProb( Index i, Index ja ) const = 0;
        
        /**\brief samples an action for domain index i.
         */
        Index SampleAction( Index i ) const;

        /**\brief return a pointer to the referred 
         * Interface_ProblemToPolicyDiscrete.
         */
        const Interface_ProblemToPolicyDiscrete* GetInterfacePTPDiscrete() const
        {
            if(_m_I_PTPD)
                return _m_I_PTPD;
            else
                return _m_I_PTPDshared.get();
        }

        /// Returns a pointer to a copy of this class.
        virtual PolicyDiscrete* Clone() const = 0;

};


#endif /* !_POLICYDISCRETE_H_ */


// Local Variables: ***
// mode:c++ ***
// End: ***
