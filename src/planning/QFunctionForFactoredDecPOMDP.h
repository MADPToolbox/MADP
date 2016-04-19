/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef _FACTOREDQFUNCTION_H_
#define _FACTOREDQFUNCTION_H_ 1

/* the include directives */
#include <iostream>
#include "Globals.h"
//#include "Referrer.h"
#include "QFunctionForFactoredDecPOMDPInterface.h"
#include "PlanningUnitFactoredDecPOMDPDiscrete.h"

/**\brief QFunctionForFactoredDecPOMDP is a base class for the implementation 
 * of a QFunction for  a Factored DecPOMDP.
 *
 * In particular this function implements the 
 *      QFunctionForFactoredDecPOMDPInterface
 * i.e., the functions:
 * SetPUF and GetPUF() */
class QFunctionForFactoredDecPOMDP : 
    virtual public QFunctionForFactoredDecPOMDPInterface

{
    private:    
        const PlanningUnitFactoredDecPOMDPDiscrete* _m_puf;
        boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> _m_pufShared;
    
    protected:
    
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        QFunctionForFactoredDecPOMDP(
                const PlanningUnitFactoredDecPOMDPDiscrete* puf) :
            _m_puf(puf){}
        QFunctionForFactoredDecPOMDP(
                const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &puf) :
            _m_pufShared(puf){}

        //operators:

        //data manipulation (set) functions:
        
        ///Changes the Planning unit pointed to.
        void SetPUF(const PlanningUnitFactoredDecPOMDPDiscrete* pu)
        { _m_puf = pu;}
        void SetPUF(const boost::shared_ptr<const PlanningUnitFactoredDecPOMDPDiscrete> &pu)
        { _m_pufShared = pu;}
        const PlanningUnitFactoredDecPOMDPDiscrete* GetPUF() const
        {return GetPU();}
        
    //Implement QFunctionForDecPOMDPInterface
        const PlanningUnitFactoredDecPOMDPDiscrete* GetPU() const
        {
            if(_m_puf)
                return _m_puf;
            else
                return _m_pufShared.get();
        }
        // This class inherits the QFunctionJAOHInterface 
        // (which specifies GetQ (Index jaohI, Index jaI) which this should 
        // implement)
        // A side effect is that the below function (which does not make sense)
        // is also inherited...
        void SetPU(const PlanningUnitDecPOMDPDiscrete* pu)
        { throw E("QFunctionForFactoredDecPOMDP needs an PlanningUnitFactoredDecPOMDPDiscrete!");}
        void SetPU(const boost::shared_ptr<const PlanningUnitDecPOMDPDiscrete> &pu)
        { throw E("QFunctionForFactoredDecPOMDP needs an PlanningUnitFactoredDecPOMDPDiscrete!");}

    virtual void ComputeWithCachedQValues(bool computeIfNotCached) 
        { throw E("QFunctionForFactoredDecPOMDP::ComputeWithCachedQValues not yet implemented");}

    virtual void Load(const std::string &filename)
        { throw E("QFunctionForFactoredDecPOMDP::Load not yet implemented");}

    virtual void Save(const std::string &filename) const
        { throw E("QFunctionForFactoredDecPOMDP::Save not yet implemented");}

    virtual std::string GetCacheFilename() const
        { throw E("QFunctionForFactoredDecPOMDP::GetCacheFilename not yet implemented");}

};


#endif /* !_FACTOREDQFUNCTION_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
