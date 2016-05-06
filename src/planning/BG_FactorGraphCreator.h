/* REPLACE_MADP_HEADER */
/* REPLACE_CONTRIBUTING_AUTHORS_START
 * Frans Oliehoek 
 * Matthijs Spaan 
 * REPLACE_CONTRIBUTING_AUTHORS_END
 */

/* Only include this header file once. */
#ifndef BG_FACTORGRAPHCREATOR_H_
#define BG_FACTORGRAPHCREATOR_H_ 1

/* the include directives */
#include "Globals.h"
//#include "BGCG_Solver.h"
#include "BayesianGameCollaborativeGraphical.h"
#include "factorgraph.h"

/** \brief BG_FactorGraphCreator will create a FG from a BG.
 *
 * Currently only works for CGBG (BayesianGameCollaborativeGraphical)
 * \todo : extend to other BG types!
 *
 * known bugs: 
 * -will fail for CGBGs in which not all agents participate in at least 1 payoff
 *  components. 
 *  (in that case libDAI will not include the variables corresponding to the excluded 
 *  agent(s) in the graphical model. However, the labels will not be changes, so
 *  an out of index exception is thrown)
 *
 *
 **/
class BG_FactorGraphCreator 
{
    public:
        enum BGFactorGraph_t {AgentTypeIndependence, AgentIndependence, TypeIndependence};

    private:    
        //the BG were constructing for
        boost::shared_ptr<const BayesianGameCollaborativeGraphical> _m_bg;
        //the type of FG to be created
        BGFactorGraph_t _m_FGt;

        // _m_var_indices[agI][tI] stores the (agent,type)-pair index atI
        std::vector< std::vector<Index> > _m_var_indices;    

        // _m_vars stores the variables. _m_vars[atI]
        std::vector< libDAI::Var > _m_vars;
        
        // _m_facs stores the factores
        std::vector< libDAI::Factor > _m_facs;

        //the factor graph
        libDAI::FactorGraph * _m_FG;

        ///verbosity level:
        int _m_verbosity;

        ///Do we want to exploit sparseness of joint type space?
        bool _m_exploitSparse;
    
    protected:
        //  --variable construction--
        
        ///construct variables for each (agent-type) pair - values are actions
        /**an assignment of variables (v3, v8, v14) corresponds to an assignment
         * of joint actions (a1, a2, a3) for a particular joint type. I.e., each 
         * (the action taken for each) (agent,type)-pair is a variable of the FG
         */
        void Construct_AgentTypePair_Variables();
        
        ///constuct variables for each agent (i.e., no type independence) 
        /**There is 1 variable for each agent, the values are BG policies
         */
        void Construct_AgentBGPolicy_Variables();

        //  --factor construction--
        ///constructs factors for each local joint type: agent+type indep.
        void Construct_LocalJointType_Factors();
        ///constructs factors for each joint type: type indep.
        void Construct_JointType_Factors();
        ///constructs factors for each local payoff function: agent indep.
        void Construct_LocalPayoff_Factors();        
        ///this constructs the strategic game: 1 single factor with agents: no indep.
        void Construct_Payoff_Factor();

        const libDAI::FactorGraph* CreateFG_AgentTypeIndepence();
        const libDAI::FactorGraph* CreateFG_AgentIndepence();
        const libDAI::FactorGraph* CreateFG_TypeIndepence();
        
        /// the amount of random perturbation to add to the factors:
        /** \todo TODO perhaps we want to be able to set this?!
         */
        double PerturbationTerm()
        {
            double d = (PROB_PRECISION *rand()) / RAND_MAX;
            return d;
        }
    public:
        // Constructor, destructor and copy assignment.
        /// (default) Constructor
        BG_FactorGraphCreator(
            const boost::shared_ptr<const BayesianGameCollaborativeGraphical> &bg,
            BGFactorGraph_t type = AgentTypeIndependence,
            int verbosity = 2,
            bool exploitSparseness = false
            );
        
        /// Copy constructor.
        BG_FactorGraphCreator(const BG_FactorGraphCreator& a)
        { throw E("BG_FactorGraphCreator::<COPY CTOR> not implemented - nor sure if desirable and unclear whether deep copy (including the contained factor graph) should be made"); };
        /// Destructor.
        ~BG_FactorGraphCreator();
        /// Copy assignment operator
        BG_FactorGraphCreator& operator= (const BG_FactorGraphCreator& o)
        { throw E("BG_FactorGraphCreator::<ASSIGNMENT OPERATOR> not implemented - nor sure if desirable and unclear whether deep copy (including the contained factor graph) should be made"); };;

        /// actually creates the FG
        const libDAI::FactorGraph*  CreateFG();

        /// returns a pointer to the FG
        const libDAI::FactorGraph*  GetFG()
        {return _m_FG;}


        Index GetVariableIndexForAgentType(Index agI, Index typeI) const;


        BGFactorGraph_t GetBGFactorGraph_t() const
        { return  _m_FGt; }

        static std::string SoftPrint(BGFactorGraph_t bgfg)
        {
            switch(bgfg)
            {
            case AgentTypeIndependence:
                return("AgentTypeIndependence");
            case AgentIndependence:
                return("AgentIndependence");
            case TypeIndependence:
                return("TypeIndependence");
            }
            throw(E("BG_FactorGraphCreator::SoftPrint() BGFactorGraph_t not handled"));
            return("");
        }

};


#endif /* !BG_FACTORGRAPHCREATOR_H_ */

// Local Variables: ***
// mode:c++ ***
// End: ***
