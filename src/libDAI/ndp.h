/*  Copyright (C) 2008  Frans Oliehoek  [F.A.Oliehoek@uva.nl], Joris Mooij  [joris at jorismooij dot nl]
    Radboud University Nijmegen, The Netherlands
    
    This file is part of libDAI.

    libDAI is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    libDAI is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libDAI; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#ifndef __defined_libdai_ndp_h
#define __defined_libdai_ndp_h

#include <list>            
#include <fstream>
#include <cstdlib>
#include <float.h>
#include "daialg.h"
#include "factorgraph.h"
#include "enum.h"
#include "MADP_util.h"



namespace libDAI {

    typedef FactorGraph::nb_type nb_type;


    class NDP : public DAIAlgFG {
        public:
            //ENUM(UpdateType,SEQFIX,SEQRND,SEQMAX,PARALL)

            static std::string printNB(nb_type n)
            {
                std::stringstream ss;                
                if(n.size() == 0)
                    ss << "< >";
                else
                {
                    ss << "< ";
                    for(size_t i=0; i < n.size() - 1; i++)
                        ss << n.at(i) << ", ";
                    ss << n.at( n.size()-1 ) << " >";
                }
                return(ss.str());
            }

        protected:
            struct {
                //UpdateType updates;
                //double     tol;
                //size_t     maxiter;
                size_t     verbose;
                //double     damping;
            } Props;
            /// Maximum difference encountered so far
            //double                       _maxdiff;
            /// Number of iterations needed
            //size_t                       _iterations;

            //typedef std::vector<size_t>  _ind_t;
            //std::vector<_ind_t>          _indices;
            //std::vector<Prob>            _messages, _newmessages;    //vector of probability vectors (Prob = TProb< Real > and Real = double)

        private:
            class VariableHasLowerDegree{
                const FactorGraph& _m_grm;
            public:
                VariableHasLowerDegree(const FactorGraph& g)
                   : _m_grm(g)
                {};
                bool operator()  (Var i, Var j) const
                { 
                    const FactorGraph& g = _m_grm;
                    size_t ind_i =  g.findVar(i);
                    size_t ind_j =  g.findVar(j);
                    size_t di = g.nbV(ind_i).size();
                    size_t dj = g.nbV(ind_j).size();
                    return (di<dj); 
                }
            };
        //-----------------------------------------------------
        //NDP bookkeeping
            ///keep track of induced width
            size_t _m_induced_width;

            ///the order in which the variable are eliminated:
            std::vector<size_t> _m_varOrder;

            /// the maximum time we are allowed to spend
            double _m_deadlineInSeconds;

            /**\brief All factors introduced in the NDP process
             * __m_newFacs[i] is introduced by the elimination of variable
             * _m_varOrder[i].
             */
            std::vector< Factor > _m_newFacs;

            /**\brief this returns all (original and 'new') factors through
             * a global index.
             */
            const Factor & GetFactor(size_t i) const
            {
                //nr of Original Factors:
                size_t nrOFs = _g->nrFactors();
                if(i < nrOFs)
                    return (_g->factor(i));
                else
                    return ( _m_newFacs.at(i - nrOFs) );
            }
            /**\brief return the global index for the factor introduced in the 
             * elimination of the i-th variable.
             */
            size_t ToGlobalFactorIndex(size_t i) const
            {
                //nr of Original Factors:
                size_t nrOFs = _g->nrFactors();
                size_t gI = nrOFs + i;
                return gI;
            }

            /**\brief the cashed best responses.
             * _m_bestResps[i][jI] contains the state (index) of variable
             * _m_varOrder[i] that is the best response against jtI, the joint
             * state index of its (induced) neighborhood.
             */
            std::vector< std::vector< size_t> > _m_bestResps;

            ///open list:
            //std::vector< Factor > _fac_open;
            ///closed list ?! not necessary I guess?
            //std::vector< Factor > _fac_closed;

            ///_m_neighV[i] contains the neighborhood (factor indices) of var i.
            /**this includes 'new factors', and it is dynamically updates
             * as NDP progresses.
             */
            std::vector< nb_type > _m_neighV;
            
            ///_m_neighF[gI] contains the neighborhood (var indices) of factor gI.
            /**-gI is a 'global' factor index (includes 'new' factors)
             * -excludes eliminated variable, and it is dynamically updates
             * as NDP progresses.
             */
            std::vector< nb_type > _m_neighF;

            //this function returns all factors in which variable varI
            //participates.
            const nb_type & GetNeighsV(size_t varI) const
            {
                return _m_neighV.at(varI);
            }
            
            //this function returns all 'variable neighbors' of v (i.e., neighnbors
            //of neigbors, excluding v itself)
            nb_type GetVNeighsV(size_t varI) const
            {
                std::set<size_t> VNVset;

                //the first degree neighbors are the factors
                const nb_type & nb1 =  _m_neighV.at(varI);
                nb_type::const_iterator it = nb1.begin();
                for(; it < nb1.end(); it++)
                {
                    size_t gI = *it;
                    //find and insert 2nd degree neighbors 
                    const nb_type & nb2  = _m_neighF.at(gI);
                    VNVset.insert( nb2.begin(), nb2.end() );
                }
                //delete varI itself!
                std::set<size_t>::iterator varI_it = VNVset.find( varI );
                VNVset.erase(varI_it);
                //convert to nb_type:
                nb_type VNV( VNVset.begin(), VNVset.end() );
                return VNV;
            }

            VarSet VarSetFromNB(const nb_type & varIs) const
            {
                VarSet vs;
                for(size_t i=0; i<varIs.size(); i++)
                {
                    size_t vI = varIs.at(i);
                    const Var & v = _g->var(vI);
                    vs |= v;
                }
                return vs;
            }

            ///varset -> nb_type (vector of indices)
            nb_type NBFromVarSet(const VarSet & vs) const
            {
                nb_type n(vs.size());
                VarSet::const_iterator it = vs.begin();
                for(size_t i=0; it != vs.end(); it++, i++)
                {
                    const Var & v = *it;
                    size_t vI = _g->findVar(v);
                    n.at(i) = vI;
                }
                return n;
            }

            
            /** sets the neighborhood (for a newly introduced factor) to vs
             */
            void SetNeighF(size_t i_th_var, const VarSet& vs)
            {
                size_t gI = ToGlobalFactorIndex(i_th_var);
                nb_type & nb = _m_neighF.at(gI);
                nb.resize( vs.size() );
                VarSet::const_iterator it = vs.begin();
                for(size_t i=0; it != vs.end(); it++, i++)
                {
                    const Var & v = *it;
                    size_t vI = _g->findVar(v);
                    nb.at(i) = vI;
                }
            }


        //-----------------------------------------------------




// DAIAlgFG interface 

        public:
            /// Default constructor
            NDP() : 
                DAIAlgFG(), 
                Props(), 
                //_maxdiff(0.0), 
                //_iterations(0UL), 
                //_indices(),
                //_messages(), 
                //_newmessages(), 
                _g(NULL),
                _k(1),
                _k_th_Val(-DBL_MAX) //,  
                //writeAnyTimeResults(false),  
                //results_f(NULL), 
                //timings_f(NULL)
            {};
            
            /// Construct NDP object using the specified properties
            NDP( const FactorGraph & fg, const Properties &opts, size_t k=1 );
            
            /// Copy constructor
            NDP( const NDP & x ) : 
                DAIAlgFG(x), 
                Props(x.Props),
                //_maxdiff(x._maxdiff),
                //_iterations(x._iterations),
                //_indices(x._indices),
                //_messages(x._messages),
                //_newmessages(x._newmessages),
                _g(x._g) 
            {};

            /// Assignment operator
            NDP & operator=( const NDP & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props        = x.Props;
                    //_maxdiff     = x._maxdiff;
                    //_iterations  = x._iterations;
                    //_indices     = x._indices;
                    //_messages    = x._messages;
                    //_newmessages = x._newmessages;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual NDP* clone() const {
                return new NDP(*this);
            }

            /// Create (virtual constructor)
            virtual NDP* create() const {
                return new NDP();
            }
            
            /// Return verbosity level
            virtual size_t Verbose() const {
                return Props.verbose;
            }

            /// Return number of passes over the factorgraph
            virtual size_t Iterations() const {
                return 42;
                //return _iterations;
            }

            /// Return maximum difference between single node 
            /// beliefs for two consecutive iterations
            virtual double maxDiff() const {
                return 42.0;
                //return _maxdiff;
            }
            /// Identifies itself for logging purposes
            virtual std::string identify() const;

            /// Get single node belief
            virtual Factor belief( const Var &n ) const;

            /// Get general belief
            virtual Factor belief( const VarSet &n ) const;

            /// Get all beliefs
            virtual std::vector<Factor> beliefs() const;

            /// Get log partition sum
            virtual Complex logZ() const;

            /// Clear messages and beliefs
            virtual void init();

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns );

            /// The actual exact inference algorithm
            /** Note, while the algorithm is exact, the reported value may 
             * be different from the value of the optimal configuration in 
             * case that there are factors that are not connected to any
             * variables!
             */
            virtual double run();

            /// Set the deadline
            virtual void setDeadline(double deadlineInS)
            { _m_deadlineInSeconds=deadlineInS; }

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;
            
            ///Turns Anytime results on and of 
            /**When turning on, valid ofstream pointers must be provided for the
             * results and timings file.
             */
            //void SetAnyTimeResults(bool turn_on, std::ofstream* results, 
                    //std::ofstream* timings);
            
            /**Returns the best configuration 
             * (the vector with the best value ('state') for each variable )
             * Should be called after NDP has run()
             */
            const std::vector<size_t> & GetBestConfiguration()
                {return _m_bestConfiguration;}

            std::list< MADP_util::valConf > & 
                GetBestConfigurations(){return _valConfs;}
            ///Returns the value of the best configuration 
            ///Should be called after NDP has run()
            double GetBestValue() {return bestValue;}
    private:
            ///calling grm() from superclass over and over is costly!, we store a pointer here and implement
            ///a local version of this function
            const FactorGraph * _g;// = grm();
            ///this should be less costly:
            const FactorGraph& grm() const {return *_g;}

            
            void ComputeHeuristicVariableOrdering();
            void EliminateVariable(size_t varIndex);
            double ConstructOptimalAssignments();

            /**\brief Select a state for the _m_varOrder[varOrderI] 
             * I.e., for the variable processed as varOrderI-th in the elimination
             * process
             */
            void SelectStateForVar(size_t varOrderI);

            ///a vector that stores the best configuration found
            ///(for each variable it stores the index of the best value ('state') )
            std::vector<size_t> _m_bestConfiguration;
            ///stores the value of the best configuration found so far.
            double bestValue;

            //the number of best configurations
            size_t _k;
            ///the list of _k best configurations, and their values
            /** .front() gives the valConf with k-th highest value, 
             *  .back() is the best found valConf
             */
            std::list< MADP_util::valConf > _valConfs;
            ///the value of the k-th first entry in _valConfs)
            double _k_th_Val;

/*            
            Factor beliefV (size_t i) const;
            Factor beliefF (size_t I) const;
*/

/*
// maxplus specific stuff
//
//

            ///FRANS: this function computes the new message sent from factor I to variable i
            void calcNewMessage(size_t iI);
            void updateMessage(size_t iI);


            //const Prob & message(size_t i, size_t I) const { return( _messages[grm().edge(i,I)] ); }
            const Prob & message(size_t i, size_t I) const { return( _messages[_g->edge(i,I)] ); }

        private:
            Prob & message(size_t i, size_t I) { return( _messages[_g->edge(i,I)] ); }  
            Prob & newMessage(size_t i, size_t I) { return( _newmessages[_g->edge(i,I)] ); }    
            const Prob & newMessage(size_t i, size_t I) const { return( _newmessages[_g->edge(i,I)] ); }    
            _ind_t & index(size_t i, size_t I) { return( _indices[_g->edge(i,I)] ); }
            const _ind_t & index(size_t i, size_t I) const { return( _indices[_g->edge(i,I)] ); }
        


            ///boolean that indicates whether writeAnyTimeResults results should be written
            bool writeAnyTimeResults;
            ///the file to which writes the results are written
            std::ofstream* results_f;
            ///the file to which writes the timings of the results are written
            std::ofstream* timings_f;
*/

    };


}


#endif
