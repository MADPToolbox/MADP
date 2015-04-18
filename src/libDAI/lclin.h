/*  Copyright (C) 2006,2007  Joris Mooij  [joris at jorismooij dot nl]
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


#ifndef __defined_libdai_lclin_h
#define __defined_libdai_lclin_h


#include "daialg.h"
#include "enum.h"
#include "factorgraph.h"


namespace libDAI {


    class LCLin : public DAIAlgFG {

        public:
            ENUM(CavityType,FULL,PAIR,PAIR2,PAIRFAST,GROUP,HEUR,UNIFORM)
            ENUM(UpdateType,SEQFIX,SEQRND,NONE)

        protected:
            struct {
                UpdateType  updates;
                CavityType  cavity;
                bool        reinit;
                double      cortol;
                double      tol;
                size_t      maxiter;
                size_t      verbose;
                double      damping;
                std::string cavainame;      // FIXME needs assignment operator
                Properties  cavaiopts;      // FIXME needs assignment operator
            } Props;
            /// Maximum difference encountered so far
            double                             _maxdiff;
            /// Number of iterations needed
            size_t                             _iterations;

            /// _phis[edge(i,I)] corresponds to \f$ \phi^{\setminus i}_I(x_{I \setminus i}) \f$
            std::vector<Factor>                _phis;
            
            /// _gamma[i][...] are factors in cavity i that have to be summed
            std::vector<std::vector<Factor > > _gamma;

            /// Single variable beliefs
            std::vector<Factor>                _beliefs;


// DAIAlgFG interface 

        public:
            /// Default constructor
            LCLin() : DAIAlgFG(), Props(), _maxdiff(0.0), _iterations(0UL), _phis(), _gamma(), _beliefs() {};
            
            /// Construct LCLin object using the specified properties
            LCLin( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            LCLin( const LCLin & x ) : DAIAlgFG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _phis(x._phis), _gamma(x._gamma), _beliefs(x._beliefs) {};

            /// Assignment operator
            LCLin & operator=( const LCLin & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props        = x.Props;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                    _phis        = x._phis;
                    _gamma       = x._gamma;
                    _beliefs     = x._beliefs;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual LCLin* clone() const {
                return new LCLin(*this);
            }

            /// Create (virtual constructor)
            virtual LCLin* create() const {
                return new LCLin();
            }
            
            /// Return verbosity level
            virtual size_t Verbose() const {
                return Props.verbose;
            }

            /// Return number of passes over the factorgraph
            virtual size_t Iterations() const {
                return _iterations;
            }

            /// Return maximum difference between single node 
            /// beliefs for two consecutive iterations
            virtual double maxDiff() const {
                return _maxdiff;
            }

            /// Identifies itself for logging purposes
            virtual std::string identify() const;

            /// Get single node belief
            virtual Factor belief( const Var &n ) const { 
                return( _beliefs[grm().findVar(n)] ); 
            }

            /// Get general belief
            virtual Factor belief( const VarSet &n ) const { 
                DAI_THROW(NOT_IMPLEMENTED); 
            }

            /// Get all beliefs
            virtual std::vector<Factor> beliefs() const { 
                return _beliefs; 
            }

            /// Get log partition sum
            virtual Complex logZ() const {
                return NAN;
            }

            /// Clear messages and beliefs
            virtual void init();

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns ) {
                init();
            }

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;


// LCLin specific stuff
        public:
            double CalcCavityDist (size_t i, const std::string &name, const Properties &opts);
            double InitCavityDists (const std::string &name, const Properties &opts);
            void CalcBelief (size_t i);
            const Factor & beliefV (size_t i) const { return _beliefs[i]; };

        private:
            Factor notSumProd (VarSet exceptVars, std::vector<const Factor *> facs);

            void update( size_t i, size_t Y );

            Factor & phi( size_t i, size_t I ) { return _phis[grm().edge(i,I)]; }
            Factor Fdelp( size_t i, const Factor &fac ); 
            Factor Fdel( size_t i );
            Factor Gdele( size_t i, size_t I1 );
            Factor Gdelpe( size_t i, const Factor &fac, size_t I1 );
            Factor Hdelpe( size_t i, const Factor &fac, size_t Y, VarSet ns );

//          void clamp( const Var &n, size_t i ) { DAI_THROW(NOT_IMPLEMENTED); }
//          virtual void makeCavity(const Var & n) { DAI_THROW(NOT_IMPLEMENTED); }

    };


}


#endif
