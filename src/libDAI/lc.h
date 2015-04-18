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


#ifndef __defined_libdai_lc_h
#define __defined_libdai_lc_h


#include "daialg.h"
#include "enum.h"
#include "factorgraph.h"
#include "exceptions.h"


namespace libDAI {


    class LC : public DAIAlgFG {

        public:
            ENUM(CavityType,FULL,PAIR,PAIR2,UNIFORM)
            ENUM(UpdateType,SEQFIX,SEQRND,NONE)

        protected:
            struct {
                UpdateType  updates;
                CavityType  cavity;
                bool        reinit;
                double      tol;
                size_t      maxiter;
                size_t      verbose;
                double      damping;
                std::string cavainame;      // FIXME needs assignment operator
                Properties  cavaiopts;      // FIXME needs assignment operator
            } Props;
            /// Maximum difference encountered so far
            double                  _maxdiff;
            /// Number of iterations needed
            size_t                  _iterations;
            std::vector<Factor>     _pancakes;      // used by all LC types (psi_I is stored in the pancake)
            std::vector<Factor>     _cavitydists;   // used by all LC types to store the approximate cavity distribution
            /// _phis[VV2E(i,I)] corresponds to \f$ \phi^{\setminus i}_I(x_{I \setminus i}) \f$
            std::vector<Factor>     _phis;
            /// Single variable beliefs
            std::vector<Factor>     _beliefs;


// DAIAlgFG interface 

        public:
            /// Default constructor
            LC() : DAIAlgFG(), Props(), _maxdiff(0.0), _iterations(0UL), _pancakes(), _cavitydists(), _phis(), _beliefs() {};
            
            /// Construct LC object using the specified properties
            LC( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            LC( const LC & x ) : DAIAlgFG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _pancakes(x._pancakes), _cavitydists(x._cavitydists), _phis(x._phis), _beliefs(x._beliefs) {};

            /// Assignment operator
            LC & operator=( const LC & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props        = x.Props;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                    _pancakes    = x._pancakes;
                    _cavitydists = x._cavitydists;
                    _phis        = x._phis;
                    _beliefs     = x._beliefs;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual LC* clone() const {
                return new LC(*this);
            }

            /// Create (virtual constructor)
            virtual LC* create() const {
                return new LC();
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


// LC specific stuff


        public:

            double CalcCavityDist (size_t i, const std::string &name, const Properties &opts);
            double InitCavityDists (const std::string &name, const Properties &opts);
            long SetCavityDists (std::vector<Factor> &Q);

            Factor NewPancake (size_t iI, bool & hasNaNs);

            void CalcBelief (size_t i);
            const Factor & belief (size_t i) const { return _beliefs[i]; };
            const Factor & pancake (size_t i) const { return _pancakes[i]; };
            const Factor & cavitydist (size_t i) const { return _cavitydists[i]; };

//            void clamp( const Var &n, size_t i ) { DAI_THROW(NOT_IMPLEMENTED); }
//            virtual void makeCavity(const Var & n) { DAI_THROW(NOT_IMPLEMENTED); }
    };


}


#endif
