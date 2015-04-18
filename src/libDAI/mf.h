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


#ifndef __defined_libdai_mf_h
#define __defined_libdai_mf_h


#include "daialg.h"
#include "factorgraph.h"


namespace libDAI {


    class MF : public DAIAlgFG {
        protected:
            struct {
                double     tol;
                size_t     maxiter;
                size_t     verbose;
                double     damping;
            } Props;
            /// Maximum difference encountered so far
            double                  _maxdiff;
            /// Number of iterations needed
            size_t                  _iterations;

            std::vector<Factor>     _beliefs;
            

// DAIAlgFG interface 

        public:
            /// Default constructor
            MF() : DAIAlgFG(), Props(), _maxdiff(0.0), _iterations(0UL), _beliefs() {};
            
            /// Construct MF object using the specified properties
            MF( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            MF( const MF & x ) : DAIAlgFG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _beliefs(x._beliefs) {};

            /// Assignment operator
            MF & operator=( const MF & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props        = x.Props;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                    _beliefs     = x._beliefs;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual MF* clone() const {
                return new MF(*this);
            }

            /// Create (virtual constructor)
            virtual MF* create() const {
                return new MF();
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

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;


// MF specific stuff

        public:
            Factor beliefV( size_t i ) const;
    };


}


#endif
