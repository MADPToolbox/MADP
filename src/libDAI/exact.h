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


#ifndef __defined_libdai_exact_h
#define __defined_libdai_exact_h


#include "daialg.h"
#include "factorgraph.h"
#include "enum.h"


namespace libDAI {


    class Exact : public DAIAlgFG {
        protected:
            struct {
                size_t     verbose;
            } Props;
            std::vector<Factor> _beliefsV;
            std::vector<Factor> _beliefsF;
            Complex             _logZ;

// DAIAlgFG interface 

        public:
            /// Default constructor
            Exact() : DAIAlgFG(), Props() {};
            
            /// Construct Exact object using the specified properties
            Exact( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            Exact( const Exact & x ) : DAIAlgFG(x), Props(x.Props), _beliefsV(x._beliefsV), _beliefsF(x._beliefsF), _logZ(x._logZ) {};

            /// Assignment operator
            Exact & operator=( const Exact & x ) {
                if( this != &x ) {
                    DAIAlgFG::operator=( x );
                    Props     = x.Props;
                    _beliefsV = x._beliefsV;
                    _beliefsF = x._beliefsF;
                    _logZ     = x._logZ;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual Exact* clone() const {
                return new Exact(*this);
            }

            /// Create (virtual constructor)
            virtual Exact* create() const {
                return new Exact();
            }
            
            /// Return verbosity level
            virtual size_t Verbose() const {
                return Props.verbose;
            }

            /// Return number of passes over the factorgraph
            virtual size_t Iterations() const {
                return 1UL;
            }

            /// Return maximum difference between single node 
            /// beliefs for two consecutive iterations
            virtual double maxDiff() const {
                return 0.0;
            }
            
            /// Identifies itself for logging purposes
            virtual std::string identify() const;

            /// Get single node belief
            virtual Factor belief( const Var &n ) const {
                return beliefV( grm().findVar( n ) ); 
            }

            /// Get general belief
            virtual Factor belief( const VarSet &n ) const;

            /// Get all beliefs
            virtual std::vector<Factor> beliefs() const;

            /// Get log partition sum
            virtual Complex logZ() const {
                return _logZ;
            }

            /// Clear messages and beliefs
            virtual void init();

            /// Clear messages and beliefs corresponding to the nodes in ns
            virtual void init( const VarSet &ns ) {}

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;


// Exact specific stuff

        public:
            Factor beliefV (size_t i) const { 
                return _beliefsV[i]; 
            }
            Factor beliefF (size_t I) const { 
                return _beliefsF[I]; 
            }
    };


}


#endif
