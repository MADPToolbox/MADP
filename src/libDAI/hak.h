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


#ifndef __defined_libdai_hak_h
#define __defined_libdai_hak_h


#include "daialg.h"
#include "regiongraph.h"
#include "enum.h"


namespace libDAI {


    /// HAK provides an implementation of the single and double-loop algorithms by Heskes, Albers and Kappen
    class HAK : public DAIAlgRG {
        public:
            ENUM(ClustersType,MIN,DELTA,LOOP)

        protected:
            struct {
                ClustersType clusters;
                double       tol;
                size_t       maxiter;
                size_t       verbose;
                double       damping;
                bool         doubleloop;
                size_t       loopdepth;
            } Props;
            /// Maximum difference encountered so far
            double                  _maxdiff;
            /// Number of iterations needed
            size_t                  _iterations;

            std::vector<Factor>     _Qa;
            std::vector<Factor>     _Qb;
            std::vector<Factor>     _muab;
            std::vector<Factor>     _muba;
            

// DAIAlgRG interface 

        public:
            /// Default constructor
            HAK() : DAIAlgRG(), Props(), _maxdiff(0.0), _iterations(0UL), _Qa(), _Qb(), _muab(), _muba() {};
            
            /// Construct HAK object using the specified properties
            HAK( const FactorGraph & fg, const Properties &opts );
            
            /// Copy constructor
            HAK( const HAK & x ) : DAIAlgRG(x), Props(x.Props), _maxdiff(x._maxdiff), _iterations(x._iterations), _Qa(x._Qa), _Qb(x._Qb), _muab(x._muab), _muba(x._muba) {};

            /// Assignment operator
            HAK & operator=( const HAK & x ) {
                if( this != &x ) {
                    DAIAlgRG::operator=( x );
                    Props        = x.Props;
                    _maxdiff     = x._maxdiff;
                    _iterations  = x._iterations;
                    _Qa          = x._Qa;
                    _Qb          = x._Qb;
                    _muab        = x._muab;
                    _muba        = x._muba;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual HAK* clone() const {
                return new HAK(*this);
            }

            /// Create (virtual constructor)
            virtual HAK* create() const {
                return new HAK();
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


// HAK specific stuff
        public:
            /// Construct from RegionGraph
            HAK(const RegionGraph & rg, const Properties &opts);

            Factor & muab( size_t alpha, size_t beta ) { return _muab[grm().ORIR2E(alpha,beta)]; }
            Factor & muba( size_t beta, size_t alpha ) { return _muba[grm().ORIR2E(alpha,beta)]; }

            const Factor& Qa( size_t alpha ) const { return _Qa[alpha]; };
            const Factor& Qb( size_t beta ) const { return _Qb[beta]; };

            double doGBP();
            double doDoubleLoop();

        private:
            void constructMessages();
            void findLoopClusters( const FactorGraph &fg, std::set<VarSet> &allcl, VarSet newcl, const Var & root, size_t length, VarSet vars );
    };


}


#endif
