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


#ifndef __defined_libdai_jtree_h
#define __defined_libdai_jtree_h


#include <vector>
#include "daialg.h"
#include "varset.h"
#include "regiongraph.h"
#include "weightedgraph.h"
#include "enum.h"


namespace libDAI {


    class JTree : public DAIAlgRG {

        public:
            ENUM(UpdateType,HUGIN,SHSH)

        protected:
            struct {
                UpdateType updates;
                size_t     verbose;
            } Props;
            DEdgeVec             _RTree;     // rooted tree
            std::vector<Factor>  _Qa;
            std::vector<Factor>  _Qb;
            std::vector<Factor>  _mes;
            double               _logZ;


// DAIAlgRG interface 

        public:
            /// Default constructor
            JTree() : DAIAlgRG(), Props(), _RTree(), _Qa(), _Qb(), _mes(), _logZ() {};
            
            /// Construct JTree object using the specified properties
            JTree( const FactorGraph & fg, const Properties &opts, bool automatic = true );
            
            /// Copy constructor
            JTree( const JTree & x ) : DAIAlgRG(x), Props(x.Props), _RTree(x._RTree), _Qa(x._Qa), _Qb(x._Qb), _mes(x._mes), _logZ(x._logZ) {};

            /// Assignment operator
            JTree & operator=( const JTree & x ) {
                if( this != &x ) {
                    DAIAlgRG::operator=( x );
                    Props   = x.Props;
                    _RTree  = x._RTree;
                    _Qa     = x._Qa;
                    _Qb     = x._Qb;
                    _mes    = x._mes;
                    _logZ   = x._logZ;
                }
                return *this;
            }

            /// Clone (virtual copy constructor)
            virtual JTree* clone() const {
                return new JTree(*this);
            }

            /// Create (virtual constructor)
            virtual JTree* create() const {
                return new JTree();
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
            virtual void init( const VarSet &ns ) {}

            /// The actual approximate inference algorithm
            virtual double run();

            /// Checks whether all necessary properties have been set
            /// and casts string-valued properties to other values if necessary
            virtual bool initProps();

            /// Name of this inference method
            static const char *Name;


// JTree specific stuff

        public:
            void GenerateJT( const std::vector<VarSet> &Cliques );

            Factor & message(size_t i1, size_t i2) { return( _mes[grm().ORIR2E(i1,i2)] ); }   
            const Factor & message(size_t i1, size_t i2) const { return( _mes[grm().ORIR2E(i1,i2)] ); }   

            void runHUGIN();
            void runShaferShenoy();
            size_t findEfficientTree( const VarSet& ns, DEdgeVec &Tree, size_t PreviousRoot=(size_t)-1 ) const;
            Factor calcMarginal( const VarSet& ns );
    };


}


#endif
